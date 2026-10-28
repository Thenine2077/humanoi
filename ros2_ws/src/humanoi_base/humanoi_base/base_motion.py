"""base_motion — สั่งฐาน mecanum วิ่งตามระยะ / หมุนตามมุม แล้วหยุด + lidar กันชน (ใช้ใน base_move และ humanoi_mission)

ฐานจริงคุมได้แค่ทิศ (relay) ความเร็วตั้งด้วยลูกบิด HW-687 → ส่ง /cmd_vel ไปทางที่ต้องการ แล้วตัดเมื่อเหลือระยะ
stop_early (เผื่อไหลหลังตัด relay) · ระยะที่ไปได้วัดจาก odometry: ฐานจริง /wheel_odom (encoder), ซิม /odom
กันชน: จุดจาก lidar ที่อยู่ในทางที่กำลังไป (กรอบตัวหุ่นยืดออกไป stop_dist) = หยุดรอจนโล่ง เกิน block_timeout = เลิก
lidar เงียบ / odometry เงียบ / รีโมทแย่งคุม / สั่งหยุด (stop_event) = หยุดทันทีแล้วแจ้ง MoveError
"""
import math
import threading
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

# ทิศ -> (vx, vy, wz) หน่วย · turn = องศา บวก = หมุนซ้าย (ทวนเข็ม)
KINDS = {"forward": (1, 0, 0), "back": (-1, 0, 0), "left": (0, 1, 0), "right": (0, -1, 0), "turn": (0, 0, 1)}
TH = {"forward": "เดินหน้า", "back": "ถอยหลัง", "left": "สไลด์ซ้าย", "right": "สไลด์ขวา", "turn": "หมุน"}

DEFAULTS = {
    "odom_topic": "/wheel_odom",   # ซิมใช้ /odom
    "cmd_topic": "/cmd_vel",
    "scan_topic": "/scan",
    "base_frame": "base_footprint",
    "linear_speed": 0.2,           # m/s ที่ส่งใน /cmd_vel (ฐานจริงวิ่งตามลูกบิด — ใช้แค่ทิศ)
    "angular_speed": 0.5,          # rad/s
    "expect_linear": 0.15,         # m/s ความเร็วจริงโดยประมาณ — ใช้คิดเวลาสูงสุด
    "expect_angular": 0.4,         # rad/s
    "stop_early": 0.03,            # m ตัดก่อนถึงเท่านี้ (ไหลต่อหลังตัด relay) — ปรับจากผลที่รายงาน
    "stop_early_deg": 3.0,
    "guard": True,                 # false = ไม่ใช้ lidar (ทดสอบเท่านั้น)
    "footprint": [0.255, 0.255, 0.21, 0.21],  # m จาก base_footprint ไป หน้า หลัง ซ้าย ขวา (ยาว 0.51 วัดแล้ว กว้างยังเดา)
    "stop_dist": 0.35,             # m ของในทางใกล้กว่านี้ (จากขอบตัวหุ่น) = หยุดรอ
    "side_margin": 0.05,           # m กรอบทางกว้างกว่าตัวหุ่นข้างละเท่านี้
    "rot_margin": 0.10,            # m ตอนหมุน: ของห่างวงกวาดมุมตัวหุ่นไม่ถึงเท่านี้ = หยุดรอ
    "block_timeout": 15.0,         # s รอทางโล่งนานสุด
    "clear_time": 1.0,             # s ทางต้องโล่งต่อเนื่องเท่านี้ก่อนไปต่อ
    "settle": 0.8,                 # s หลังหยุด รอล้อนิ่งแล้วค่อยวัดผล
    "sensor_timeout": 0.6,         # s odometry / lidar เงียบเกินนี้ = หยุด
}


class MoveError(Exception):
    pass


def declare(node, prefix="base."):
    """ประกาศพารามิเตอร์ของฐาน (prefix base.) แล้วคืน dict ค่าที่ใช้"""
    out = {}
    for k, v in DEFAULTS.items():
        node.declare_parameter(prefix + k, v)
        out[k] = node.get_parameter(prefix + k).value
    return out


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def obstacle_gap(points, kind, footprint, side_margin, rot_margin):
    """ระยะจากขอบตัวหุ่นถึงจุดใกล้สุดที่อยู่ในทางของการวิ่งแบบ kind (m) — ไม่มีจุดในทาง = None
    points = [(x, y)] ในเฟรมฐาน · จุดในกรอบตัวหุ่นเอง (เสา แขน สาย) ไม่นับ
    ตอนหมุน: ระยะเลยวงกวาดมุมตัวหุ่น ลบ rot_margin (≤ 0 = ชนแน่)"""
    f, b, l, r = footprint
    m = side_margin
    corner = math.hypot(max(f, b), max(l, r))
    best = None
    for x, y in points:
        if -b <= x <= f and -r <= y <= l:
            continue
        if kind == "turn":
            gap = math.hypot(x, y) - corner - rot_margin
        elif kind in ("forward", "back"):
            if not -r - m <= y <= l + m:
                continue
            gap = x - f if kind == "forward" else -b - x
        else:
            if not -b - m <= x <= f + m:
                continue
            gap = y - l if kind == "left" else -r - y
        if gap < -0.5:  # อยู่ด้านหลังทางที่ไป
            continue
        best = gap if best is None else min(best, gap)
    return best


class BaseMotion:
    def __init__(self, node, cfg):
        self.node = node
        self.cfg = dict(DEFAULTS, **(cfg or {}))
        self.lock = threading.Lock()
        self.odom = None        # (t, x, y, yaw)
        self.scan = None        # (t, msg)
        self.mode = None        # MODE ของ Mega จาก /motor_state (None = ไม่มี Mega เช่นในซิม)
        self.stop_event = threading.Event()
        self.pub = node.create_publisher(Twist, self.cfg["cmd_topic"], 10)
        node.create_subscription(Odometry, self.cfg["odom_topic"], self._on_odom, 20)
        node.create_subscription(String, "/motor_state", self._on_state, 50)
        if self.cfg["guard"]:
            node.create_subscription(LaserScan, self.cfg["scan_topic"], self._on_scan, 5)
            self.tf = Buffer()
            self.tf_listener = TransformListener(self.tf, node)
        self._tf_warned = False

    # ---------- รับข้อมูล ----------
    def _on_odom(self, msg):
        p = msg.pose.pose
        with self.lock:
            self.odom = (time.monotonic(), p.position.x, p.position.y, yaw_of(p.orientation))

    def _on_scan(self, msg):
        with self.lock:
            self.scan = (time.monotonic(), msg)

    def _on_state(self, msg):
        s = msg.data
        if s.startswith("MODE:"):
            self.mode = s[5:]
        elif s.startswith("RC CH1"):
            for key, mode in (("| ROS", "ROS"), ("| RC", "RC"), ("| NO SIGNAL", "NO_SIGNAL"),
                              ("| disarmed", "DISARMED"), ("| SWA up first", "DISARMED")):
                if key in s:
                    self.mode = mode
                    break

    # ---------- lidar ----------
    def scan_points(self, msg):
        """จุดจาก LaserScan ในเฟรมฐาน (ใช้ TF; ไม่มี TF = lidar ตั้งตามค่าใน lidar.launch.py)"""
        try:
            tf = self.tf.lookup_transform(self.cfg["base_frame"], msg.header.frame_id, Time())
            t, q = tf.transform.translation, tf.transform.rotation
            tx, ty, tyaw = t.x, t.y, yaw_of(q)
        except TransformException:
            if not self._tf_warned:
                self._tf_warned = True
                self.node.get_logger().warn(f"ไม่มี TF {self.cfg['base_frame']} -> {msg.header.frame_id} "
                                            "— ถือว่า lidar อยู่หน้าฐาน 0.16 m หันหน้าตรง")
            tx, ty, tyaw = 0.16, 0.0, 0.0
        c, s = math.cos(tyaw), math.sin(tyaw)
        pts = []
        a = msg.angle_min
        for rng in msg.ranges:
            if math.isfinite(rng) and msg.range_min <= rng <= msg.range_max:
                lx, ly = rng * math.cos(a), rng * math.sin(a)
                pts.append((tx + c * lx - s * ly, ty + s * lx + c * ly))
            a += msg.angle_increment
        return pts

    def gap(self, kind):
        """(ระยะของขวางในทาง หรือ None, lidar สดไหม)"""
        with self.lock:
            scan = self.scan
        if scan is None or time.monotonic() - scan[0] > self.cfg["sensor_timeout"]:
            return None, False
        g = obstacle_gap(self.scan_points(scan[1]), kind, self.cfg["footprint"],
                         self.cfg["side_margin"], self.cfg["rot_margin"])
        return g, True

    def blocked(self, gap, kind):
        if gap is None:
            return False
        return gap < (0.0 if kind == "turn" else self.cfg["stop_dist"])

    # ---------- สั่ง ----------
    def send(self, vx=0.0, vy=0.0, wz=0.0):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        self.pub.publish(t)

    def halt(self, seconds=0.0):
        """ส่งหยุดซ้ำๆ (relay_bridge หยุดเองถ้าเงียบ แต่หยุดทันทีดีกว่า)"""
        t_end = time.monotonic() + seconds
        while True:
            self.send()
            if time.monotonic() >= t_end:
                return
            time.sleep(0.05)

    def pose(self):
        with self.lock:
            return self.odom

    def wait_ready(self, timeout=5.0):
        """รอ odometry + lidar + TF ของ lidar (โหนดเพิ่งเปิด ข้อความแรกอาจมาช้าเป็นวินาที)"""
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            o = self.pose()
            with self.lock:
                scan = self.scan
            scan_ok = not self.cfg["guard"] or (
                scan is not None and time.monotonic() - scan[0] <= self.cfg["sensor_timeout"]
                and (self.tf.can_transform(self.cfg["base_frame"], scan[1].header.frame_id, Time())
                     or time.monotonic() > t_end - 1.0))  # ไม่มี TF จริงๆ ก็ไปต่อด้วยค่าตั้งต้น (เตือน)
            if o and time.monotonic() - o[0] <= self.cfg["sensor_timeout"] and scan_ok:
                return
            time.sleep(0.05)
        o = self.pose()
        if not o or time.monotonic() - o[0] > self.cfg["sensor_timeout"]:
            raise MoveError(f"ไม่มี odometry ที่ {self.cfg['odom_topic']} — เปิดฐาน (arm base) แล้ว encoder ต่อแล้วหรือยัง")
        raise MoveError(f"ไม่มี lidar ที่ {self.cfg['scan_topic']} — เปิด lidar ก่อน (ฐานไม่วิ่งถ้าไม่มีตัวกันชน)")

    def move(self, kind, amount, log=print):
        """วิ่ง/หมุนแล้วหยุด — kind ตาม KINDS, amount = m (turn = องศา ลบ = หมุนขวา) · คืนผลที่ทำได้จริง"""
        if kind not in KINDS:
            raise MoveError(f"ไม่รู้จัก '{kind}' — ใช้ {', '.join(KINDS)}")
        amount = float(amount)
        if kind != "turn" and amount < 0:
            kind, amount = {"forward": "back", "back": "forward", "left": "right", "right": "left"}[kind], -amount
        sign = 1.0 if amount >= 0 else -1.0
        target = math.radians(abs(amount)) if kind == "turn" else amount
        if target <= 0:
            return {"kind": kind, "target": 0.0, "done": 0.0}
        if self.mode in ("RC", "DISARMED", "NO_SIGNAL"):
            raise MoveError(f"Mega อยู่โหมด {self.mode} — ROS ขับไม่ได้ (รีโมทเปิด, SWA ลง, ปล่อยคันโยก)")
        self.stop_event.clear()
        self.wait_ready()
        dx, dy, dw = KINDS[kind]
        if kind == "turn":
            dw = sign
            twist = (0.0, 0.0, sign * self.cfg["angular_speed"])
            early = math.radians(self.cfg["stop_early_deg"])
            limit = target / self.cfg["expect_angular"] * 2.0 + 5.0
        else:
            twist = (dx * self.cfg["linear_speed"], dy * self.cfg["linear_speed"], 0.0)
            early = self.cfg["stop_early"]
            limit = target / self.cfg["expect_linear"] * 2.0 + 5.0
        _, x0, y0, yaw0 = self.pose()
        state = {"yaw": yaw0, "turned": 0.0}

        def progress():
            _, x, y, yaw = self.pose()
            state["turned"] += wrap(yaw - state["yaw"])
            state["yaw"] = yaw
            px, py = x - x0, y - y0
            fx = math.cos(yaw0) * px + math.sin(yaw0) * py     # ในเฟรมตอนเริ่ม
            fy = -math.sin(yaw0) * px + math.cos(yaw0) * py
            if kind == "turn":
                return sign * state["turned"], math.hypot(fx, fy), state["turned"]
            return fx * dx + fy * dy, -fx * dy + fy * dx, state["turned"]

        moving_time, blocked_since, clear_since, last = 0.0, None, None, time.monotonic()
        try:
            while True:
                time.sleep(0.05)
                now = time.monotonic()
                dt, last = now - last, now
                if self.stop_event.is_set():
                    raise MoveError("หยุดแล้ว (สั่งหยุด)")
                if self.mode in ("RC", "DISARMED", "NO_SIGNAL"):
                    raise MoveError(f"หยุด — Mega เปลี่ยนเป็นโหมด {self.mode} (รีโมทแย่งคุม/ปลด ARM)")
                o = self.pose()
                if now - o[0] > self.cfg["sensor_timeout"]:
                    raise MoveError(f"odometry เงียบ ({self.cfg['odom_topic']}) — หยุด")
                done, side, turned = progress()
                if done >= target - early:
                    break
                if self.cfg["guard"]:
                    g, fresh = self.gap(kind)
                    if not fresh:
                        raise MoveError(f"lidar เงียบ ({self.cfg['scan_topic']}) — หยุด")
                    if self.blocked(g, kind):
                        clear_since = None
                        self.send()
                        if blocked_since is None:
                            blocked_since = now
                            log(f"  ! มีของขวางทาง{TH[kind]} ห่าง {max(g, 0.0):.2f} m — หยุดรอ")
                        if now - blocked_since > self.cfg["block_timeout"]:
                            raise MoveError(f"ทางไม่โล่งนานเกิน {self.cfg['block_timeout']:.0f} วิ — เลิก "
                                            f"(ไปได้ {self._fmt(kind, done)} จาก {self._fmt(kind, target)})")
                        continue
                    if blocked_since is not None:
                        clear_since = clear_since or now
                        if now - clear_since < self.cfg["clear_time"]:
                            self.send()
                            continue
                        log("  ทางโล่งแล้ว — ไปต่อ")
                        blocked_since = None
                moving_time += dt
                if moving_time > limit:
                    raise MoveError(f"ช้าผิดปกติ — {moving_time:.0f} วิแล้วไปได้แค่ {self._fmt(kind, done)} "
                                    f"จาก {self._fmt(kind, target)} (ล้อหมุนไหม / Mega อยู่โหมด ROS ไหม)")
                self.send(*twist)
        finally:
            self.halt(self.cfg["settle"])
        done, side, turned = progress()
        return {"kind": kind, "target": target, "done": done, "side": side, "turned": turned}

    @staticmethod
    def _fmt(kind, v):
        return f"{math.degrees(v):.1f}°" if kind == "turn" else f"{v:.2f} m"

    def describe(self, res):
        k = res["kind"]
        if not res["target"]:
            return "ไม่ต้องขยับ"
        err = res["done"] - res["target"]
        if k == "turn":
            return (f"{TH[k]} {math.degrees(res['done']):.1f}° จาก {math.degrees(res['target']):.1f}° "
                    f"({'เลย' if err > 0 else 'ขาด'} {abs(math.degrees(err)):.1f}°) · ตัวเลื่อน {100 * res['side']:.1f} cm")
        return (f"{TH[k]} {res['done']:.3f} m จาก {res['target']:.3f} m ({'เลย' if err > 0 else 'ขาด'} "
                f"{100 * abs(err):.1f} cm) · เบี้ยวข้าง {100 * res['side']:+.1f} cm · หัวหมุน {math.degrees(res['turned']):+.1f}°")
