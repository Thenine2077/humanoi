"""กันชน lidar ชั้น Mega — คิดว่าทิศไหนมีของขวาง แล้ว relay_bridge ส่ง "G <บิต>" ให้ Mega ทุกรอบ (20 Hz)

Mega (firmware/humanoi_base_mega/guard.h) ไม่ขับไปทิศที่ห้าม ไม่ว่าคำสั่งมาจากรีโมทหรือ ROS — ชั้นสุดท้าย
base_motion (arm drive / mission) มีกันชนของตัวเองที่ระยะไกลกว่า (หยุดรอ/เลิกเอง) ตัวนี้คือเส้นห้ามเด็ดขาด

บิต: 1 หน้า · 2 หลัง · 4 ซ้าย · 8 ขวา · 16 หมุน
ห้าม: ของในทางที่ไป ใกล้กว่า stop_dist จากขอบตัวหุ่น · หมุน: ของในวงกวาดมุมตัวหุ่น + rot_margin
ปลด: ไกลเกิน stop_dist + release_margin ต่อเนื่อง clear_time (กัน relay กระพริบตอนของอยู่ตรงขอบ)
ไม่มี lidar (/scan เงียบเกิน scan_timeout หรือยังไม่เคยมา) = ห้ามทุกทิศ
เข้าจอดหน้าโต๊ะ (base_motion.approach / arm show dock): ส่ง /base_guard/dock (Float32 m) ซ้ำๆ = ระยะห้ามเดินหน้า
  ลดลงเหลือค่านั้นชั่วคราว (ไม่ต่ำกว่า DOCK_MIN) — ทิศอื่นไม่เปลี่ยน · หยุดส่งเกิน DOCK_HOLD = กลับ stop_dist เอง
"""
import time

from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener

from humanoi_base import lidar_mount
from humanoi_base.base_motion import obstacle_gap, scan_to_points, yaw_of

FWD, BACK, LEFT, RIGHT, TURN = 1, 2, 4, 8, 16
ALL = FWD | BACK | LEFT | RIGHT | TURN
BITS = (("forward", FWD), ("back", BACK), ("left", LEFT), ("right", RIGHT), ("turn", TURN))
DOCK_MIN = 0.05    # m ระยะห้ามเดินหน้าตอนจอด ต่ำสุดที่ยอม
DOCK_HOLD = 1.0    # s คำสั่งจอดหนึ่งครั้งใช้ได้นานเท่านี้ (ต้องส่งซ้ำ)

DEFAULTS = {
    "enabled": True,                 # false = ส่ง "G OFF" (Mega ไม่กัน) — arm base noguard
    "scan_topic": "/scan",
    "footprint": [0.265, 0.265, 0.2125, 0.2125],  # m จากกลางฐานไป หน้า หลัง ซ้าย ขวา (เท่า base_motion)
    "stop_dist": 0.30,               # m ห้ามเมื่อของใกล้กว่านี้ (base_motion หยุดรอที่ 0.35 ก่อนถึงตรงนี้)
    "side_margin": 0.05,             # m ทางกว้างกว่าตัวหุ่นข้างละเท่านี้
    "rot_margin": 0.05,              # m ห้ามหมุนเมื่อของห่างวงกวาดมุมตัวหุ่นไม่ถึงเท่านี้
    "release_margin": 0.05,          # m ต้องห่างเกิน stop_dist + เท่านี้ถึงปลด
    "clear_time": 0.5,               # s ต้องโล่งต่อเนื่องเท่านี้ถึงปลด
    "scan_timeout": 0.5,             # s /scan เงียบเกินนี้ = ห้ามทุกทิศ
}


def mask_text(mask):
    """บิต -> "F---T" (ตัวอักษร = ทิศที่ห้าม เหมือนที่ Mega รายงาน)"""
    return "".join(c if mask & bit else "-" for c, (_, bit) in zip("FBLRT", BITS))


class GuardLogic:
    """ทิศไหนห้าม จากระยะของขวางแต่ละทิศ + กันกระพริบ (ไม่ใช้ ROS — ทดสอบตรงๆ ได้)"""

    def __init__(self, cfg):
        self.cfg = dict(DEFAULTS, **(cfg or {}))
        self.mask = ALL          # ยังไม่เห็น lidar = ห้ามหมด
        self.clear_since = {}    # บิต -> เวลาที่เริ่มโล่งพอจะปลด
        self.dock = (None, 0.0)  # (ระยะห้ามเดินหน้าตอนจอด, ใช้ได้ถึงเวลา)

    def set_dock(self, dist, now, hold=DOCK_HOLD):
        """เข้าจอด: ห้ามเดินหน้าเมื่อใกล้กว่า dist แทน stop_dist ช่วง hold วินาที (ไม่เกิน stop_dist ไม่ต่ำกว่า DOCK_MIN)"""
        self.dock = (max(DOCK_MIN, min(float(dist), self.cfg["stop_dist"])), now + hold)

    def docking(self, now):
        return self.dock[0] is not None and now < self.dock[1]

    def limit(self, kind, now):
        """ระยะห้ามของทิศนี้ตอนนี้ (หมุน = 0 เพราะ gap หัก rot_margin แล้ว)"""
        if kind == "turn":
            return 0.0
        if kind == "forward" and self.docking(now):
            return self.dock[0]
        return self.cfg["stop_dist"]

    def gaps(self, points):
        """ระยะของขวางแต่ละทิศ {kind: m | None} จากจุดในเฟรมฐาน"""
        c = self.cfg
        return {kind: obstacle_gap(points, kind, c["footprint"], c["side_margin"], c["rot_margin"])
                for kind, _ in BITS}

    def update(self, gaps, now):
        """gaps: {kind: ระยะ m หรือ None = ไม่มีของในทาง} -> mask ใหม่"""
        mask = 0
        for kind, bit in BITS:
            g = gaps.get(kind)
            limit = self.limit(kind, now)
            if g is not None and g < limit:
                self.clear_since.pop(bit, None)
                mask |= bit
            elif self.mask & bit:  # ห้ามอยู่ — ปลดเมื่อโล่งพ้นแถบกันกระพริบนานพอ
                if g is not None and g < limit + self.cfg["release_margin"]:
                    self.clear_since.pop(bit, None)
                    mask |= bit
                elif now - self.clear_since.setdefault(bit, now) < self.cfg["clear_time"]:
                    mask |= bit
                else:
                    self.clear_since.pop(bit, None)
        self.mask = mask
        return mask

    def lost(self):
        """ไม่มี lidar -> ห้ามทุกทิศ และต้องโล่งนับใหม่ตั้งแต่ต้น"""
        self.mask = ALL
        self.clear_since.clear()
        return ALL


class LidarGuard:
    """ฝั่ง ROS: ฟัง /scan + TF ของ lidar -> GuardLogic"""

    def __init__(self, node, base_frame, cfg):
        self.node = node
        self.base_frame = base_frame
        self.logic = GuardLogic(cfg)
        self.cfg = self.logic.cfg
        self.last_scan = None
        self.mount = lidar_mount.load()  # ใช้ตอนไม่มี TF ของ lidar
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, node)
        self._tf_warned = False
        node.create_subscription(LaserScan, self.cfg["scan_topic"], self.on_scan, 5)
        node.create_subscription(Float32, "/base_guard/dock", self.on_dock, 10)

    def on_dock(self, msg):
        was = self.logic.docking(time.monotonic())
        self.logic.set_dock(msg.data, time.monotonic())
        if not was:
            self.node.get_logger().info(f"กันชน: เข้าจอด — ห้ามเดินหน้าเมื่อใกล้กว่า {self.logic.dock[0]:.2f} m "
                                        f"(ปกติ {self.cfg['stop_dist']:.2f} m) จนกว่าคำสั่งจอดจะหยุด")

    def lidar_pose(self, frame):
        try:
            tf = self.tf.lookup_transform(self.base_frame, frame, Time())
            t = tf.transform.translation
            return t.x, t.y, yaw_of(tf.transform.rotation)
        except TransformException:
            if not self._tf_warned:
                self._tf_warned = True
                self.node.get_logger().warn(f"กันชน: ไม่มี TF {self.base_frame} -> {frame} — ใช้ตำแหน่งใน "
                                            f"lidar_mount.yaml (หน้ากลางฐาน {self.mount['x']:.3f} m "
                                            f"หัน {self.mount['yaw_deg']:+.1f}°)")
            return self.mount["x"], self.mount["y"], self.mount["yaw"]

    def on_scan(self, msg):
        pts = scan_to_points(msg, *self.lidar_pose(msg.header.frame_id))
        now = time.monotonic()
        self.logic.update(self.logic.gaps(pts), now)
        self.last_scan = now

    def mask(self, now):
        """(บิตที่ห้าม, lidar สดไหม)"""
        if self.last_scan is None or now - self.last_scan > self.cfg["scan_timeout"]:
            return self.logic.lost(), False
        return self.logic.mask, True
