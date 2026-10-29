#!/usr/bin/env python3
"""ทดสอบกันชน lidar ครบวงโดยไม่ใช้ฮาร์ดแวร์
  mega_sim (เฟิร์มแวร์ Mega ตัวจริง) <-pty-> relay_bridge (ตัวจริง) <- /scan ปลอม + /cmd_vel + รีโมทปลอม (ไฟล์)

ใช้ใน Linux/WSL ที่มี ROS 2 Jazzy (รันผ่าน tools/mega_sim/run_tests.sh ซึ่ง build ให้ก่อน):
  python3 tools/mega_sim/test_guard_e2e.py <mega_sim> <guard_test>
"""
import math
import os
import re
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, "..", "..", "ros2_ws", "src", "humanoi_base")
sys.path.insert(0, PKG)
from humanoi_base.guard import ALL, BACK, FWD, LEFT, RIGHT, TURN, GuardLogic  # noqa: E402
from humanoi_base.relay_bridge import RelayBridge  # noqa: E402

FRONT, SIDE = 0.265, 0.2125   # footprint ใน base.yaml
LIDAR_X = 0.16                # ไม่มี TF -> relay_bridge ถือว่า lidar อยู่ตรงนี้
RC_CENTER, SWA_UP, SWA_DOWN = 1500, 2000, 1000

results = []


def check(name, ok, detail=""):
    results.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name}{('  — ' + detail) if detail else ''}", flush=True)


# ------------------------------------------------------------------ ส่วนที่ไม่ต้องรันอะไร
def unit_tests(guard_test):
    # สูตรล้อฝั่ง relay_bridge กับฝั่ง Mega ต้องเห็นทิศตรงกัน
    rb = SimpleNamespace(geom=0.385, r=0.0725)
    for name, twist, bit in (("forward", (0.2, 0, 0), FWD), ("back", (-0.2, 0, 0), BACK),
                             ("left", (0, 0.2, 0), LEFT), ("right", (0, -0.2, 0), RIGHT),
                             ("turn left", (0, 0, 0.5), TURN), ("turn right", (0, 0, -0.5), TURN)):
        w = RelayBridge._mecanum_wheels(rb, *twist)
        pat = "".join("F" if v > 0.05 else ("B" if v < -0.05 else "S") for v in w)
        got = int(subprocess.check_output([guard_test, "dirs", pat]).decode())
        check(f"relay_bridge {name} -> ล้อ {pat} -> Mega เห็นทิศ {got}", got == bit)

    # GuardLogic: ห้ามทันที ปลดช้า (กันกระพริบ)
    g = GuardLogic({})
    check("เริ่มต้น ห้ามทุกทิศจนกว่าจะเห็น lidar", g.mask == ALL)
    clear = {"forward": None, "back": None, "left": None, "right": None, "turn": None}
    check("โล่งแต่ยังไม่ครบ clear_time = ยังห้าม", g.update(clear, 0.0) == ALL and g.update(clear, 0.4) == ALL)
    check("โล่งครบ 0.5 วิ = ปลด", g.update(clear, 0.51) == 0)
    check("ของเข้ามา 0.25 m ข้างหน้า = ห้ามทันที", g.update(dict(clear, forward=0.25), 0.6) == FWD)
    check("ถอยไปอยู่ 0.32 m (แถบกันกระพริบ) = ยังห้าม", g.update(dict(clear, forward=0.32), 2.0) == FWD)
    g.update(dict(clear, forward=0.40), 2.1)
    check("ไกล 0.40 m ยังไม่ครบ 0.5 วิ = ยังห้าม", g.update(dict(clear, forward=0.40), 2.5) == FWD)
    check("ไกล 0.40 m ครบ 0.5 วิ = ปลด", g.update(dict(clear, forward=0.40), 2.61) == 0)
    check("หมุน: gap < 0 = ห้ามหมุน", g.update(dict(clear, turn=-0.01), 3.0) == TURN)
    check("lidar หาย = ห้ามทุกทิศ", g.lost() == ALL)


# ------------------------------------------------------------------ วงจริง
class Rig(Node):
    """/scan ปลอม (ของ 1 จุดในเฟรมฐาน) + /cmd_vel + จด /base_guard"""

    def __init__(self):
        super().__init__("guard_e2e")
        self.obstacle = None   # (x, y) ในเฟรมฐาน
        self.scan_on = True
        self.cmd = None        # (vx, vy, wz) หรือ None = ไม่ส่ง
        self.guard = []
        self.scan_pub = self.create_publisher(LaserScan, "scan", 5)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(UInt8, "base_guard", lambda m: self.guard.append(m.data), 10)
        self.create_timer(0.1, self.tick)

    def tick(self):
        if self.scan_on:
            n = 360
            msg = LaserScan()
            msg.header.frame_id = "laser_link"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.angle_min, msg.angle_max = -math.pi, math.pi
            msg.angle_increment = 2 * math.pi / n
            msg.range_min, msg.range_max = 0.03, 12.0
            ranges = [float("inf")] * n
            if self.obstacle:
                lx, ly = self.obstacle[0] - LIDAR_X, self.obstacle[1]
                i = round((math.atan2(ly, lx) - msg.angle_min) / msg.angle_increment) % n
                ranges[i] = math.hypot(lx, ly)
            msg.ranges = ranges
            self.scan_pub.publish(msg)
        if self.cmd:
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = self.cmd
            self.cmd_pub.publish(t)


class MegaLog:
    def __init__(self, proc):
        self.lines = []  # (เวลา, บรรทัด)
        self.lock = threading.Lock()
        threading.Thread(target=self._read, args=(proc,), daemon=True).start()

    def _read(self, proc):
        for raw in proc.stdout:
            line = raw.decode(errors="replace").strip().split(" ", 1)[-1]
            with self.lock:
                self.lines.append((time.monotonic(), line))

    def mark(self):
        with self.lock:
            return len(self.lines)

    def wait(self, pattern, timeout, since):
        """รอบรรทัดที่ตรง pattern (นับตั้งแต่ since) -> เวลาที่เจอ หรือ None"""
        rx = re.compile(pattern)
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            with self.lock:
                for t, line in self.lines[since:]:
                    if rx.search(line):
                        return t
            time.sleep(0.02)
        return None

    def state(self):
        with self.lock:
            for _, line in reversed(self.lines):
                if line.startswith("STATE:"):
                    return line[6:]
        return "SSSS"

    def tail(self, n=8):
        with self.lock:
            return " | ".join(line for _, line in self.lines[-n:])


def e2e(mega_sim):
    tmp = tempfile.mkdtemp(prefix="guard_e2e_")
    port_file, rc_file = os.path.join(tmp, "port"), os.path.join(tmp, "rc")

    def rc(ch1=RC_CENTER, ch2=RC_CENTER, ch4=RC_CENTER, swa=SWA_DOWN):
        with open(rc_file + ".tmp", "w") as f:
            f.write(f"{ch1} {ch2} {ch4} {swa} 1000 1000\n")
        os.replace(rc_file + ".tmp", rc_file)

    rc(swa=SWA_UP)
    mega = subprocess.Popen([mega_sim, port_file, rc_file], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    log = MegaLog(mega)
    time.sleep(0.6)
    rc(swa=SWA_DOWN)  # เห็น SWA ขึ้นแล้วลง = ARM, คันโยกกลาง = ROS คุม
    port = open(port_file).read().strip()

    env = dict(os.environ, PYTHONPATH=PKG + os.pathsep + os.environ.get("PYTHONPATH", ""))
    params = os.path.join(PKG, "config", "base.yaml")

    def start_bridge(*extra):
        return subprocess.Popen([sys.executable, "-m", "humanoi_base.relay_bridge", "--ros-args",
                                 "--params-file", params, "-p", f"port:={port}", *extra],
                                env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    bridge = start_bridge()
    rclpy.init()
    rig = Rig()
    ex = SingleThreadedExecutor()
    ex.add_node(rig)
    spinner = threading.Thread(target=ex.spin, daemon=True)
    spinner.start()
    procs = [mega, bridge]
    try:
        m = log.mark()
        ok = log.wait(r"^GUARD:ON -----$", 10.0, m)
        check("relay_bridge ต่อ Mega จำลองได้ + lidar โล่ง -> GUARD:ON -----", ok is not None, log.tail())
        check("MODE:ROS (SWA ลง คันโยกกลาง)", log.wait(r"^MODE:ROS$", 2.0, 0) is not None, log.tail())

        # 1) ROS เดินหน้าทางโล่ง
        m = log.mark()
        rig.cmd = (0.2, 0.0, 0.0)
        check("ROS เดินหน้า ทางโล่ง -> ล้อ FFFF", log.wait(r"^STATE:FFFF$", 3.0, m) is not None, log.tail())

        # 2) ของโผล่ข้างหน้า 0.15 m -> หยุดทันที
        m = log.mark()
        t0 = time.monotonic()
        rig.obstacle = (FRONT + 0.15, 0.0)
        t = log.wait(r"^STATE:SSSS$", 2.0, m)
        check("ของโผล่หน้า 0.15 m ตอนวิ่ง -> หยุด", t is not None,
              f"{(t - t0) * 1000:.0f} ms จากตอนของโผล่ (รวมรอบ lidar 10 Hz)" if t else log.tail())
        check("Mega รายงาน GUARD:ON F---- + HOLD F----",
              log.wait(r"^GUARD:ON F----$", 0.5, m) is not None and log.wait(r"^GUARD:HOLD F----$", 0.5, m) is not None,
              log.tail())

        # 3) ถอยออกได้ ทั้งที่ข้างหน้ายังมีของ
        m = log.mark()
        rig.cmd = (-0.2, 0.0, 0.0)
        check("ROS ถอยหลัง (ทางหลังโล่ง) -> ล้อ BBBB", log.wait(r"^STATE:BBBB$", 3.0, m) is not None, log.tail())

        # 4) กลับมาเดินหน้า = ถูกห้าม, ของหายไป = ไปต่อหลังโล่งครบ clear_time
        m = log.mark()
        rig.cmd = (0.2, 0.0, 0.0)
        time.sleep(1.5)
        check("ROS เดินหน้าใส่ของ -> ล้อไม่หมุนเดินหน้า", "F" not in log.state(), f"STATE {log.state()}")
        m = log.mark()
        t0 = time.monotonic()
        rig.obstacle = None
        t = log.wait(r"^STATE:FFFF$", 3.0, m)
        check("ของหายไป -> เดินหน้าต่อ (รอโล่งครบ 0.5 วิ + กลับทาง 0.5 วิ)", t is not None and t - t0 >= 0.5,
              f"{(t - t0) * 1000:.0f} ms" if t else log.tail())

        # 5) ของด้านซ้าย: สไลด์ซ้าย/หมุนห้าม สไลด์ขวาได้
        m = log.mark()
        rig.obstacle = (0.0, SIDE + 0.15)
        check("ของด้านซ้าย 0.15 m -> ห้ามซ้าย + หมุน", log.wait(r"^GUARD:ON --L-T$", 2.0, m) is not None, log.tail())
        rig.cmd = (0.0, 0.2, 0.0)
        time.sleep(1.5)
        check("ROS สไลด์ซ้ายใส่ของ -> ไม่หมุน", log.state() == "SSSS", f"STATE {log.state()}")
        m = log.mark()
        rig.cmd = (0.0, -0.2, 0.0)
        check("ROS สไลด์ขวาหนีของ -> ล้อ FBBF", log.wait(r"^STATE:FBBF$", 3.0, m) is not None, log.tail())
        rig.cmd = None
        rig.obstacle = None
        time.sleep(1.5)

        # 6) รีโมทก็โดนกัน
        m = log.mark()
        rig.obstacle = (FRONT + 0.10, 0.0)
        log.wait(r"^GUARD:ON F", 2.0, m)
        m = log.mark()
        rc(ch2=2000)  # คันโยกขวาดันขึ้น = เดินหน้า
        check("รีโมทแย่งคุม (MODE:RC)", log.wait(r"^MODE:RC$", 2.0, m) is not None, log.tail())
        time.sleep(1.0)
        check("รีโมทเดินหน้าใส่ของ -> ไม่หมุน + HOLD", log.state() == "SSSS"
              and log.wait(r"^GUARD:HOLD F", 0.1, m) is not None, f"STATE {log.state()} · {log.tail()}")
        m = log.mark()
        rc(ch2=1000)  # ดึงลง = ถอย
        check("รีโมทถอยหนีของ -> ล้อ BBBB", log.wait(r"^STATE:BBBB$", 3.0, m) is not None, log.tail())

        # 7) lidar เงียบ -> relay_bridge ส่งห้ามทุกทิศ
        m = log.mark()
        rig.scan_on = False
        t0 = time.monotonic()
        t = log.wait(r"^STATE:SSSS$", 3.0, m)
        check("lidar เงียบตอนรีโมทถอยอยู่ -> หยุด + GUARD:ON FBLRT",
              t is not None and log.wait(r"^GUARD:ON FBLRT$", 0.5, m) is not None,
              f"{(t - t0) * 1000:.0f} ms" if t else log.tail())
        check("/base_guard = 31 ตอนไม่มี lidar", bool(rig.guard) and rig.guard[-1] == ALL, str(rig.guard[-3:]))

        # 8) relay_bridge ตาย (kill -9) -> Mega LOST เอง, รีโมทขับไม่ได้จนโยก SWA ขึ้น-ลง
        rc()  # ปล่อยคันโยก
        rig.scan_on = True
        time.sleep(1.0)
        m = log.mark()
        bridge.kill()
        bridge.wait()
        t0 = time.monotonic()
        t = log.wait(r"^GUARD:LOST$", 2.0, m)
        check("relay_bridge ตาย -> GUARD:LOST", t is not None, f"{(t - t0) * 1000:.0f} ms" if t else log.tail())
        m = log.mark()
        rc(ch2=2000)
        time.sleep(1.2)
        check("LOST: รีโมทเดินหน้า -> ไม่หมุน", log.state() == "SSSS", f"STATE {log.state()}")
        rc(ch2=2000, swa=SWA_UP)
        check("โยก SWA ขึ้น -> GUARD:OFF (ปลด)", log.wait(r"^GUARD:OFF$", 2.0, m) is not None, log.tail())
        m = log.mark()
        rc(ch2=1500, swa=SWA_DOWN)
        time.sleep(0.4)
        rc(ch2=2000, swa=SWA_DOWN)
        check("SWA ลง + รีโมทเดินหน้า -> ล้อ FFFF (ขับโดยไม่มี lidar)",
              log.wait(r"^STATE:FFFF$", 3.0, m) is not None, log.tail())
        rc()
        time.sleep(1.0)

        # 9) noguard -> G OFF, relay_bridge ตายก็ไม่ LOST
        m = log.mark()
        bridge = start_bridge("-p", "guard.enabled:=false")
        procs.append(bridge)
        time.sleep(4.0)
        rig.obstacle = (FRONT + 0.05, 0.0)
        rig.cmd = (0.2, 0.0, 0.0)
        check("noguard: ROS เดินหน้าใส่ของได้ (G OFF)", log.wait(r"^STATE:FFFF$", 4.0, m) is not None, log.tail())
        rig.cmd = None
        m = log.mark()
        bridge.kill()
        bridge.wait()
        time.sleep(1.2)
        check("noguard: relay_bridge ตายแล้วไม่ LOST", log.wait(r"^GUARD:LOST$", 0.1, m) is None, log.tail())
    finally:
        for p in procs:
            if p.poll() is None:
                p.kill()
        ex.shutdown()
        spinner.join(timeout=2.0)  # ปิด ROS ตอนเธรด spin ยังวิ่ง = โปรแกรมล้มตอนออก (terminate called ...)
        rig.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    unit_tests(sys.argv[2])
    e2e(sys.argv[1])
    bad = results.count(False)
    print(f"\n{len(results) - bad}/{len(results)} ผ่าน" + (f" — {bad} FAIL" if bad else ""))
    sys.exit(1 if bad else 0)
