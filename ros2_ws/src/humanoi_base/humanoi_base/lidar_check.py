#!/usr/bin/env python3
"""lidar_check — lidar หันตรงหน้าฐานไหม + ซ้าย/ขวาถูกด้านไหม -> บันทึกมุมเบี้ยวลง config/lidar_mount.yaml

  arm lidar-check      (เปิด arm lidar ไว้อีกหน้าต่าง)

1) วางของแคบๆ ตั้งตรง (ขวด กระป๋อง ขาเก้าอี้ กว้าง 3-10 cm) หน้าหุ่นตรงแนวกลาง ห่างราว 1 m รอบของโล่ง 40 cm
   ตรงแนวกลาง = วัดจากมุมหน้าซ้ายและมุมหน้าขวาของแผ่นฐานถึงของ ได้เท่ากัน
   -> lidar ต้องเห็นของที่ 0° · เห็นที่มุมไหน = lidar หันเบี้ยวเท่านั้น (เบี้ยว 1° = ห่าง 1 m คลาดข้าง 1.7 cm)
2) ย้ายของไปวางข้างซ้ายของหุ่น (ฝั่งแขนซ้าย) -> lidar ต้องเห็นของโผล่ทางซ้าย (มุมบวก)
   ถ้าโผล่ทางขวา = ข้อมูล lidar กลับด้าน — กันชนจะห้ามผิดข้าง ต้องแก้ laser_scan_dir ใน lidar.launch.py
ใช้ /scan ในเฟรมของ lidar ตรงๆ (ไม่ผ่าน TF) — บันทึกแล้วปิด/เปิด arm lidar ใหม่
"""
import math
import statistics
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan

from humanoi_base import lidar_mount

FRONT_DEG = 25.0      # หาของในกรวยหน้า ±เท่านี้
RANGE = (0.3, 2.5)    # m
CLUSTER_GAP = 0.05    # m จุดที่ไกลกว่าจุดใกล้สุดไม่เกินนี้ (และติดกัน) = ของชิ้นเดียวกัน
MAX_WIDTH = 0.25      # m ของกว้างกว่านี้ = ไม่ใช่ของแคบ (ผนัง/กล่องใหญ่)
MAX_SPREAD = 1.0      # องศา มุมแกว่งเกินนี้ระหว่างสแกน = ของขยับ/มีของอื่นปน
OK_DEG = 0.5          # เบี้ยวไม่เกินนี้ = ตรงแล้ว
MOVED = 0.15          # m ระยะในมุมเดียวกันใกล้เข้ามาเกินนี้ = ของที่ย้ายมาวาง


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def rays(msg):
    """[(มุม rad -pi..pi, ระยะ m)] เฉพาะจุดที่วัดได้"""
    out = []
    for i, r in enumerate(msg.ranges):
        if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
            out.append((wrap(msg.angle_min + i * msg.angle_increment), r))
    return out


def front_target(msg):
    """ของที่ใกล้สุดในกรวยหน้า -> (มุมกลาง rad, ระยะใกล้สุด m, ความกว้าง m) | None"""
    pts = sorted((a, r) for a, r in rays(msg)
                 if abs(a) <= math.radians(FRONT_DEG) and RANGE[0] <= r <= RANGE[1])
    if not pts:
        return None
    k = min(range(len(pts)), key=lambda i: pts[i][1])
    r0 = pts[k][1]
    step = 2.5 * abs(msg.angle_increment)  # จุดติดกัน (ยอมขาด 1 จุด)
    lo = hi = k
    while lo > 0 and pts[lo][0] - pts[lo - 1][0] <= step and pts[lo - 1][1] <= r0 + CLUSTER_GAP:
        lo -= 1
    while hi < len(pts) - 1 and pts[hi + 1][0] - pts[hi][0] <= step and pts[hi + 1][1] <= r0 + CLUSTER_GAP:
        hi += 1
    angles = [a for a, _ in pts[lo:hi + 1]]
    width = r0 * (angles[-1] - angles[0] + abs(msg.angle_increment))
    return sum(angles) / len(angles), r0, width


def binned(msg, deg=1.0):
    """{ช่องมุม: ระยะใกล้สุด} ช่องละ deg องศา"""
    out = {}
    for a, r in rays(msg):
        b = round(math.degrees(a) / deg)
        out[b] = min(r, out.get(b, r))
    return out


def appeared(before, after, deg=1.0):
    """มุม (องศา) ที่มีของใหม่โผล่ใกล้เข้ามา เทียบสแกนก่อน/หลังย้ายของ (ไม่นับกรวยหน้าที่ของเพิ่งออกไป)"""
    b0 = [binned(m, deg) for m in before]
    b1 = [binned(m, deg) for m in after]
    out = []
    for b in set().union(*b1):
        if abs(b * deg) <= FRONT_DEG + 5:
            continue
        now = statistics.median([x[b] for x in b1 if b in x]) if any(b in x for x in b1) else None
        if now is None or now > 1.5:
            continue
        was = [x[b] for x in b0 if b in x]
        if not was or statistics.median(was) - now > MOVED:
            out.append(b * deg)
    return out


def yaw_from_angle(angle, rng, mount_y):
    """ของอยู่ตรงแนวกลางฐาน แต่ lidar เห็นที่ angle -> มุมที่ lidar หันเบี้ยว (rad, ทวนเข็ม = บวก)"""
    return math.atan2(-mount_y, rng) - angle


class Checker(Node):
    def __init__(self):
        super().__init__("lidar_check")
        self.lock = threading.Lock()
        self.scans = []
        self.create_subscription(LaserScan, "scan", self.on_scan, 10)

    def on_scan(self, msg):
        with self.lock:
            self.scans.append(msg)
            del self.scans[:-200]

    def collect(self, seconds):
        with self.lock:
            self.scans.clear()
        time.sleep(seconds)
        with self.lock:
            return list(self.scans)


def ask(text):
    try:
        return input(text).strip()
    except EOFError:  # ไม่มีคนตอบ — ไม่เดาแทน
        sys.exit("\n✗ ไม่มีคำตอบ — หยุด")


def countdown(seconds):
    """ให้คนที่กด Enter เดินออกจากรอบหุ่นก่อนเริ่มวัด (ขาคนจะถูกนับเป็นของ)"""
    for s in range(seconds, 0, -1):
        print(f"\r   ถอยออกห่างหุ่นเกิน 1.5 m — เริ่มวัดใน {s} วิ ", end="", flush=True)
        time.sleep(1.0)
    print("\r   กำลังวัด...                                  ")


def run(node):
    mount = lidar_mount.load()
    print(f"ตอนนี้ใน {lidar_mount.path()}: หน้ากลางฐาน {mount['x']:.3f} m · หันเบี้ยว {mount['yaw_deg']:+.1f}°\n")
    ask("1) วางของแคบๆ ตั้งตรง (ขวด/กระป๋อง/ขาเก้าอี้) หน้าหุ่น ตรงแนวกลาง ห่างราว 1 m\n"
        "   ตรงแนวกลาง = วัดจากมุมหน้าซ้ายและมุมหน้าขวาของแผ่นฐานถึงของ ได้เท่ากัน · รอบของโล่ง 40 cm\n"
        "   กด Enter แล้วถอยออก ")
    countdown(5)
    front = node.collect(3.0)
    if len(front) < 10:
        sys.exit(f"✗ ได้สแกนแค่ {len(front)} รอบใน 3 วิ — เปิด arm lidar แล้วหรือยัง")
    found = [t for t in (front_target(m) for m in front) if t]
    if len(found) < len(front) // 2:
        sys.exit(f"✗ ไม่เจอของในกรวยหน้า ±{FRONT_DEG:.0f}° ระยะ {RANGE[0]}-{RANGE[1]} m")
    angle = statistics.median(t[0] for t in found)
    rng = statistics.median(t[1] for t in found)
    width = statistics.median(t[2] for t in found)
    spread = math.degrees(statistics.pstdev(t[0] for t in found))
    print(f"   เห็นของห่าง {rng:.3f} m ที่มุม {math.degrees(angle):+.2f}° · กว้าง {width * 100:.0f} cm · "
          f"แกว่ง ±{spread:.2f}° ({len(found)} สแกน)")
    if width > MAX_WIDTH:
        sys.exit(f"✗ ของกว้าง {width * 100:.0f} cm — น่าจะเจอผนัง/ของใหญ่ ใช้ของแคบๆ แล้วลองใหม่")
    if spread > MAX_SPREAD:
        sys.exit("✗ มุมแกว่งมาก — ของขยับหรือมีของอื่นใกล้ๆ เคลียร์รอบของแล้วลองใหม่")

    tape = ask("   ระยะจากกลางหัว lidar ถึงผิวหน้าของ (cm) [Enter = ข้าม]: ")
    if tape:
        try:
            diff = rng * 100 - float(tape)
            print(f"   lidar วัด {rng * 100:.1f} cm · ตลับเมตร {float(tape):.1f} cm · ต่าง {diff:+.1f} cm"
                  + ("" if abs(diff) <= 3 else "  ! ต่างมาก — เจอของถูกชิ้นไหม / วัดถูกจุดไหม"))
        except ValueError:
            print("   (อ่านตัวเลขไม่ได้ — ข้าม)")

    yaw = math.degrees(yaw_from_angle(angle, rng, mount["y"]))
    side = "ซ้าย (ทวนเข็ม)" if yaw > 0 else "ขวา (ตามเข็ม)"
    print(f"   -> lidar หันเบี้ยวไปทาง{side} {abs(yaw):.1f}°" if abs(yaw) > OK_DEG
          else f"   -> lidar หันตรงแล้ว (เบี้ยว {yaw:+.1f}° ไม่เกิน {OK_DEG}°)")

    ask("\n2) ย้ายของไปวางข้างซ้ายของหุ่น (ฝั่งแขนซ้าย) ห่างข้างฐานราว 30-50 cm แล้วกด Enter แล้วถอยออก ")
    countdown(5)
    after = node.collect(2.0)
    hits = appeared(front, after)
    mirrored = None
    if not hits:
        print("   ! ไม่เห็นของโผล่ใหม่ (ใกล้กว่า 1.5 m) — ข้ามข้อนี้ ลองใหม่ได้")
    else:
        side_deg = statistics.median(hits)
        mirrored = side_deg < 0
        print(f"   ของโผล่ที่มุม {side_deg:+.0f}° ({'ขวา' if mirrored else 'ซ้าย'}ของหุ่น)")
        if mirrored:
            print("   ✗ ข้อมูล lidar กลับด้าน (ซ้ายเป็นขวา) — กันชนจะห้ามผิดข้าง แผนที่จะกลับด้าน\n"
                  "     แก้: laser_scan_dir ใน humanoi_description/launch/lidar.launch.py (True <-> False) "
                  "แล้วรัน arm lidar-check ใหม่ · ไม่บันทึกมุมรอบนี้")
            return
        print("   ✓ ซ้าย/ขวาถูกด้าน")

    if abs(yaw - mount["yaw_deg"]) < 0.1:
        print(f"\n✓ ค่าในไฟล์ ({mount['yaw_deg']:+.1f}°) ตรงกับที่วัดแล้ว ไม่ต้องบันทึก")
        return
    a = ask(f"\nบันทึกมุมเบี้ยว {yaw:+.1f}° (เดิม {mount['yaw_deg']:+.1f}°) ลง lidar_mount.yaml ? [Y/n] ").lower()
    if a in ("", "y", "yes", "ใช่", "ช"):
        lidar_mount.save_yaw(round(yaw, 1))
        print("บันทึกแล้ว — ปิด/เปิด arm lidar ใหม่ (กันชน/แผนที่ใช้มุมใหม่)")


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Checker()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        run(node)
    except KeyboardInterrupt:
        print()
    finally:
        executor.shutdown()
        spinner.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
