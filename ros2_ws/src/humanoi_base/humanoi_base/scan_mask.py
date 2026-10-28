#!/usr/bin/env python3
"""scan_mask — ตัดจุด lidar ที่ตกบนตัวหุ่นเอง (เสา ของบนแผ่นฐาน) ออกจากสแกน

  /scan_raw (ไดรเวอร์ lidar) -> scan_mask -> /scan (rf2o, slam_toolbox, ตัวกันชนของฐาน)

ทำไม: เสาอยู่หลัง lidar ~10-18 cm ในระนาบเดียวกัน — จุดพวกนั้นไม่ขยับเวลาหุ่นวิ่ง
rf2o (เดาการเคลื่อนที่จากสแกน) จะเชื่อว่าหุ่นอยู่นิ่ง และแผนที่จะมีจุดเปื้อนตามทางที่วิ่ง
จุดที่อยู่ในกรอบตัวหุ่น (footprint + margin) ถูกตั้งเป็น inf = "ไม่เห็นอะไร"

พารามิเตอร์: footprint [หน้า หลัง ซ้าย ขวา] m จากกลางฐาน (ค่าเดียวกับ base_motion), margin,
  lidar_x / lidar_y / lidar_yaw = ตำแหน่ง lidar จากกลางฐาน (เท่ากับ static TF ใน lidar.launch.py)
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from humanoi_base.base_motion import DEFAULTS


class ScanMask(Node):
    def __init__(self):
        super().__init__("scan_mask")
        self.declare_parameter("footprint", DEFAULTS["footprint"])
        self.declare_parameter("margin", 0.03)
        self.declare_parameter("lidar_x", 0.16)
        self.declare_parameter("lidar_y", 0.0)
        self.declare_parameter("lidar_yaw", 0.0)
        self.f, self.b, self.l, self.r = [float(v) for v in self.get_parameter("footprint").value]
        self.m = float(self.get_parameter("margin").value)
        self.lx = float(self.get_parameter("lidar_x").value)
        self.ly = float(self.get_parameter("lidar_y").value)
        self.yaw = float(self.get_parameter("lidar_yaw").value)
        self.table = None   # (angle_min, increment, n) -> ระยะสูงสุดที่ยังอยู่ในกรอบตัวหุ่น ต่อมุม
        self.pub = self.create_publisher(LaserScan, "scan", 10)
        self.create_subscription(LaserScan, "scan_raw", self.on_scan, qos_profile_sensor_data)
        self.reported = False

    def inside_until(self, angle):
        """ระยะจาก lidar ตามมุมนี้ ที่จุดยังอยู่ในกรอบตัวหุ่น (เดินจาก lidar ออกไปจนพ้นกรอบ)"""
        a = angle + self.yaw
        c, s = math.cos(a), math.sin(a)
        d, last = 0.0, 0.0
        while d < 1.0:
            x, y = self.lx + d * c, self.ly + d * s
            if -self.b - self.m <= x <= self.f + self.m and -self.r - self.m <= y <= self.l + self.m:
                last = d
            d += 0.005
        return last

    def on_scan(self, msg):
        key = (msg.angle_min, msg.angle_increment, len(msg.ranges))
        if self.table is None or self.table[0] != key:
            self.table = (key, [self.inside_until(msg.angle_min + i * msg.angle_increment)
                                for i in range(len(msg.ranges))])
        limit = self.table[1]
        ranges = list(msg.ranges)
        cut = 0
        for i, rng in enumerate(ranges):
            if math.isfinite(rng) and rng <= limit[i]:
                ranges[i] = float("inf")
                cut += 1
        if not self.reported:
            self.reported = True
            self.get_logger().info(f"scan_mask พร้อม — ตัดจุดบนตัวหุ่นสแกนแรก {cut} จุด จาก {len(ranges)} "
                                   f"(กรอบ หน้า {self.f} หลัง {self.b} ซ้าย {self.l} ขวา {self.r} m + {self.m})")
        msg.ranges = ranges
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ScanMask()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
