#!/usr/bin/env python3
"""ขับเปิดลูป (open-loop) เป็นสี่เหลี่ยมด้วยการเลื่อนตัว (mecanum, หัวไม่หมุน)
ใช้ทดสอบ M2 — วัด drift ของ /odom เทียบจุดเริ่ม

  ros2 run humanoi_teleop square_drive --ros-args -p use_sim_time:=true
  ros2 run humanoi_teleop square_drive --ros-args -p use_sim_time:=true -p side:=2.0 -p speed:=0.3

ลำดับ: หน้า -> ขวา -> หลัง -> ซ้าย  (แต่ละด้าน = side เมตร ที่ speed m/s)
จบแล้วพิมพ์ตำแหน่ง /odom ตอนเริ่ม/ตอนจบ + ระยะคลาดเคลื่อน

พารามิเตอร์
  side   (float) 2.0   ความยาวด้าน (m)
  speed  (float) 0.3   ความเร็วเลื่อนตัว (m/s)
  settle (float) 1.0   หยุดพักระหว่างด้าน (s)

** ต้องตั้ง use_sim_time:=true ** ไม่งั้นจับเวลาผิดเพราะ sim ไม่ได้รันเรียลไทม์
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class SquareDrive(Node):
    def __init__(self):
        super().__init__("square_drive")
        self.declare_parameter("side", 2.0)
        self.declare_parameter("speed", 0.3)
        self.declare_parameter("settle", 1.0)

        self.side = float(self.get_parameter("side").value)
        spd = float(self.get_parameter("speed").value)
        settle = float(self.get_parameter("settle").value)
        leg = self.side / spd

        # (duration, vx, vy)  — หน้า, ขวา, หลัง, ซ้าย
        self.plan = [
            (leg, spd, 0.0), (settle, 0.0, 0.0),
            (leg, 0.0, -spd), (settle, 0.0, 0.0),
            (leg, -spd, 0.0), (settle, 0.0, 0.0),
            (leg, 0.0, spd), (settle, 0.0, 0.0),
        ]
        self.total = sum(d for d, _, _ in self.plan)

        if not self.get_parameter("use_sim_time").value:
            self.get_logger().warn(
                "use_sim_time = false — ถ้า sim ไม่ได้รันเรียลไทม์ ระยะจะเพี้ยน "
                "ให้เพิ่ม  --ros-args -p use_sim_time:=true"
            )

        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.odom = None
        self.create_subscription(Odometry, "odom", self._on_odom, 10)

        self.t0 = None
        self.start_pose = None
        self.done = False
        self.timer = self.create_timer(0.02, self._tick)
        self.get_logger().info(
            f"ขับสี่เหลี่ยม side={self.side} m speed={spd} m/s รวม ~{self.total:.1f}s (เวลา sim)"
        )

    def _on_odom(self, msg: Odometry):
        self.odom = msg.pose.pose

    @staticmethod
    def _xy(p):
        return p.position.x, p.position.y

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        if self.done:
            return

        if self.t0 is None:
            if self.odom is None:
                return  # รอ /odom ตัวแรก
            self.t0 = self._now()
            self.start_pose = self.odom
            sx, sy = self._xy(self.start_pose)
            self.get_logger().info(f"จุดเริ่ม odom = ({sx:.3f}, {sy:.3f})")

        el = self._now() - self.t0
        if el >= self.total:
            self.pub.publish(Twist())
            self._report()
            self.done = True
            return

        acc, vx, vy = 0.0, 0.0, 0.0
        for d, cx, cy in self.plan:
            if el < acc + d:
                vx, vy = cx, cy
                break
            acc += d

        tw = Twist()
        tw.linear.x = vx
        tw.linear.y = vy
        self.pub.publish(tw)

    def _report(self):
        sx, sy = self._xy(self.start_pose)
        ex, ey = self._xy(self.odom)
        err = math.hypot(ex - sx, ey - sy)
        perim = 4 * self.side
        line = "=" * 46
        self.get_logger().info(line)
        self.get_logger().info(f"เริ่ม odom : ({sx:.3f}, {sy:.3f})")
        self.get_logger().info(f"จบ   odom : ({ex:.3f}, {ey:.3f})")
        self.get_logger().info(
            f"คลาดเคลื่อน : {err:.3f} m  = {err / perim * 100:.1f}% ของเส้นรอบรูป {perim:.0f} m"
        )
        self.get_logger().info(f"เกณฑ์ M2  : < 5%  ->  {'ผ่าน' if err / perim < 0.05 else 'ไม่ผ่าน'}")
        self.get_logger().info(line)


def main():
    rclpy.init()
    node = SquareDrive()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
