#!/usr/bin/env python3
"""คีย์บอร์ด teleop สำหรับฐาน mecanum -> publish geometry_msgs/Twist ที่ /cmd_vel

ปุ่ม
  w / s   เดินหน้า / ถอยหลัง        (linear.x)
  a / d   สไลด์ซ้าย / สไลด์ขวา       (linear.y)
  q / e   หมุนซ้าย / หมุนขวา         (angular.z)
  space   หยุดทันที
  z / c   ลด / เพิ่ม สเกลความเร็ว (10%)
  Ctrl-C  ออก

พฤติกรรม
  - กดค้าง = เคลื่อนต่อเนื่อง (อาศัย key auto-repeat ของ terminal)
  - ปล่อยปุ่มเกิน key_hold_timeout วินาที = สั่งหยุดเอง (จำลอง failsafe ของรีโมท)
  - ส่งซ้ำที่ publish_rate Hz เป็น heartbeat เผื่อฝั่งล่างมี watchdog

พารามิเตอร์ ROS
  linear_speed      (m/s)   ความเร็วเชิงเส้นที่สเกลเต็ม   ดีฟอลต์ 0.5
  angular_speed     (rad/s) ความเร็วหมุนที่สเกลเต็ม        ดีฟอลต์ 1.2
  publish_rate      (Hz)    อัตราส่งซ้ำ                    ดีฟอลต์ 20.0
  key_hold_timeout  (s)     ปล่อยปุ่มนานเกินนี้ = หยุด     ดีฟอลต์ 0.5
  stamped           (bool)  ส่ง TwistStamped แทน Twist     ดีฟอลต์ false
"""
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node

# ปุ่ม -> ทิศ (fx = หน้า+, fy = ซ้าย+, fz = ทวนเข็ม+)
KEY_DIR = {
    "w": (1.0, 0.0, 0.0),
    "s": (-1.0, 0.0, 0.0),
    "a": (0.0, 1.0, 0.0),
    "d": (0.0, -1.0, 0.0),
    "q": (0.0, 0.0, 1.0),
    "e": (0.0, 0.0, -1.0),
}


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")
        self.declare_parameter("linear_speed", 0.5)
        self.declare_parameter("angular_speed", 1.2)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("key_hold_timeout", 0.5)
        self.declare_parameter("stamped", False)

        self.v_lin = float(self.get_parameter("linear_speed").value)
        self.v_ang = float(self.get_parameter("angular_speed").value)
        rate = float(self.get_parameter("publish_rate").value)
        self.timeout = float(self.get_parameter("key_hold_timeout").value)
        self.stamped = bool(self.get_parameter("stamped").value)

        msg_type = TwistStamped if self.stamped else Twist
        self.pub = self.create_publisher(msg_type, "cmd_vel", 10)

        self.scale = 1.0
        self.dir = (0.0, 0.0, 0.0)
        self.last_key_time = 0.0

        # อ่านคีย์แบบ non-blocking
        self._fd = sys.stdin.fileno()
        self._old_term = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)

        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info("keyboard_teleop พร้อม — โฟกัสหน้าต่างนี้แล้วกดปุ่ม (ดูปุ่มใน --help)")
        print(__doc__)

    # -- อ่านทุกไบต์ที่ค้างใน buffer, คืนตัวอักษรตัวสุดท้าย --
    def _drain_stdin(self):
        last = None
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0.0)
            if not r:
                break
            ch = sys.stdin.read(1)
            if ch:
                last = ch
        return last

    def tick(self):
        key = self._drain_stdin()
        now = time.monotonic()

        if key is not None:
            if key == "\x03":  # Ctrl-C
                raise KeyboardInterrupt
            elif key in KEY_DIR:
                self.dir = KEY_DIR[key]
                self.last_key_time = now
            elif key == " ":
                self.dir = (0.0, 0.0, 0.0)
                self.last_key_time = now
            elif key == "z":
                self.scale = max(0.1, round(self.scale - 0.1, 2))
                self.get_logger().info(f"scale = {self.scale:.1f}")
            elif key == "c":
                self.scale = min(1.0, round(self.scale + 0.1, 2))
                self.get_logger().info(f"scale = {self.scale:.1f}")

        # ปล่อยปุ่มนานเกินไป -> หยุด
        if now - self.last_key_time > self.timeout:
            self.dir = (0.0, 0.0, 0.0)

        fx, fy, fz = self.dir
        tw = Twist()
        tw.linear.x = fx * self.v_lin * self.scale
        tw.linear.y = fy * self.v_lin * self.scale
        tw.angular.z = fz * self.v_ang * self.scale

        if self.stamped:
            out = TwistStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            # ฐานเปลี่ยนชื่อลิงก์จาก base_link -> chassis_link แล้ว (กันชนกับลิงก์รากของแขน AR4
            # ที่ผู้ขายฮาร์ดโค้ดชื่อ base_link ไว้) ใช้ base_footprint ซึ่งเป็นเฟรมตัวถังที่ฉาย
            # ลงพื้น — ตรงกับที่ Nav2/SLAM ใช้เป็น robot_base_frame อยู่แล้ว
            out.header.frame_id = "base_footprint"
            out.twist = tw
            self.pub.publish(out)
        else:
            self.pub.publish(tw)

    def send_stop(self):
        if self.stamped:
            out = TwistStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(out)
        else:
            self.pub.publish(Twist())

    def restore_terminal(self):
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_term)


def main():
    rclpy.init()
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.restore_terminal()
        for _ in range(5):  # ย้ำ stop หลายครั้งกันหลุด
            node.send_stop()
            time.sleep(0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
