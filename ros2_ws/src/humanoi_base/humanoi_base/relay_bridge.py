#!/usr/bin/env python3
"""สะพาน /cmd_vel -> USB serial -> Arduino MEGA (relay H-bridge)

รับ geometry_msgs/Twist ที่ /cmd_vel แล้วส่งคำสั่งตัวอักษรไป Arduino
รองรับ 2 โปรโตคอล (พารามิเตอร์ `protocol`):

  single   : มอเตอร์เดียว — ใช้แค่ linear.x -> 'F' / 'B' / 'S'
             (คู่กับ firmware/relay_motor_mega/)

  mecanum4 : 4 ล้อ — ทำ mecanum mixing จาก (linear.x, linear.y, angular.z)
             ได้ความเร็วเป้าหมายต่อล้อ แล้วส่ง "เครื่องหมาย" ของแต่ละล้อ
             เป็น  "W f f f f"  (ลำดับ FL FR RL RR, f = F|B|S)
             ความเร็วจริงตั้งด้วยลูกบิด PWM governor (บริดจ์คุมแค่ทิศ)
             (คู่กับ firmware/relay_motor_mega_4wd/)

ส่งซ้ำที่ cmd_rate Hz เป็น heartbeat ให้ watchdog ฝั่ง Arduino
ไม่มี /cmd_vel เกิน cmd_timeout วินาที -> บังคับหยุด
อ่านบรรทัดที่ Arduino ตอบ (READY/STATE/WARN/OK) -> log + republish /motor_state

พารามิเตอร์
  port            (str)   /dev/ttyACM0   ("none" = dry-run ไม่เปิด serial ทดสอบ mixing)
  baudrate        (int)   115200
  protocol        (str)   mecanum4 | single
  cmd_rate        (float) 20.0    Hz
  cmd_timeout     (float) 0.5     s
  linear_deadband (float) 0.03    m/s   (โหมด single)
  wheel_deadband  (float) 0.05    rad/s (โหมด mecanum4 — ต่ำกว่านี้ถือว่าล้อหยุด)
  wheel_radius    (float) 0.075   m
  track           (float) 0.40    m     ระยะล้อซ้าย-ขวา
  wheelbase       (float) 0.32    m     ระยะล้อหน้า-หลัง
  reopen_period   (float) 2.0     s

ไม่มี pyserial / เปิดพอร์ตไม่ได้ = log warn แต่โหนดยังรันต่อ (ดูคำสั่งที่ /motor_command)
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

try:
    import serial
except ImportError:
    serial = None

WHEEL_NAMES = ("FL", "FR", "RL", "RR")


def sign_cmd(v, deadband):
    if v > deadband:
        return "F"
    if v < -deadband:
        return "B"
    return "S"


class RelayBridge(Node):
    def __init__(self):
        super().__init__("relay_bridge")
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("protocol", "mecanum4")
        self.declare_parameter("cmd_rate", 20.0)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("linear_deadband", 0.03)
        self.declare_parameter("wheel_deadband", 0.05)
        self.declare_parameter("wheel_radius", 0.075)
        self.declare_parameter("track", 0.40)
        self.declare_parameter("wheelbase", 0.32)
        self.declare_parameter("reopen_period", 2.0)

        self.port = str(self.get_parameter("port").value).strip()
        # ค่าเหล่านี้ = โหมด dry-run (ไม่เปิด serial) สำหรับทดสอบ mixing ฝั่ง ROS
        if self.port.lower() in ("", "none", "null", "-", "/dev/null"):
            self.port = ""
        self.baud = int(self.get_parameter("baudrate").value)
        self.protocol = str(self.get_parameter("protocol").value)
        rate = float(self.get_parameter("cmd_rate").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.lin_deadband = float(self.get_parameter("linear_deadband").value)
        self.wheel_deadband = float(self.get_parameter("wheel_deadband").value)
        self.r = float(self.get_parameter("wheel_radius").value)
        self.track = float(self.get_parameter("track").value)
        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.reopen_period = float(self.get_parameter("reopen_period").value)

        if self.protocol not in ("mecanum4", "single"):
            self.get_logger().warn(f"protocol '{self.protocol}' ไม่รู้จัก -> ใช้ mecanum4")
            self.protocol = "mecanum4"

        # (lx + ly) สำหรับสูตร mecanum
        self.geom = (self.track + self.wheelbase) / 2.0

        self.stop_line = "S" if self.protocol == "single" else "W S S S S"
        self.last_line = self.stop_line
        self.last_cmd_vel_t = 0.0
        self.last_reopen_t = 0.0
        self.ser = None

        self.sub = self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.state_pub = self.create_publisher(String, "motor_state", 10)
        self.cmd_pub = self.create_publisher(String, "motor_command", 10)
        self.wheel_pub = self.create_publisher(Float64MultiArray, "wheel_target", 10)

        self._open_serial()
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f"relay_bridge พร้อม — protocol={self.protocol} port={self.port} "
            f"baud={self.baud} rate={rate:.0f}Hz timeout={self.cmd_timeout:.1f}s"
        )

    # ---------------------------------------------------------------
    def _open_serial(self):
        if not self.port:
            self.get_logger().info("dry-run: port='' -> ไม่เปิด serial (ดูคำสั่งที่ /motor_command)")
            return
        if serial is None:
            self.get_logger().error("ไม่พบ pyserial -> sudo apt install python3-serial")
            return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.02)
            time.sleep(2.0)  # Arduino auto-reset ตอนเปิดพอร์ต
            self.ser.reset_input_buffer()
            self.get_logger().info(f"เปิดพอร์ต {self.port} สำเร็จ")
        except Exception as e:  # noqa: BLE001
            self.ser = None
            self.get_logger().warn(f"เปิดพอร์ต {self.port} ไม่ได้: {e}")

    def _maybe_reopen(self, now):
        if self.ser is not None or serial is None or not self.port:
            return
        if now - self.last_reopen_t >= self.reopen_period:
            self.last_reopen_t = now
            self._open_serial()

    # ---------------------------------------------------------------
    def _mecanum_wheels(self, vx, vy, wz):
        """twist ตัวถัง -> ความเร็วเชิงมุมเป้าหมายของ 4 ล้อ (rad/s) : FL FR RL RR"""
        k = self.geom
        return (
            (vx - vy - k * wz) / self.r,  # FL
            (vx + vy + k * wz) / self.r,  # FR
            (vx + vy - k * wz) / self.r,  # RL
            (vx - vy + k * wz) / self.r,  # RR
        )

    def on_cmd_vel(self, msg: Twist):
        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z

        if self.protocol == "single":
            self.last_line = sign_cmd(vx, self.lin_deadband)
        else:
            w = self._mecanum_wheels(vx, vy, wz)
            self.wheel_pub.publish(Float64MultiArray(data=list(w)))
            signs = [sign_cmd(x, self.wheel_deadband) for x in w]
            self.last_line = "W " + " ".join(signs)

        self.last_cmd_vel_t = time.monotonic()

    # ---------------------------------------------------------------
    def tick(self):
        now = time.monotonic()
        self._maybe_reopen(now)

        line = self.last_line
        if now - self.last_cmd_vel_t > self.cmd_timeout:  # teleop ตาย / ไม่มีคนสั่ง
            line = self.stop_line

        self.cmd_pub.publish(String(data=line))
        self._write(line)
        self._read_lines()

    def _write(self, line: str):
        if self.ser is None:
            return
        try:
            self.ser.write((line + "\n").encode("ascii"))
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"เขียน serial ไม่สำเร็จ: {e}")
            self._close()

    def _read_lines(self):
        if self.ser is None:
            return
        try:
            while self.ser.in_waiting:
                line = self.ser.readline().decode("ascii", "ignore").strip()
                if not line:
                    continue
                self.state_pub.publish(String(data=line))
                if line.startswith("WARN"):
                    self.get_logger().warn(f"[MEGA] {line}")
                elif line.startswith("READY"):
                    self.get_logger().info(f"[MEGA] {line}")
                else:
                    self.get_logger().debug(f"[MEGA] {line}")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"อ่าน serial ไม่สำเร็จ: {e}")
            self._close()

    def _close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:  # noqa: BLE001
                pass
            self.ser = None

    # ---------------------------------------------------------------
    def stop_motor(self):
        for _ in range(5):
            self._write(self.stop_line)
            time.sleep(0.02)


def main():
    rclpy.init()
    node = RelayBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motor()
        node._close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
