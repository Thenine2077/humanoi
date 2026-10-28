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

encoder (firmware/humanoi_base_mega ส่ง "ENC t c1 c2 c3 c4 e1 e2 e3 e4" ทุก ~50 ms):
  /wheel_states  sensor_msgs/JointState  มุมล้อ (rad) + ความเร็ว (rad/s) ชื่อตาม URDF front_left_wheel_joint ...
  /wheel_odom    nav_msgs/Odometry       odometry จากล้อ (mecanum) — TF odom->base ปิดไว้ (rf2o ใช้อยู่)
  /encoder_raw   std_msgs/Int64MultiArray [t_ms, c1..c4, e1..e4] ค่าดิบตามขา E1-E4 (arm wheel-check ใช้)
/wheel_cmd (std_msgs/String "W F S S S") สั่งทิศ 4 ล้อตรงๆ ข้าม mixing — ไว้จูน/เช็กล้อ (ชนะ /cmd_vel
  ตอนที่ยังส่งมาไม่เกิน cmd_timeout)

พารามิเตอร์
  port            (str)   /dev/ttyACM0   ("none" = dry-run ไม่เปิด serial ทดสอบ mixing)
  baudrate        (int)   115200
  protocol        (str)   mecanum4 | single
  cmd_rate        (float) 20.0    Hz
  cmd_timeout     (float) 0.5     s
  linear_deadband (float) 0.03    m/s   (โหมด single)
  wheel_deadband  (float) 0.05    rad/s (โหมด mecanum4 — ต่ำกว่านี้ถือว่าล้อหยุด)
  wheel_radius    (float) 0.0725  m     ล้อ Ø14.5 cm
  track           (float) 0.385   m     กลางล้อซ้าย-ขวา
  wheelbase       (float) 0.385   m     เพลาหน้า-หลัง
  reopen_period   (float) 2.0     s
  counts_per_rev  (float) 1440.0  ค่านับต่อรอบ "ล้อ" (360 P/R x4 = 1440 ถ้า encoder หมุนรอบเดียวกับล้อ)
  encoder_map     (int[]) [0, 1, 2, 3]  ล้อ FL FR RL RR ใช้ encoder ตัวไหน (0 = E1 ... 3 = E4)
  encoder_signs   (int[]) [1, 1, 1, 1]  1 / -1 ให้ล้อเดินหน้า = ค่าบวก
  odom_frame / base_frame (str) odom / base_footprint
  publish_odom_tf (bool)  false   true = ส่ง TF odom->base_frame จากล้อ (อย่าเปิดคู่กับ rf2o)

ไม่มี pyserial / เปิดพอร์ตไม่ได้ = log warn แต่โหนดยังรันต่อ (ดูคำสั่งที่ /motor_command)
"""
import math
import re
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int64MultiArray, String
from tf2_ros import TransformBroadcaster

try:
    import serial
except ImportError:
    serial = None

WHEEL_NAMES = ("FL", "FR", "RL", "RR")
WHEEL_JOINTS = ("front_left_wheel_joint", "front_right_wheel_joint",
                "rear_left_wheel_joint", "rear_right_wheel_joint")  # ชื่อเดียวกับ mecanum_base.urdf.xacro
WHEEL_CMD = re.compile(r"^W\s*([FBS])\s*([FBS])\s*([FBS])\s*([FBS])$")
ENC_GAP = 0.5  # s — ค่านับสองครั้งห่างกันเกินนี้ (Mega รีเซ็ต/สายหลุด) ไม่คิดความเร็ว


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
        self.declare_parameter("wheel_radius", 0.0725)
        self.declare_parameter("track", 0.385)
        self.declare_parameter("wheelbase", 0.385)
        self.declare_parameter("reopen_period", 2.0)
        self.declare_parameter("counts_per_rev", 1440.0)
        self.declare_parameter("encoder_map", [0, 1, 2, 3])
        self.declare_parameter("encoder_signs", [1, 1, 1, 1])
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_odom_tf", False)

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
        self.cpr = float(self.get_parameter("counts_per_rev").value)
        self.enc_map = [int(v) for v in self.get_parameter("encoder_map").value]
        self.enc_signs = [1 if int(v) >= 0 else -1 for v in self.get_parameter("encoder_signs").value]
        if sorted(self.enc_map) != [0, 1, 2, 3] or len(self.enc_signs) != 4:
            self.get_logger().error(f"encoder_map {self.enc_map} / encoder_signs {self.enc_signs} ไม่ถูก "
                                    "-> ใช้ [0,1,2,3] / [1,1,1,1]")
            self.enc_map, self.enc_signs = [0, 1, 2, 3], [1, 1, 1, 1]
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.odom_tf = bool(self.get_parameter("publish_odom_tf").value)

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
        self.wheel_cmd_line = self.stop_line
        self.last_wheel_cmd_t = -1e9
        self.prev_enc = None          # (t_ms ของ Mega, มุมล้อ rad 4 ค่า)
        self.x = self.y = self.yaw = 0.0

        self.sub = self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.state_pub = self.create_publisher(String, "motor_state", 10)
        self.cmd_pub = self.create_publisher(String, "motor_command", 10)
        self.wheel_pub = self.create_publisher(Float64MultiArray, "wheel_target", 10)
        self.wheel_cmd_sub = self.create_subscription(String, "wheel_cmd", self.on_wheel_cmd, 10)
        self.joint_pub = self.create_publisher(JointState, "wheel_states", 10)
        self.odom_pub = self.create_publisher(Odometry, "wheel_odom", 10)
        self.raw_pub = self.create_publisher(Int64MultiArray, "encoder_raw", 10)
        self.tf_pub = TransformBroadcaster(self) if self.odom_tf else None

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

    def on_wheel_cmd(self, msg: String):
        m = WHEEL_CMD.match(msg.data.strip().upper())
        if not m or self.protocol != "mecanum4":
            self.get_logger().warn(f"wheel_cmd '{msg.data}' ไม่ถูก (ใช้ \"W F S S S\" ลำดับ FL FR RL RR)",
                                   throttle_duration_sec=2.0)
            return
        self.wheel_cmd_line = "W " + " ".join(m.groups())
        self.last_wheel_cmd_t = time.monotonic()

    # ---------------------------------------------------------------
    def tick(self):
        now = time.monotonic()
        self._maybe_reopen(now)

        line = self.last_line
        if now - self.last_cmd_vel_t > self.cmd_timeout:  # teleop ตาย / ไม่มีคนสั่ง
            line = self.stop_line
        if now - self.last_wheel_cmd_t <= self.cmd_timeout:  # สั่งล้อตรงๆ (จูน/เช็กล้อ) ชนะ /cmd_vel
            line = self.wheel_cmd_line

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
                if line.startswith("ENC "):  # ~20 ครั้ง/วิ — ไม่ส่งต่อ /motor_state
                    self.on_encoders(line)
                    continue
                if line.startswith("READY"):
                    self.prev_enc = None  # Mega รีเซ็ต ค่านับเริ่ม 0 ใหม่
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

    # ---------------------------------------------------------------
    def on_encoders(self, line: str):
        """ENC t c1 c2 c3 c4 e1 e2 e3 e4 -> /encoder_raw, /wheel_states, /wheel_odom"""
        parts = line.split()
        if len(parts) != 10:
            return
        try:
            t_ms = int(parts[1])
            raw = [int(v) for v in parts[2:6]]
            errs = [int(v) for v in parts[6:10]]
        except ValueError:
            return
        self.raw_pub.publish(Int64MultiArray(data=[t_ms] + raw + errs))
        pos = [self.enc_signs[i] * raw[self.enc_map[i]] * 2.0 * math.pi / self.cpr for i in range(4)]
        vel = [0.0] * 4
        if self.prev_enc is not None:
            dt = (t_ms - self.prev_enc[0]) / 1000.0
            if 0.0 < dt <= ENC_GAP:
                vel = [(p - q) / dt for p, q in zip(pos, self.prev_enc[1])]
                self._integrate(vel, dt)
        self.prev_enc = (t_ms, pos)

        stamp = self.get_clock().now().to_msg()
        js = JointState()
        js.header.stamp = stamp
        js.name = list(WHEEL_JOINTS)
        js.position = pos
        js.velocity = vel
        self.joint_pub.publish(js)
        self._publish_odom(stamp, self._body_twist(vel))

    def _body_twist(self, w):
        """ความเร็วล้อ (rad/s) FL FR RL RR -> (vx, vy, wz) ตัวถัง — ย้อนสูตร _mecanum_wheels"""
        r, k = self.r, self.geom
        vx = r / 4.0 * (w[0] + w[1] + w[2] + w[3])
        vy = r / 4.0 * (-w[0] + w[1] + w[2] - w[3])
        wz = r / (4.0 * k) * (-w[0] + w[1] - w[2] + w[3])
        return vx, vy, wz

    def _integrate(self, w, dt):
        vx, vy, wz = self._body_twist(w)
        mid = self.yaw + wz * dt / 2.0
        self.x += (vx * math.cos(mid) - vy * math.sin(mid)) * dt
        self.y += (vx * math.sin(mid) + vy * math.cos(mid)) * dt
        self.yaw = math.atan2(math.sin(self.yaw + wz * dt), math.cos(self.yaw + wz * dt))

    def _publish_odom(self, stamp, twist):
        qz, qw = math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)
        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.angular.z = twist
        # ล้อ mecanum ลื่นตอนสไลด์/หมุน — ค่าประมาณคร่าวๆ ให้ EKF (robot_localization) รู้ว่าเชื่อได้แค่ไหน
        for i, v in zip((0, 7, 35), (0.05, 0.05, 0.1)):
            od.pose.covariance[i] = v
        for i, v in zip((0, 7, 35), (0.02, 0.05, 0.1)):
            od.twist.covariance[i] = v
        self.odom_pub.publish(od)
        if self.tf_pub is not None:
            tf = TransformStamped()
            tf.header = od.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_pub.sendTransform(tf)

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
