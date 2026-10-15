#!/usr/bin/env python3
"""rc_arm_trigger — โยกสวิตช์บนรีโมท FlySky แล้วสั่งแขน AR4 เล่นโปรแกรม / หยุด

  SWB ขึ้น -> ลง : เล่นโปรแกรม (พารามิเตอร์ program_swb ค่าเริ่ม "1") ผ่าน /arm/run
  SWD ขึ้น -> ลง : หยุดแขน (/arm/stop) — และระหว่างที่ SWD ลงค้างอยู่ SWB สั่งอะไรไม่ได้

ตำแหน่งสวิตช์มาจาก Mega (firmware/humanoi_base_mega) เป็นบรรทัด "SW:BxDy" ที่ relay_bridge
ส่งต่อมาใน /motor_state  x, y = U (ขึ้น) | D (ลง) | - (ไม่รู้: รีโมทดับ ตัวรับส่งค่า failsafe)

กันสั่งมั่ว
  - สั่งเฉพาะ "จังหวะโยก" ขึ้น -> ลง สวิตช์ลงค้างไว้ไม่สั่งซ้ำ จะสั่งอีกต้องยกขึ้นแล้วโยกลงใหม่
  - ต้องเห็นสวิตช์ "ขึ้น" ก่อนหนึ่งครั้ง หลังเปิดโหนดหรือหลังรีโมทหลุด
    (เปิดระบบตอนสวิตช์ลงอยู่ หรือสัญญาณกลับมาตอนสวิตช์ลงอยู่ = ไม่สั่ง)
  - "-" ทั้งสองตัว (รีโมทหลุด) ล้างสถานะ ต้องเห็นขึ้นใหม่ก่อน
  - ใช้สวิตช์ 3 ระดับแทนได้ (เช่น SWC ตั้งเป็น CH6): ตำแหน่งกลางไม่นับ บนสุด = พร้อม ล่างสุด = สั่ง

ต้องเปิด relay_bridge (Mega) และ arm_program (แขน) อยู่ด้วย
  ros2 run humanoi_base rc_arm_trigger --ros-args -p program_swb:=1
"""
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Empty, String


class Switch:
    """ตำแหน่งสวิตช์ 2 ทาง + กลไกต้องเห็น 'ขึ้น' ก่อนถึงจะนับการโยกลง"""

    def __init__(self):
        self.pos = None     # 'U' | 'D' | None (ไม่รู้)
        self.ready = False  # เห็นขึ้นแล้ว พร้อมนับการโยกลงครั้งถัดไป

    def update(self, p):
        """รับตำแหน่งล่าสุด คืน True ถ้าเป็นจังหวะโยก ขึ้น -> ลง ที่นับได้"""
        fired = p == "D" and self.pos == "U" and self.ready
        if p == "U":
            self.ready = True
        elif p not in ("U", "D"):
            self.ready = False
        self.pos = p if p in ("U", "D") else None
        return fired


class RcArmTrigger(Node):
    def __init__(self):
        super().__init__("rc_arm_trigger")
        # dynamic_typing: ชื่อโปรแกรมเป็นตัวเลขได้ ("-p program_swb:=1" ROS อ่านเป็น int ไม่ใช่ string)
        self.declare_parameter("program_swb", "1", ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("state_topic", "motor_state")
        self.program = str(self.get_parameter("program_swb").value)

        self.swb = Switch()
        self.swd = Switch()
        self.last_raw = None

        self.run_pub = self.create_publisher(String, "/arm/run", 10)
        self.stop_pub = self.create_publisher(Empty, "/arm/stop", 10)
        self.create_subscription(
            String, self.get_parameter("state_topic").value, self.on_state, 50)
        self.create_subscription(String, "/arm/done", self.on_done, 10)
        self.get_logger().info(
            f"rc_arm_trigger พร้อม — SWB โยกลง = เล่นโปรแกรม '{self.program}', SWD โยกลง = หยุดแขน")

    def on_state(self, msg):
        line = msg.data.strip()
        if not line.startswith("SW:B") or len(line) < 7 or line[5] != "D":
            return
        b, d = line[4], line[6]
        if line != self.last_raw:
            self.last_raw = line
            mid = lambda p: "กลาง" if (p == "-" and (b, d) != ("-", "-")) else p
            self.get_logger().info(f"สวิตช์ SWB={mid(b)} SWD={mid(d)}")

        # "-" ทั้งคู่ = รีโมทดับ/หลุด (Mega ตรวจจาก SWA แล้วส่ง "-" ทุกตัว) -> ล้างสถานะ ต้องเห็นขึ้นใหม่
        # "-" ตัวเดียวแต่อีกตัวปกติ = ตำแหน่งกลางของสวิตช์ 3 ระดับ (เช่น SWC ~1500) หรืออ่านพลาดครั้งเดียว
        #   -> ข้ามไป ถือว่ายังอยู่ตำแหน่งเดิม: บน -> กลาง -> ล่าง ยังนับเป็นโยกลง 1 ครั้ง
        #      และค้างล่างแล้วอ่านพลาดแวบเดียวก็ไม่สั่งซ้ำ
        lost = b == "-" and d == "-"
        if not lost and b == "-":
            b = None
        if not lost and d == "-":
            d = None

        # SWD ก่อน — ถ้าโยกทั้งคู่ในจังหวะเดียวกัน ให้หยุดชนะ
        if d is not None and self.swd.update(d):
            self.stop_pub.publish(Empty())
            self.get_logger().warn("SWD โยกลง -> หยุดแขน (/arm/stop)")
        if b is not None and self.swb.update(b):
            if self.swd.pos == "D":
                self.get_logger().warn("SWB โยกลง แต่ SWD ลงอยู่ (หยุด) -> ไม่สั่ง ยก SWD ขึ้นก่อน")
            else:
                self.run_pub.publish(String(data=self.program))
                self.get_logger().info(f"SWB โยกลง -> เล่นโปรแกรม '{self.program}'")

    def on_done(self, msg):
        self.get_logger().info(f"แขนรายงาน: {msg.data}")


def main():
    rclpy.init()
    node = RcArmTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
