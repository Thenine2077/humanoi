#!/usr/bin/env python3
"""ประทับเวลา LaserScan จากลิดาร์จริงใหม่ ให้เป็น "เวลาของซิม" แล้วส่งต่อ

ใช้ตอนเอาลิดาร์ฮาร์ดแวร์จริงมาป้อน Nav2 ที่รันกับหุ่นใน Gazebo:

  /scan_real (ไดรเวอร์ลิดาร์จริง, ประทับด้วยนาฬิกาเครื่อง ~1.79e9 s)
      -> scan_restamp (use_sim_time: true)
  /scan      (ประทับด้วยนาฬิกาซิม ที่เริ่มนับจาก 0 ตอนเปิด Gazebo)

ทำไมต้องมี: ทั้ง Nav2 และ TF ของหุ่นในซิมใช้ use_sim_time ถ้าป้อนสแกนที่ประทับด้วย
นาฬิกาจริงตรงๆ costmap จะหา TF ณ เวลานั้นไม่เจอ (ต่างกันเป็นพันล้านวินาที) แล้วทิ้ง
สแกนทุกอันเงียบๆ — costmap ว่างเปล่าทั้งที่ /scan มีข้อมูลวิ่งอยู่ 10 Hz

ประทับเวลา = เวลาซิมตอนได้รับ "ลบ" stamp_offset (ค่าเริ่ม 0.1 s) ด้วยเหตุผล 2 ข้อ:
  1) ถ้าใช้ "เวลาปัจจุบัน" ตรงๆ สแกนจะล้ำหน้า TF ของหุ่นเสมอ — TF odom->base_footprint
     มาจาก gz ผ่าน bridge ช้ากว่า /clock ราว 20-50 ms (ยิ่ง RTF ต่ำยิ่งช้า) ผู้รับทุกตัว
     (costmap, RViz) จะเจอ "Lookup would require extrapolation into the future" แล้วสแกน
     ค้างในคิวรอ TF จนคิวเต็ม -> "discarding message because the queue is full"
  2) สแกนหนึ่งรอบใช้เวลาหมุน 100 ms และมาถึงช้ากว่าที่วัดจริงอีก (usbip + ประมวลผล)
     การถอยเวลา 0.1 s จึงใกล้เคียงเวลาที่วัดจริงมากกว่าเวลาที่ได้รับ

  ros2 run humanoi_base scan_restamp --ros-args -p use_sim_time:=true
"""
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanRestamp(Node):
    def __init__(self):
        super().__init__("scan_restamp")
        self.declare_parameter("in_topic", "/scan_real")
        self.declare_parameter("out_topic", "/scan")
        # ว่าง = คง frame_id เดิมจากไดรเวอร์
        self.declare_parameter("frame_id", "")
        # ถอยเวลาจาก "ตอนได้รับ" กี่วินาที (ดู docstring ด้านบน)
        self.declare_parameter("stamp_offset", 0.1)

        in_topic = str(self.get_parameter("in_topic").value)
        out_topic = str(self.get_parameter("out_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.offset = Duration(seconds=float(self.get_parameter("stamp_offset").value))

        # รับแบบ best-effort: เข้ากันได้กับ publisher ทุกแบบ (ทั้ง reliable และ best-effort)
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # ส่งแบบ reliable: เข้ากันได้กับ subscriber ทุกแบบ — costmap ของ Nav2 รับแบบ
        # best-effort ส่วน LaserScan display ของ RViz บางเวอร์ชันรับแบบ reliable
        # ถ้าส่ง best-effort ไป RViz จะไม่เห็นสแกนเลยโดยไม่มี error ใดๆ
        self.pub = self.create_publisher(LaserScan, out_topic, 10)
        self.create_subscription(LaserScan, in_topic, self._on_scan, sub_qos)

        self.count = 0
        self.get_logger().info(
            f"scan_restamp: {in_topic} -> {out_topic} "
            f"(use_sim_time={self.get_parameter('use_sim_time').value}, "
            f"stamp_offset={self.offset.nanoseconds * 1e-9:.3f}s)"
        )

    def _on_scan(self, msg: LaserScan):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            # ยังไม่ได้รับ /clock จาก Gazebo — ถ้าส่งไปตอนนี้จะได้ stamp = 0 ซึ่ง costmap ทิ้งอยู่ดี
            self.get_logger().warn(
                "ยังไม่มี /clock จาก Gazebo — รอซิมเริ่มก่อน", throttle_duration_sec=3.0
            )
            return

        # ต้องเช็ค "ก่อน" ลบ: rclpy.Time แทนค่าติดลบไม่ได้ และ throw ValueError ตั้งแต่
        # ตอนลบเลย (เช่น เวลาซิม 0.02 s - 0.1 s) — เคยทำให้โหนดนี้ตายทันทีที่ซิมเริ่ม
        if now.nanoseconds <= self.offset.nanoseconds:
            return  # ช่วง 0.1 วินาทีแรกของซิม ยังถอยเวลาไม่ได้ ทิ้งไปก่อน
        msg.header.stamp = (now - self.offset).to_msg()
        if self.frame_id:
            msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

        self.count += 1
        if self.count == 1:
            self.get_logger().info(
                f"ส่งสแกนแรกแล้ว frame={msg.header.frame_id} "
                f"จุด={len(msg.ranges)} เวลาซิม={now.nanoseconds * 1e-9:.2f}s"
            )


def main():
    rclpy.init()
    node = ScanRestamp()
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
