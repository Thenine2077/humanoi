#!/usr/bin/env python3
"""เดินหน้า-ถอยกลับวนซ้ำ (patrol) ให้โมเดล walker ใน worlds/room_walker.sdf
ใช้แทนการสั่ง `ros2 topic pub` มือทีละครั้ง — เดินไปทาง +y แล้วย้อนกลับ -y สลับกันเป็นวงรอบ

  ros2 run humanoi_teleop walker_patrol
  ros2 run humanoi_teleop walker_patrol --ros-args -p loops:=3
  ros2 run humanoi_teleop walker_patrol --ros-args -p speed:=0.5 -p distance:=5.5 -p pause:=1.5

พารามิเตอร์
  speed     (m/s)  ความเร็วเดิน                    ดีฟอลต์ 0.8
  distance  (m)    ระยะเดินต่อขา (ทิศเดียว)         ดีฟอลต์ 5.5  (พอสำหรับ y=-2.5 -> +3.0)
  pause     (s)    หยุดพักที่ปลายทางแต่ละฝั่ง        ดีฟอลต์ 1.0
  loops     (int)  จำนวนรอบไป-กลับ (0 = วนไม่รู้จบ)  ดีฟอลต์ 0
  topic     (str)  topic คุมความเร็ว (Twist)         ดีฟอลต์ walker/cmd_vel

Ctrl-C หยุด (ส่ง Twist ศูนย์ก่อนออกเสมอ กัน walker เดินต่อค้าง)
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class WalkerPatrol(Node):
    def __init__(self):
        super().__init__("walker_patrol")
        self.declare_parameter("speed", 0.8)
        self.declare_parameter("distance", 5.5)
        self.declare_parameter("pause", 1.0)
        self.declare_parameter("loops", 0)
        self.declare_parameter("topic", "walker/cmd_vel")

        self.speed = float(self.get_parameter("speed").value)
        self.distance = float(self.get_parameter("distance").value)
        self.pause = float(self.get_parameter("pause").value)
        self.loops = int(self.get_parameter("loops").value)
        topic = str(self.get_parameter("topic").value)

        self.pub = self.create_publisher(Twist, topic, 10)

        leg = self.distance / self.speed
        # เฟสในหนึ่งรอบ : เดิน +y, พัก, เดิน -y, พัก
        self.phases = [(leg, self.speed), (self.pause, 0.0),
                        (leg, -self.speed), (self.pause, 0.0)]
        self.cycle_len = sum(d for d, _ in self.phases)

        self.t0 = self.get_clock().now()
        self.done = False

        self.timer = self.create_timer(0.05, self._tick)
        loop_desc = "ไม่รู้จบ" if self.loops <= 0 else f"{self.loops} รอบ"
        self.get_logger().info(
            f"walker_patrol พร้อม — เดิน {self.distance:.1f} m ที่ {self.speed:.1f} m/s "
            f"({leg:.1f}s/ขา) วน{loop_desc} บน '{topic}' — Ctrl-C หยุด"
        )

    def _elapsed(self):
        return (self.get_clock().now() - self.t0).nanoseconds * 1e-9

    def _tick(self):
        if self.done:
            return

        el = self._elapsed()
        cycles_done = int(el // self.cycle_len)
        if self.loops > 0 and cycles_done >= self.loops:
            self.pub.publish(Twist())
            self.get_logger().info(f"ครบ {self.loops} รอบแล้ว — หยุด walker")
            self.done = True
            return

        t_in_cycle = el % self.cycle_len
        acc, vy = 0.0, 0.0
        for d, v in self.phases:
            if t_in_cycle < acc + d:
                vy = v
                break
            acc += d

        tw = Twist()
        tw.linear.y = vy
        self.pub.publish(tw)


def main():
    rclpy.init()
    node = WalkerPatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
