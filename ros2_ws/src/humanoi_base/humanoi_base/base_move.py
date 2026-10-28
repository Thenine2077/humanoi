#!/usr/bin/env python3
"""base_move — สั่งฐานวิ่งตามระยะ / หมุนตามมุม แล้วหยุด (encoder วัดระยะ + lidar กันชน)

  arm drive forward 1.0     เดินหน้า 1 m          arm drive back 0.5      ถอย 0.5 m
  arm drive left 0.3        สไลด์ซ้าย 0.3 m        arm drive right 0.3     สไลด์ขวา
  arm drive turn 90         หมุนซ้าย 90°           arm drive turn -90      หมุนขวา 90°

  ros2 run humanoi_base base_move forward 1.0 --ros-args -p base.odom_topic:=/odom   (ซิม)

ต้องมี: relay_bridge (arm base) + lidar · รีโมทเปิด SWA ลง ปล่อยคันโยก (Mega โหมด ROS)
ของขวางทาง = หยุดรอ (ไม่อ้อม) · ขยับคันโยก / ปลด SWA / Ctrl+C = หยุดทันที
ท้ายสุดบอกว่าไปได้จริงเท่าไหร่ — เลยเป้าทุกครั้ง = เพิ่ม base.stop_early (ล้อไหลหลังตัด relay)
"""
import sys
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from humanoi_base.base_motion import KINDS, BaseMotion, MoveError, declare


def main():
    args = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    if len(args) != 2 or args[0] not in KINDS:
        print(__doc__)
        return 1
    try:
        amount = float(args[1])
    except ValueError:
        print(f"✗ '{args[1]}' ไม่ใช่ตัวเลข")
        return 1
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)  # Ctrl+C ต้องมาถึงโค้ดนี้ จะได้สั่งหยุดฐานทัน
    node = Node("base_move")
    motion = BaseMotion(node, declare(node))
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    rc = 0
    try:
        unit = "°" if args[0] == "turn" else " m"
        print(f"ฐาน: {args[0]} {amount:g}{unit} (odometry {motion.cfg['odom_topic']}, "
              f"กันชน {'เปิด' if motion.cfg['guard'] else 'ปิด'}) · Ctrl+C = หยุด")
        res = motion.move(args[0], amount)
        print("✓ " + motion.describe(res))
    except MoveError as exc:
        print(f"✗ {exc}")
        rc = 1
    except KeyboardInterrupt:
        motion.halt(0.3)
        print("\n✗ หยุดแล้ว (Ctrl+C)")
        rc = 1
    finally:
        motion.halt(0.2)
        ex.shutdown()
        spin.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
