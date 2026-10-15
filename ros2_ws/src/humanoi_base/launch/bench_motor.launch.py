"""ทดสอบ M3 บนโต๊ะ : คีย์บอร์ด -> /cmd_vel -> serial -> Arduino relay H-bridge

  ros2 launch humanoi_base bench_motor.launch.py
  ros2 launch humanoi_base bench_motor.launch.py port:=/dev/ttyACM1

หมายเหตุ: keyboard_teleop ต้องการ tty ของตัวเอง ถ้ารันผ่าน launch แล้วปุ่มไม่ตอบ
ให้แยกรัน 2 หน้าต่างแทน:
  หน้าต่าง 1: ros2 run humanoi_base   relay_bridge --ros-args -p port:=/dev/ttyACM0
  หน้าต่าง 2: ros2 run humanoi_teleop keyboard_teleop
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port")
    protocol = LaunchConfiguration("protocol")

    relay_bridge = Node(
        package="humanoi_base",
        executable="relay_bridge",
        output="screen",
        parameters=[{"port": port, "baudrate": 115200, "protocol": protocol}],
    )

    keyboard = Node(
        package="humanoi_teleop",
        executable="keyboard_teleop",
        output="screen",
        prefix="xterm -e",  # ให้ teleop มี tty ของตัวเอง (ต้องมี xterm)
        parameters=[{"linear_speed": 1.0, "angular_speed": 1.0}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("protocol", default_value="single",
                              description="single (1 มอเตอร์) | mecanum4 (4 ล้อ)"),
        relay_bridge,
        keyboard,
    ])
