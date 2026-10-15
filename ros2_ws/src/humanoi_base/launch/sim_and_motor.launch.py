"""M4 — รัน Gazebo + relay_bridge พร้อมกัน คีย์บอร์ดตัวเดียวสั่งทั้งคู่

  ros2 launch humanoi_base sim_and_motor.launch.py
  ros2 launch humanoi_base sim_and_motor.launch.py port:=/dev/ttyACM0

แล้วเปิดอีกหน้าต่างสำหรับคีย์บอร์ด (ต้องการ tty ของตัวเอง):
  ros2 run humanoi_teleop keyboard_teleop

กด w/s -> หุ่นใน Gazebo เดินหน้า/ถอย  และ  มอเตอร์จริงเดินหน้า/ถอย พร้อมกัน
กด a/d -> Gazebo สไลด์ข้าง (มอเตอร์เดียวไม่ตอบ — linear.y ยังไม่ถูกใช้)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc_share = get_package_share_directory("humanoi_description")
    port = LaunchConfiguration("port")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_share, "launch", "sim.launch.py")
        ),
    )

    relay_bridge = Node(
        package="humanoi_base",
        executable="relay_bridge",
        output="screen",
        parameters=[{
            "port": port,
            "baudrate": 115200,
            "protocol": LaunchConfiguration("protocol"),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("protocol", default_value="mecanum4",
                              description="mecanum4 (4 ล้อ) | single (1 มอเตอร์)"),
        sim,
        relay_bridge,
    ])
