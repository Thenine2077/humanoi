"""SLAM (slam_toolbox online async) — สร้างแผนที่จาก /scan ระหว่างขับหุ่น

  # หน้าต่าง 1
  ros2 launch humanoi_description sim.launch.py world:=room rviz:=true
  # หน้าต่าง 2
  ros2 launch humanoi_nav slam.launch.py
  # หน้าต่าง 3
  ros2 run humanoi_teleop keyboard_teleop

ขับให้ทั่วห้อง แล้วเซฟแผนที่:
  ros2 run nav2_map_server map_saver_cli -f src/humanoi_nav/maps/room --ros-args -p use_sim_time:=true

ใช้ launch ของ slam_toolbox เอง (จัดการ lifecycle configure/activate ให้)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    slam_params = os.path.join(
        get_package_share_directory("humanoi_nav"), "config", "slam.yaml"
    )
    slam_toolbox_launch = os.path.join(
        get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py"
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch),
            launch_arguments={
                "use_sim_time": "true",
                "slam_params_file": slam_params,
            }.items(),
        ),
    ])
