"""SLAM กับลิดาร์จริง (ยังไม่มี encoder) — รันบน Pi ใน container humanoi_ros

  ros2 launch humanoi_nav real_slam.launch.py

ลิดาร์ STL-19P -> /scan -> rf2o_laser_odometry -> /odom + TF odom->base_footprint
                       +-> slam_toolbox -> /map + TF map->odom
ดูใน Foxglove (ws://192.168.1.68:8765): 3D panel, Display frame = map, เปิด /map กับ /scan

ถือลิดาร์เดินช้าๆ (หรือขับฐาน) ให้ทั่วห้อง — rf2o เดาการเคลื่อนที่จากสแกนที่ต่อเนื่องกัน
หมุนเร็วหรือเดินเร็วเกินจะหลุด ให้ช้าไว้ก่อน แล้วเซฟแผนที่:
  ros2 run nav2_map_server map_saver_cli -f /humanoi_ws/src/humanoi_nav/maps/real_room

standalone:=true (ค่าเริ่มต้น) = ยังไม่มี robot_state_publisher บน Pi
  lidar.launch.py จะ publish static TF base_footprint->laser_link ให้เอง
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LIDAR_BY_ID = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"


def generate_launch_description():
    desc_share = get_package_share_directory("humanoi_description")
    nav_share = get_package_share_directory("humanoi_nav")
    slam_share = get_package_share_directory("slam_toolbox")

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(desc_share, "launch", "lidar.launch.py")),
        launch_arguments={
            "port": LaunchConfiguration("lidar_port"),
            "standalone": LaunchConfiguration("standalone"),
        }.items(),
    )

    # init_pose_from_topic ต้องว่าง ไม่งั้นมันรอ /base_pose_ground_truth (มีแค่ในซิมของผู้เขียน)
    rf2o = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        parameters=[{
            "laser_scan_topic": "/scan",
            "odom_topic": "/odom",
            "publish_tf": True,
            "base_frame_id": "base_footprint",
            "odom_frame_id": "odom",
            "init_pose_from_topic": "",
            "freq": 10.0,  # เท่ารอบลิดาร์
        }],
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_share, "launch", "online_async_launch.py")),
        launch_arguments={
            "use_sim_time": "false",
            "slam_params_file": os.path.join(nav_share, "config", "slam.yaml"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("lidar_port", default_value=LIDAR_BY_ID,
                              description="พอร์ตลิดาร์ (by-id ไม่สลับกับ Mega / บอร์ดมือ)"),
        DeclareLaunchArgument("standalone", default_value="true", choices=["true", "false"],
                              description="true = ไม่มี robot_state_publisher ให้ลิดาร์ publish TF เอง"),
        lidar,
        rf2o,
        slam,
    ])
