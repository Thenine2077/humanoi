"""เดโม: ลิดาร์ฮาร์ดแวร์จริง -> Nav2 -> หุ่นใน Gazebo หลบสิ่งกีดขวางตามของจริง

ยื่นมือไปหน้าลิดาร์จริง แล้วดูว่าหุ่นในซิมเบี่ยงหลบไหม

  # หน้าต่าง 1 — ซิม (ย้ายสแกนจำลองไป /scan_sim ให้ลิดาร์จริงเป็นเจ้าของ /scan คนเดียว)
  ros2 launch humanoi_description sim.launch.py scan_source:=real rviz:=true
  # หน้าต่าง 2 — ลิดาร์จริง + ตัวแปลงเวลา + Nav2 (ไม่มีแผนที่)
  ros2 launch humanoi_nav lidar_demo.launch.py

  RViz: Fixed Frame = odom (ไม่มี map ในโหมดนี้) -> 2D Goal Pose ห่างหุ่นสัก 1.5-2 m

สายข้อมูล:
  STL-19P --usbip--> ldlidar_ros2 --/scan_real (นาฬิกาจริง)-->
  scan_restamp --/scan (นาฬิกาซิม)--> costmap ของ Nav2 --/cmd_vel--> หุ่นใน Gazebo

ข้อจำกัดโดยธรรมชาติของเดโมนี้: ลิดาร์จริงวางนิ่ง แต่หุ่นในซิมวิ่ง สิ่งที่ลิดาร์เห็นจึง
"ติดไปกับหุ่น" — ใช้ทดสอบการตอบสนองต่อสิ่งกีดขวางที่โผล่มา (มือ) ได้ดี แต่ไม่ใช่การนำทาง
จริง วางลิดาร์ให้มีที่โล่งรอบตัวราว 1 m และตั้งเป้าหมายระยะสั้นๆ
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory("humanoi_nav")
    desc_share = get_package_share_directory("humanoi_description")

    # ลำดับสำคัญ: nav2.yaml ก่อน แล้วค่อยทับด้วยค่าของเดโม
    params = [
        os.path.join(nav_share, "config", "nav2.yaml"),
        os.path.join(nav_share, "config", "nav2_lidar_demo.yaml"),
        {"use_sim_time": True},
    ]

    # ไม่มี map_server / amcl — ดูเหตุผลใน config/nav2_lidar_demo.yaml
    lifecycle_nodes = ["controller_server", "planner_server", "behavior_server", "bt_navigator"]

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(desc_share, "launch", "lidar.launch.py")),
        launch_arguments={
            "port": LaunchConfiguration("port"),
            "scan_topic": "scan_real",
            "standalone": "false",  # TF ของ laser_link มาจาก robot_state_publisher ของซิม
        }.items(),
    )

    restamp = Node(
        package="humanoi_base",
        executable="scan_restamp",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "in_topic": "/scan_real",
            "out_topic": "/scan",
        }],
    )

    def nav_node(package, executable, name):
        return Node(package=package, executable=executable, name=name,
                    output="screen", parameters=params)

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0",
                              description="พอร์ต serial ของลิดาร์จริง"),
        lidar,
        restamp,
        nav_node("nav2_controller", "controller_server", "controller_server"),
        nav_node("nav2_planner", "planner_server", "planner_server"),
        nav_node("nav2_behaviors", "behavior_server", "behavior_server"),
        nav_node("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_nav",
            output="screen",
            parameters=[{"use_sim_time": True, "autostart": True,
                         "node_names": lifecycle_nodes}],
        ),
    ])
