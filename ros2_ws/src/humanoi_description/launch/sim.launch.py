"""เปิด Gazebo + วางหุ่น mecanum + สะพาน ros_gz + robot_state_publisher

  ros2 launch humanoi_description sim.launch.py
  ros2 launch humanoi_description sim.launch.py rviz:=true
  ros2 launch humanoi_description sim.launch.py world:=room       # ห้อง + สิ่งกีดขวาง
  ros2 launch humanoi_description sim.launch.py scan_source:=real # ใช้ลิดาร์ฮาร์ดแวร์จริงแทน

scan_source:=real — ย้ายสแกนจากลิดาร์จำลองไปที่ /scan_sim แทน /scan เพื่อเปิดทางให้
ลิดาร์จริง (ผ่าน humanoi_base scan_restamp) เป็นเจ้าของ /scan แต่ผู้เดียว ถ้าปล่อยสองแหล่ง
ยิง /scan พร้อมกัน costmap จะรับสลับกัน แล้วสแกนของซิม (ซึ่งเห็นโลกว่าง) จะ raytrace
ลบสิ่งกีดขวางที่ลิดาร์จริงเพิ่งเห็นทิ้งทุกรอบ ของจริงจะกระพริบติดๆดับๆ
"""
import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory("humanoi_description")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    xacro_file = os.path.join(pkg, "urdf", "mecanum_base.urdf.xacro")
    bridge_cfg = os.path.join(pkg, "config", "gz_bridge.yaml")
    rviz_file = os.path.join(pkg, "rviz", "view.rviz")

    # scan_source:=real -> สแกนจำลองไปอยู่ /scan_sim (ยังดูเทียบใน RViz ได้)
    #                     และใช้ RViz config ที่ Fixed Frame = odom
    # (view.rviz ตั้งไว้ที่ map ซึ่งมาจาก AMCL — โหมดลิดาร์จริงไม่รัน AMCL จึงไม่มีเฟรม map
    #  RViz จะแปลงอะไรไม่ได้เลย แล้วทิ้งทุก message ด้วย "discarding message because the
    #  queue is full" ทั้งสแกนทั้ง odom — หน้าจอดูเหมือนข้อมูลหายไปเฉยๆ ทั้งที่ยังไหลอยู่)
    if LaunchConfiguration("scan_source").perform(context) == "real":
        rviz_file = os.path.join(pkg, "rviz", "view_lidar_demo.rviz")
        with open(bridge_cfg, encoding="utf-8") as f:
            entries = yaml.safe_load(f)
        for e in entries:
            if e.get("gz_topic_name") == "/scan":
                e["ros_topic_name"] = "/scan_sim"
        tmp = tempfile.NamedTemporaryFile(
            "w", suffix="_gz_bridge.yaml", delete=False, encoding="utf-8"
        )
        yaml.safe_dump(entries, tmp, allow_unicode=True)
        tmp.close()
        bridge_cfg = tmp.name

    world = LaunchConfiguration("world").perform(context)
    world_file = os.path.join(pkg, "worlds", f"{world}.sdf")
    gui = LaunchConfiguration("gui").perform(context) == "true"

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]), value_type=str
    )

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        # gui:=false -> รันแค่ server (-s) ไม่เปิดหน้าต่าง 3D ประหยัด CPU ได้มาก
        # วัดจริงตอนเปิดทั้ง Gazebo GUI + RViz + Nav2: GUI ของ gz กิน ~145% CPU และทำให้ซิม
        # วิ่งได้แค่ 0.61 เท่าของเวลาจริง ดูหุ่น/สแกน/costmap ใน RViz แทนได้ครบ
        launch_arguments={"gz_args": f"-r -v3 {world_file}" if gui
                          else f"-r -s -v3 {world_file}"}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "humanoi", "-z", "0.0"],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{"config_file": bridge_cfg, "use_sim_time": True}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_file],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return [gz, robot_state_publisher, spawn, bridge, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false",
                              description="เปิด RViz ด้วยหรือไม่"),
        DeclareLaunchArgument("world", default_value="empty",
                              description="ชื่อไฟล์โลกใน worlds/ (empty | room)"),
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"],
                              description="เปิดหน้าต่าง 3D ของ Gazebo หรือไม่ (false = ประหยัด CPU)"),
        DeclareLaunchArgument("scan_source", default_value="sim", choices=["sim", "real"],
                              description="sim = ลิดาร์จำลองเป็น /scan, real = ใช้ลิดาร์ฮาร์ดแวร์จริง"),
        OpaqueFunction(function=launch_setup),
    ])
