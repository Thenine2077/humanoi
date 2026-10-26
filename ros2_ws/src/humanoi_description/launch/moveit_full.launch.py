"""MoveIt2 + RViz สำหรับหุ่น Humanoi เต็มตัวในซิม (เปิดหลัง sim_with_arm.launch.py)

  ros2 launch humanoi_description moveit_full.launch.py

ต่างจาก ar_dual/launch/dual_moveit.launch.py (ระบบแขนจริง) แค่โมเดล:
  - robot_description / robot_description_semantic มาจากหุ่นเต็มตัว (มีฐาน ล้อ ไลดาร์ กล้อง)
  - kinematics / joint_limits / ompl / moveit_controllers ใช้ของ ar_dual ตรงๆ (ชื่อข้อต่อ/ตัวคุมเดียวกัน)
ใน RViz: Planning Group เลือก left_arm / right_arm / both_arms -> ลากลูกบอลหรือแท็บ Joints -> Plan & Execute

⚠️ RViz ถูกเปิดแบบ "เปล่า" โดยตั้งใจ ห้ามใส่ -d ตอนเปิด — ใต้ WSLg การใส่ -d ทำให้หน้าต่าง
   โชว์ช้าไปชนจังหวะที่ WSLg ลงทะเบียนรับ input แล้วเมาส์คลิกอะไรไม่ได้เลยตลอดอายุหน้าต่าง
   ให้โหลด config ทีหลังผ่านเมนู File > Recent Configs / Open Config: ~/.rviz2/dual_moveit.rviz
   (ลิงก์ไปที่ <ar4_ws>/install/ar_dual/share/ar_dual/config/dual_moveit.rviz)
"""
import importlib.util
import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    pkg = get_package_share_directory("humanoi_description")
    dual = get_package_share_directory("ar_dual")

    # ตัวสร้าง URDF ตัวเดียวกับ sim_with_arm.launch.py — MoveIt กับ Gazebo เห็นหุ่นตัวเดียวกันเป๊ะ
    spec = importlib.util.spec_from_file_location(
        "sim_with_arm_launch", os.path.join(pkg, "launch", "sim_with_arm.launch.py")
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    robot_description = {"robot_description": mod.full_robot_description()}
    robot_description_semantic = {"robot_description_semantic": xacro.process_file(
        os.path.join(pkg, "srdf", "humanoi_full.srdf.xacro")).toxml()}

    # พารามิเตอร์ที่เป็น list มาจากไฟล์ YAML ตรงๆ (กัน launch_ros แปลงเป็น !!python/tuple)
    kinematics = os.path.join(dual, "config", "kinematics.yaml")
    with open(os.path.join(dual, "config", "joint_limits.yaml"), encoding="utf-8") as f:
        planning = {"robot_description_planning": yaml.safe_load(f)}
    ompl = os.path.join(dual, "config", "ompl_planning_pipeline.yaml")
    controllers = os.path.join(dual, "config", "moveit_controllers.yaml")

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.1,
    }
    scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    common = [robot_description, robot_description_semantic, kinematics, planning, ompl,
              {"use_sim_time": use_sim_time}]

    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=common + [trajectory_execution, controllers, scene_monitor])
    rviz = Node(package="rviz2", executable="rviz2", name="rviz2_moveit", output="log",
                parameters=common)

    # ไม่รัน robot_state_publisher ที่นี่ — sim_with_arm.launch.py รันของตัวเองอยู่แล้ว
    return [rviz, move_group]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="ให้ MoveIt ใช้เวลาของซิม"),
        OpaqueFunction(function=launch_setup),
    ])
