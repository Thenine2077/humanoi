"""MoveIt2 + RViz สำหรับ "หุ่นรวม" (ฐาน mecanum + แขน AR4)

  ros2 launch humanoi_description moveit_full.launch.py use_sim_time:=true

ต่างจาก ar_gz_bringup/launch/ar_moveit_jazzy.launch.py (ฝั่งแขนเดี่ยว) แค่ 2 จุด:
  - robot_description / robot_description_semantic มาจากหุ่นรวม ไม่ใช่แขนเดี่ยว
  - โครงสร้างพารามิเตอร์อื่นๆ (ompl pipeline, moveit_controllers, kinematics, joint_limits)
    reuse ของเดิมทั้งหมด เพราะชื่อข้อต่อ/ลิงก์ของแขนไม่ได้เปลี่ยนเลย

⚠️ RViz ถูกเปิดแบบ "เปล่า" โดยตั้งใจ ห้ามใส่ -d ตอนเปิด — ใต้ WSLg การใส่ -d ทำให้หน้าต่าง
   โชว์ช้าไปชนจังหวะที่ WSLg ลงทะเบียนรับ input แล้วเมาส์คลิกอะไรไม่ได้เลยตลอดอายุหน้าต่าง
   (แม้แต่ปุ่ม minimize) ให้โหลด config ทีหลังผ่านเมนู File > Open Config แทน:
     <ar4_ws>/install/ar_gz_bringup/share/ar_gz_bringup/config/ar_moveit.rviz
   แล้วเปลี่ยน Fixed Frame เป็น "odom" (ของเดิมตั้งไว้สำหรับแขนที่ยึดกับพื้น)
"""
import importlib.util
import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, file_name):
    path = os.path.join(get_package_share_directory(package_name), file_name)
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    pkg = get_package_share_directory("humanoi_description")

    # ใช้ตัวสร้าง URDF ตัวเดียวกับ sim_with_arm.launch.py เพื่อให้ MoveIt กับ Gazebo เห็น
    # หุ่นตัวเดียวกันเป๊ะ (รวมถึง effort/velocity ที่ถูกแก้จาก 0 แล้ว — ถ้า MoveIt เห็น
    # velocity=0 ของ joint_2 ตามไฟล์ผู้ขาย ตัว time parameterization จะคำนวณ trajectory ไม่ได้)
    spec = importlib.util.spec_from_file_location(
        "sim_with_arm_launch", os.path.join(pkg, "launch", "sim_with_arm.launch.py")
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    urdf = mod.full_robot_description(os.path.join(pkg, "config", "arm_controllers.yaml"))

    srdf = xacro.process_file(os.path.join(pkg, "srdf", "humanoi_full.srdf.xacro")).toxml()

    robot_description = {"robot_description": urdf}
    robot_description_semantic = {"robot_description_semantic": srdf}

    # ชื่อข้อต่อ/ลิงก์ของแขนไม่เปลี่ยน จึง reuse ไฟล์ของผู้ขาย + ของ ar_gz_bringup ได้ตรงๆ
    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("ar_moveit_config"), "config", "kinematics.yaml"]
    )
    robot_description_planning = {
        "robot_description_planning": load_yaml(
            "ar_moveit_config", os.path.join("config", "joint_limits.yaml")
        )
    }
    ompl_planning_pipeline_config = PathJoinSubstitution(
        [FindPackageShare("ar_gz_bringup"), "config", "ompl_planning_pipeline.yaml"]
    )
    moveit_controllers = PathJoinSubstitution(
        [FindPackageShare("ar_gz_bringup"), "config", "moveit_controllers.yaml"]
    )

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.1,
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    common = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        ompl_planning_pipeline_config,
        {"use_sim_time": use_sim_time},
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=common + [trajectory_execution, moveit_controllers,
                             planning_scene_monitor_parameters],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        parameters=common,
    )

    # ไม่รัน robot_state_publisher ที่นี่ — sim_with_arm.launch.py รันของตัวเองอยู่แล้ว
    return [rviz_node, move_group_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="ให้ MoveIt ใช้เวลาของซิม"),
        OpaqueFunction(function=launch_setup),
    ])
