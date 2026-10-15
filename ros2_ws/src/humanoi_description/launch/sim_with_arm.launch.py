"""เปิด Gazebo + วางหุ่นรวม (ฐาน mecanum + แขน AR4) + สะพาน ros_gz + robot_state_publisher

  ros2 launch humanoi_description sim_with_arm.launch.py
  ros2 launch humanoi_description sim_with_arm.launch.py world:=room
  ros2 launch humanoi_description sim_with_arm.launch.py gui:=false     # ประหยัด CPU

สั่งฐานด้วย keyboard_teleop / Nav2 เหมือนเดิม (/cmd_vel) สั่งแขนด้วย MoveIt อีกหน้าต่าง:

  ros2 launch humanoi_description moveit_full.launch.py use_sim_time:=true

ต้อง source ทั้งสอง workspace ก่อน (ar4_ws ก่อน แล้วค่อย ros2_ws) เพราะไฟล์นี้อ้าง
ar_description ของฝั่ง AR4:
  source /mnt/c/Users/User/Desktop/ar4/ar4_ws/install/setup.bash
  source /mnt/c/Users/User/Desktop/Humanoi/ros2_ws/install/setup.bash
"""
import os
import re

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def full_robot_description(arm_controllers_path: str) -> str:
    """URDF หุ่นรวม พร้อมแก้ effort/velocity limit ของแขนที่ผู้ขายตั้งเป็น 0

    ar_description/urdf/ar_macro.xacro ของผู้ขาย (อ่านอย่างเดียว) ตั้ง effort="0" ไว้แทบทุก
    ข้อต่อ และ joint_2 ตั้ง velocity="0" ด้วย ค่าพวกนี้ไม่มีผลกับแขนจริง (ฝั่งจริงสั่งผ่าน
    serial ไม่ได้อ่านค่านี้) แต่ในซิม ฟิสิกส์ใช้ตรงๆ — effort=0 แปลว่ามอเตอร์ออกแรงได้ 0 Nm
    ข้อต่อจึงเป็นอิสระ แรงโน้มถ่วงดึงแขนตกจนชน hard-stop ภายในไม่กี่สิบวินาที
    ตรรกะเดียวกับ ar_gz_bringup/launch/ar_gz.launch.py ฝั่ง AR4 เดี่ยว
    """
    urdf = xacro.process_file(
        os.path.join(
            get_package_share_directory("humanoi_description"),
            "urdf",
            "humanoi_full.urdf.xacro",
        ),
        mappings={"arm_controllers": arm_controllers_path},
    ).toxml()
    urdf = re.sub(r'(<limit[^>]*?)effort="[^"]*"', r'\1effort="100"', urdf)
    urdf = re.sub(r'(<limit[^>]*?)velocity="[^"]*"', r'\1velocity="3.0"', urdf)
    return urdf


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory("humanoi_description")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    world = LaunchConfiguration("world").perform(context)
    world_file = os.path.join(pkg, "worlds", f"{world}.sdf")
    gui = LaunchConfiguration("gui").perform(context) == "true"
    gz_args = f"-r -v3 {world_file}" if gui else f"-r -s -v3 {world_file}"

    arm_controllers = os.path.join(pkg, "config", "arm_controllers.yaml")
    robot_description = {
        "robot_description": full_robot_description(arm_controllers),
        "use_sim_time": True,
    }

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
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
        parameters=[
            {"config_file": os.path.join(pkg, "config", "gz_bridge.yaml"),
             "use_sim_time": True}
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", os.path.join(pkg, "rviz", "view.rviz")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # controller ของแขนต้องรอให้โมเดลถูก spawn เข้า gz ก่อน (controller_manager เกิดจาก
    # ปลั๊กอิน gz_ros2_control ที่อยู่ในตัวโมเดล ไม่ใช่โหนดที่รันแยก) — ลำดับเดียวกับ
    # ar_gz_bringup/launch/ar_gz.launch.py
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "60"],
    )
    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager-timeout", "60"],
    )

    return [
        gz,
        robot_state_publisher,
        spawn,
        bridge,
        rviz,
        RegisterEventHandler(
            OnProcessExit(target_action=spawn, on_exit=[joint_state_broadcaster])
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[joint_trajectory_controller],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false",
                              description="เปิด RViz ของฝั่งฐาน/Nav2 ด้วยหรือไม่"),
        DeclareLaunchArgument("world", default_value="empty",
                              description="ชื่อไฟล์โลกใน worlds/ (empty | room | room_obstacle | room_walker)"),
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"],
                              description="เปิดหน้าต่าง 3D ของ Gazebo หรือไม่"),
        OpaqueFunction(function=launch_setup),
    ])
