"""เปิด Gazebo + วางหุ่น Humanoi เต็มตัว (ฐาน mecanum + เสา หัว กล้อง + แขน AR4 สองข้าง + มือ)
+ สะพาน ros_gz + robot_state_publisher + ตัวคุมแขนสองข้าง

  ros2 launch humanoi_description sim_with_arm.launch.py
  ros2 launch humanoi_description sim_with_arm.launch.py world:=room
  ros2 launch humanoi_description sim_with_arm.launch.py gui:=false     # ประหยัด CPU

สั่งฐานด้วย keyboard_teleop / Nav2 เหมือนเดิม (/cmd_vel) สั่งแขนด้วย MoveIt อีกหน้าต่าง:

  ros2 launch humanoi_description moveit_full.launch.py

หรือตัวเล่นโปรแกรม (ตัวคุมชื่อ left_arm_controller / right_arm_controller เหมือนแขนจริง):

  ros2 launch ar_program arm_program.launch.py

กล้องจำลองออกหัวข้อชื่อเดียวกับไดรเวอร์จริง: /camera/color/image_raw, /camera/color/camera_info,
/camera/depth/image_raw (32FC1 หน่วยเมตร — ตัวจริงเป็น 16UC1 หน่วยมม.), /camera/depth/points

ต้อง source ทั้งสอง workspace ก่อน (ar4_ws ก่อน แล้วค่อย ros2_ws) เพราะไฟล์นี้อ้าง ar_dual ของฝั่ง AR4:
  source /mnt/c/Users/User/Desktop/ar4/ar4_ws/install/setup.bash
  source /mnt/c/Users/User/Desktop/Humanoi/ros2_ws/install/setup.bash
"""
import os

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


def controllers_file() -> str:
    """ตัวคุมสองแขน — ไฟล์เดียวกับระบบแขนจริง (ar_dual)"""
    return os.path.join(get_package_share_directory("ar_dual"), "config", "controllers.yaml")


def full_robot_description(controllers_path: str = "") -> str:
    """URDF หุ่นเต็มตัว (moveit_full.launch.py เรียกใช้ด้วย ให้ MoveIt กับ Gazebo เห็นหุ่นตัวเดียวกัน)"""
    return xacro.process_file(
        os.path.join(
            get_package_share_directory("humanoi_description"),
            "urdf",
            "humanoi_full.urdf.xacro",
        ),
        mappings={"controllers": controllers_path or controllers_file()},
    ).toxml()


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory("humanoi_description")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    world = LaunchConfiguration("world").perform(context)
    world_file = os.path.join(pkg, "worlds", f"{world}.sdf")
    gui = LaunchConfiguration("gui").perform(context) == "true"
    gz_args = f"-r -v3 {world_file}" if gui else f"-r -s -v3 {world_file}"

    robot_description = {
        "robot_description": full_robot_description(),
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

    # ตัวคุมแขนต้องรอให้โมเดลถูก spawn เข้า gz ก่อน (controller_manager เกิดจากปลั๊กอิน
    # gz_ros2_control ที่อยู่ในตัวโมเดล ไม่ใช่โหนดที่รันแยก)
    controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "left_arm_controller", "right_arm_controller",
                   "--controller-manager-timeout", "60"],
    )

    return [
        gz,
        robot_state_publisher,
        spawn,
        bridge,
        rviz,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[controllers])),
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
