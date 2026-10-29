"""ตัวเล่นโปรแกรมแขน (ar_program) + ตัวคุมมือ (amazinghand_node) — โปรแกรมที่มีท่ามือเล่นได้ครบ

  ros2 launch humanoi_games player.launch.py                 # = arm player
  ros2 launch humanoi_games player.launch.py hand:=false     # ไม่ต่อมือจริง (ตัวคุมมือแค่ log ท่า)

ตัวคุมมือเปิดพอร์ตมือ — ห้ามเปิดพร้อมเกม (arm rps) หรือ arm hand-calibrate (ใช้พอร์ตเดียวกัน)
มือสองข้าง: ตัวคุมมือหนึ่งตัวต่อแขนที่มีใน humanoi_games/config/hand_boards.yaml (ไม่มีไฟล์ = แขนซ้ายแบบเดิม)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def hand_nodes(context):
    from humanoi_games.amazinghand_node import hand_boards
    dry_run = LaunchConfiguration("hand").perform(context) != "true"
    return [Node(package="humanoi_games", executable="amazinghand_node", output="screen",
                 name="amazinghand" if arm == "left" else f"amazinghand_{arm}",
                 parameters=[{"arm": arm, "dry_run": dry_run}])
            for arm in (sorted(hand_boards()) or ["left"])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("speed", default_value="0.19", description="ความเร็วแขน rad/s (เพดาน 0.19)"),
        DeclareLaunchArgument("hand", default_value="true", description="false = ไม่ต่อมือจริง"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("ar_program"), "launch", "arm_program.launch.py"])),
            launch_arguments={"speed": LaunchConfiguration("speed")}.items()),
        OpaqueFunction(function=hand_nodes),
    ])
