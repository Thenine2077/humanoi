"""Nav2 (แบบเรียบ) — map_server + AMCL + planner + controller + behaviors + BT

รันเฉพาะโหนดหลัก ไม่เอา route_server / collision_monitor / docking_server / velocity_smoother
เพื่อลดจุดที่ต้องตั้ง param และ debug

  # หน้าต่าง 1
  ros2 launch humanoi_description sim.launch.py world:=room rviz:=true
  # หน้าต่าง 2
  ros2 launch humanoi_nav nav2.launch.py

RViz : เปลี่ยน Fixed Frame เป็น map → (ถ้าตำแหน่งเพี้ยน) 2D Pose Estimate → 2D Goal Pose
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory("humanoi_nav")
    params = os.path.join(nav_share, "config", "nav2.yaml")
    default_map = os.path.join(nav_share, "maps", "room.yaml")

    sim = {"use_sim_time": True}
    map_yaml = LaunchConfiguration("map")

    lifecycle_nodes = [
        "map_server",
        "amcl",
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
    ]

    def nav_node(package, executable, name):
        return Node(
            package=package,
            executable=executable,
            name=name,
            output="screen",
            parameters=[params, sim],
        )

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=default_map,
                              description="ไฟล์แผนที่ .yaml"),

        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[params, sim, {"yaml_filename": map_yaml}],
        ),
        nav_node("nav2_amcl", "amcl", "amcl"),
        nav_node("nav2_controller", "controller_server", "controller_server"),
        nav_node("nav2_planner", "planner_server", "planner_server"),
        nav_node("nav2_behaviors", "behavior_server", "behavior_server"),
        nav_node("nav2_bt_navigator", "bt_navigator", "bt_navigator"),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_nav",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "autostart": True,
                "node_names": lifecycle_nodes,
            }],
        ),
    ])
