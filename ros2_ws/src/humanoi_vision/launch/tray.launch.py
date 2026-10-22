"""กล้อง Astra Pro + TF ฐานแขน -> กล้อง (ผล hand-eye) + tray_detector ในคำสั่งเดียว (รันใน container humanoi_ros)

  docker exec -it humanoi_ros /entrypoint.sh ros2 launch humanoi_vision tray.launch.py
  ros2 launch humanoi_vision tray.launch.py marker_size:=0.118     # ป้ายพิมพ์ออกมาไม่ตรง 120 mm
  ros2 launch humanoi_vision tray.launch.py camera:=false          # กล้องเปิดอยู่แล้วจากที่อื่น (เช่น yolo.launch.py)

Foxglove (ws://EIT-Raspberry-Pi-5.local:8765): Image panel -> /tray/image/compressed
ป้าย: docs/calibration/aruco_tray_id1_120mm.pdf
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from humanoi_vision.launch_tools import camera_tf


def generate_launch_description():
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("humanoi_description"), "launch", "camera.launch.py"])),
        launch_arguments={"enable_point_cloud": "false"}.items(),
        condition=IfCondition(LaunchConfiguration("camera")),
    )
    detector = Node(
        package="humanoi_vision",
        executable="tray_detector",
        name="tray_detector",
        output="screen",
        parameters=[{
            "marker_id": ParameterValue(LaunchConfiguration("marker_id"), value_type=int),
            "marker_size": ParameterValue(LaunchConfiguration("marker_size"), value_type=float),
            "target_frame": ParameterValue(LaunchConfiguration("target_frame"), value_type=str),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument("camera", default_value="true", choices=["true", "false"],
                              description="เปิดกล้อง + Foxglove ด้วย (humanoi_description camera.launch.py)"),
        DeclareLaunchArgument("marker_id", default_value="1", description="id ป้าย ArUco 4x4_50 บนพื้นกล่อง"),
        DeclareLaunchArgument("marker_size", default_value="0.120",
                              description="ด้านสี่เหลี่ยมดำที่วัดจริงหลังพิมพ์ (เมตร)"),
        DeclareLaunchArgument("target_frame", default_value="left_base_link",
                              description="เฟรมที่ส่งตำแหน่งกล่องออกไป (/tray/pose_arm)"),
        DeclareLaunchArgument("camera_tf", default_value=PathJoinSubstitution(
                                  [FindPackageShare("humanoi_description"), "config", "camera_to_left_arm.yaml"]),
                              description="ไฟล์ผล hand_eye_calibrate ('' = ไม่ส่ง TF ฐานแขน -> กล้อง)"),
        camera,
        OpaqueFunction(function=camera_tf),
        detector,
    ])
