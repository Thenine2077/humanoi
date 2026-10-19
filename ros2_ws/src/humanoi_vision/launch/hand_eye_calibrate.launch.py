"""hand-eye calibration: กล้อง Astra Pro + hand_eye_calibrate ในคำสั่งเดียว (รันใน container)

  bash ~/Humanoi/scripts/hand_eye_calibrate.sh          # บน Pi: มีหน้าต่างภาพบนจอ Pi
ต้องเปิดระบบแขนไว้ก่อน (arm start) ให้มี TF base_link -> link_6 และ MoveIt (arm moveit) ไว้พาแขนไปแต่ละท่า
เสร็จแล้วได้ humanoi_description/config/camera_to_left_arm.yaml — colcon build humanoi_description หนึ่งครั้ง
แล้ว yolo.launch.py จะส่ง TF ฐานแขน -> กล้อง ให้เอง
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("humanoi_description"), "launch", "camera.launch.py"])),
        # ใช้ค่าเลนส์ที่ calibrate แล้ว + depth จัดตรงภาพสี (ค่าเริ่มของ camera.launch.py) เหมือนตอนรัน yolo
        launch_arguments={"foxglove": "false", "enable_ir": "false", "enable_point_cloud": "false"}.items(),
    )
    calibrate = Node(
        package="humanoi_vision",
        executable="hand_eye_calibrate",
        name="hand_eye_calibrate",
        output="screen",
        parameters=[{
            "marker_size": ParameterValue(LaunchConfiguration("marker_size"), value_type=float),
            "marker_id": ParameterValue(LaunchConfiguration("marker_id"), value_type=int),
            "samples": ParameterValue(LaunchConfiguration("samples"), value_type=int),
            "depth_offset": ParameterValue(LaunchConfiguration("depth_offset"), value_type=float),
            "measure": ParameterValue(LaunchConfiguration("measure"), value_type=str),
            "show": ParameterValue(LaunchConfiguration("show"), value_type=bool),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument("marker_size", default_value="0.040",
                              description="ด้านของสี่เหลี่ยมดำที่วัดจริง (เมตร)"),
        DeclareLaunchArgument("marker_id", default_value="0", description="id ของป้าย ArUco 4x4_50 (-1 = ไหนก็ได้)"),
        DeclareLaunchArgument("samples", default_value="15", description="จำนวนท่าที่เก็บก่อนคำนวณ"),
        DeclareLaunchArgument("depth_offset", default_value="0.03",
                              description="ต้องเท่ากับ depth_offset ของ yolo.launch.py"),
        DeclareLaunchArgument("measure", default_value="pnp", choices=["pnp", "depth"],
                              description="วัดตำแหน่งป้ายจากขนาดป้ายในภาพ (pnp, ทุกระยะ) หรือ depth (ต้องห่าง > 0.6 m)"),
        DeclareLaunchArgument("show", default_value="false", choices=["true", "false"],
                              description="เปิดหน้าต่างภาพบนจอ Pi (ใช้ผ่าน scripts/hand_eye_calibrate.sh)"),
        camera,
        calibrate,
    ])
