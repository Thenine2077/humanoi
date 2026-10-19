"""calibrate ค่าเลนส์กล้องสี: กล้อง Astra Pro + calibrate_color + Foxglove ในคำสั่งเดียว (รันใน container)

  bash ~/Humanoi/scripts/calibrate_camera.sh square:=0.03 cols:=8 rows:=5   # บน Pi: มีหน้าต่างภาพบนจอ Pi
  docker exec -it humanoi_ros /entrypoint.sh ros2 launch humanoi_vision calibrate_color.launch.py   # ดูใน Foxglove

กระดาษ: docs/calibration/checkerboard_9x6_25mm.pdf พิมพ์ A4 แบบ 100% ติดแผ่นแข็งเรียบ
ขนาดช่องไม่มีผลกับค่าเลนส์ แต่ใส่ให้ตรงของจริงไว้ (วัด 10 ช่องแล้วหาร 10) — cols/rows = จำนวน "มุมด้านใน"
ดูใน Foxglove (ws://192.168.1.68:8765): Image panel ที่ /calibration/image/compressed
เสร็จแล้วได้ humanoi_description/config/astra_color.yaml — colcon build humanoi_description หนึ่งครั้ง
แล้ว camera.launch.py จะโหลดไฟล์นี้ให้ไดรเวอร์เอง
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
        # ตอน calibrate ต้องเห็นภาพดิบของเลนส์ — ไม่โหลดค่าเลนส์เดิม ไม่ใช้ depth (ปิดเพื่อลด CPU)
        launch_arguments={"color_info_url": "", "enable_ir": "false",
                          "enable_point_cloud": "false"}.items(),
    )
    calibrate = Node(
        package="humanoi_vision",
        executable="calibrate_color",
        name="calibrate_color",
        output="screen",
        parameters=[{
            "square": ParameterValue(LaunchConfiguration("square"), value_type=float),
            "cols": ParameterValue(LaunchConfiguration("cols"), value_type=int),
            "rows": ParameterValue(LaunchConfiguration("rows"), value_type=int),
            "show": ParameterValue(LaunchConfiguration("show"), value_type=bool),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument("square", default_value="0.025",
                              description="ขนาดช่องจริงที่พิมพ์ได้ (เมตร) เช่น 0.03"),
        DeclareLaunchArgument("cols", default_value="9", description="มุมด้านในแนวนอน (จำนวนช่อง - 1)"),
        DeclareLaunchArgument("rows", default_value="6", description="มุมด้านในแนวตั้ง (จำนวนช่อง - 1)"),
        DeclareLaunchArgument("show", default_value="false", choices=["true", "false"],
                              description="เปิดหน้าต่างภาพบนจอ Pi (ใช้ผ่าน scripts/calibrate_camera.sh)"),
        camera,
        calibrate,
    ])
