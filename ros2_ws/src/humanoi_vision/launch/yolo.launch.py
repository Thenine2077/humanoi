"""กล้อง Astra Pro + YOLO (Hailo-10H) + Foxglove ในคำสั่งเดียว (รันใน container humanoi_ros)

ตัวคิดบนการ์ด Hailo (scripts/hailo_yolo_server.py) เปิดเองตอน Pi บูตผ่าน systemd
(scripts/hailo-yolo-server.service) — เช็กด้วย systemctl status hailo-yolo-server บนเครื่อง Pi

ใน container:
  docker exec -it humanoi_ros /entrypoint.sh ros2 launch humanoi_vision yolo.launch.py
  ros2 launch humanoi_vision yolo.launch.py classes:=bottle,cup conf:=0.5   # เฉพาะขวด/แก้ว
  ros2 launch humanoi_vision yolo.launch.py camera:=false                   # กล้องเปิดอยู่แล้วจากที่อื่น
  bash ~/Humanoi/scripts/yolo_view.sh        # บน Pi นอก container: มีหน้าต่างภาพ + รายการที่เจอขึ้นจอ Pi

ดูผลใน Foxglove (ws://192.168.1.68:8765): Image panel ที่ /yolo/image/compressed (กรอบ + ระยะ)
3D panel: /yolo/markers + /camera/depth/points (Display frame = camera_color_optical_frame)
กรอบที่ตรวจเจอ (พิกเซล) อยู่ที่ /yolo/detections · พิกัด 3D (เมตร ในเฟรมกล้อง) อยู่ที่ /yolo/detections_3d
มีไฟล์ hand-eye (humanoi_description/config/camera_to_left_arm.yaml จาก hand_eye_calibrate) -> ส่ง TF
ฐานแขน -> camera_link ให้ด้วย จึงแปลงพิกัดกล้องเป็นพิกัดแขนผ่าน tf2 ได้
"""
import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def camera_tf(context):
    """TF ฐานแขน -> camera_link จากผล hand_eye_calibrate (ไม่มีไฟล์ = ข้าม แค่บอกใน log)"""
    path = LaunchConfiguration("camera_tf").perform(context)
    if not path or not os.path.exists(path):
        return [LogInfo(msg=f"ยังไม่มีผล hand-eye ({path}) — /yolo/detections_3d อยู่ในเฟรมกล้องอย่างเดียว")]
    with open(path) as f:
        c = yaml.safe_load(f)
    args = []
    for k in ("x", "y", "z", "qx", "qy", "qz", "qw"):
        args += [f"--{k}", str(c[k])]
    return [Node(package="tf2_ros", executable="static_transform_publisher", name="camera_tf",
                 arguments=args + ["--frame-id", c["parent_frame"], "--child-frame-id", c["child_frame"]])]


def generate_launch_description():
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("humanoi_description"), "launch", "camera.launch.py"])),
        condition=IfCondition(LaunchConfiguration("camera")),
    )

    yolo = Node(
        package="humanoi_vision",
        executable="yolo_node",
        name="yolo_node",
        output="screen",
        parameters=[{
            # บังคับชนิด: conf:=1 จะได้ไม่กลายเป็น int, classes:=person จะได้เป็น string เสมอ
            "conf": ParameterValue(LaunchConfiguration("conf"), value_type=float),
            "classes": ParameterValue(LaunchConfiguration("classes"), value_type=str),
            "server_port": ParameterValue(LaunchConfiguration("server_port"), value_type=int),
            "show": ParameterValue(LaunchConfiguration("show"), value_type=bool),
            "depth_offset": ParameterValue(LaunchConfiguration("depth_offset"), value_type=float),
            "table_mode": ParameterValue(LaunchConfiguration("table_mode"), value_type=str),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("camera", default_value="true", choices=["true", "false"],
                              description="เปิดกล้อง + Foxglove ด้วย (humanoi_description camera.launch.py)"),
        DeclareLaunchArgument("conf", default_value="0.4",
                              description="ความมั่นใจขั้นต่ำ 0-1"),
        DeclareLaunchArgument("classes", default_value="",
                              description="เฉพาะคลาส คั่นด้วยจุลภาค เช่น bottle,cup — ว่าง = ทุกคลาส (COCO 80)"),
        DeclareLaunchArgument("server_port", default_value="5555",
                              description="พอร์ตของ scripts/hailo_yolo_server.py บนเครื่อง Pi"),
        DeclareLaunchArgument("depth_offset", default_value="0.03",
                              description="เมตร บวกเข้ากับระยะ depth — Astra Pro อ่านสั้นกว่าจริง ~3 cm (วัด 15 ต.ค.)"),
        DeclareLaunchArgument("camera_tf", default_value=PathJoinSubstitution(
                                  [FindPackageShare("humanoi_description"), "config", "camera_to_left_arm.yaml"]),
                              description="ไฟล์ผล hand_eye_calibrate ('' = ไม่ส่ง TF ฐานแขน -> กล้อง)"),
        DeclareLaunchArgument("table_mode", default_value="auto", choices=["auto", "always", "off"],
                              description="ของที่ depth วัดไม่ได้ (ใกล้กว่า ~60 cm) คิดจากผิวโต๊ะ: auto = เฉพาะตอนนั้น"),
        DeclareLaunchArgument("show", default_value="false", choices=["true", "false"],
                              description="เปิดหน้าต่างภาพบนจอ Pi (ต้องมี DISPLAY — ใช้ scripts/yolo_view.sh)"),
        camera,
        OpaqueFunction(function=camera_tf),
        yolo,
    ])
