"""กล้อง depth Orbbec Astra Pro + ภาพสีบีบอัด + Foxglove bridge ในคำสั่งเดียว (รันบน Pi)

  # ใน container humanoi_ros บน Pi
  docker exec -it humanoi_ros /entrypoint.sh ros2 launch humanoi_description camera.launch.py

  # มี foxglove_bridge รันอยู่แล้วจากที่อื่น — เปิดแค่กล้อง + บีบอัด
  ros2 launch humanoi_description camera.launch.py foxglove:=false

ดูภาพจาก PC: Foxglove -> Open connection -> Foxglove WebSocket -> ws://EIT-Raspberry-Pi-5.local:8765
แล้วเปิด Image panel ที่ /camera/color/image_raw/compressed

ทำไมต้องบีบอัด: Pi ต่อ WiFi อย่างเดียว ภาพสีดิบ 640x480@30 ~27 MB/s ส่งไม่ทัน (SSH หลุดได้)
JPEG เหลือ ~2.5 MB/s ไดรเวอร์ไม่ได้ publish /compressed เองจึงต้องมี republish
อย่าเปิด /camera/depth/points ใน Foxglove ผ่าน WiFi — หนักเกินไป

ไดรเวอร์ astra_camera ไม่ได้อยู่ใน ros2_ws แต่ build ไว้ใน image ที่ /opt/astra_ws
(ดู docker/Dockerfile) ไฟล์นี้จึงรันได้เฉพาะบน Pi — PC/WSL ไม่มีไดรเวอร์

ระวัง: กล้องสีเปิดได้ทีละโปรแกรม ห้ามเปิด guvcview บนจอ Pi พร้อมกัน
(เคยวน error จน ~/.xsession-errors โต 6 GB ดิสก์เต็ม)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("astra_camera"), "launch", "astra_pro.launch.xml"])),
        launch_arguments={
            "enable_ir": LaunchConfiguration("enable_ir"),
            "enable_point_cloud": LaunchConfiguration("enable_point_cloud"),
            "depth_registration": LaunchConfiguration("depth_registration"),
            "color_info_url": LaunchConfiguration("color_info_url"),
        }.items(),
    )

    # ภาพสีดิบ -> JPEG ที่ /camera/color/image_raw/compressed
    # ต้อง remap "out/compressed" ตรงตัว — ROS 2 remap ทั้งชื่อ "out:=" จะไม่มีผลกับ topic ของ plugin
    compress = Node(
        package="image_transport",
        executable="republish",
        name="color_compress",
        output="screen",
        parameters=[{"in_transport": "raw", "out_transport": "compressed"}],
        remappings=[("in", "/camera/color/image_raw"),
                    ("out/compressed", "/camera/color/image_raw/compressed")],
    )

    foxglove = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("foxglove_bridge"), "launch", "foxglove_bridge_launch.xml"])),
        launch_arguments={"port": LaunchConfiguration("foxglove_port")}.items(),
        condition=IfCondition(LaunchConfiguration("foxglove")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("foxglove", default_value="true", choices=["true", "false"],
                              description="เปิด foxglove_bridge ด้วยหรือไม่"),
        DeclareLaunchArgument("foxglove_port", default_value="8765",
                              description="พอร์ต WebSocket ของ Foxglove"),
        DeclareLaunchArgument("enable_ir", default_value="true", choices=["true", "false"],
                              description="ภาพอินฟราเรด — ยังไม่มีใครใช้ ปิดได้เพื่อลด CPU"),
        DeclareLaunchArgument("enable_point_cloud", default_value="true", choices=["true", "false"],
                              description="point cloud /camera/depth/points — Nav2 จะใช้ภายหลัง"),
        # true = กล้องจัด depth ให้ตรงกับภาพสีทีละพิกเซล (frame_id = camera_color_optical_frame)
        # ต้องเปิดไว้ถ้าจะเอาพิกัดพิกเซลจาก YOLO (บนภาพสี) ไปอ่าน depth — humanoi_vision ใช้แบบนี้
        DeclareLaunchArgument("depth_registration", default_value="true", choices=["true", "false"],
                              description="จัด depth ให้ตรงกับภาพสี (ต้องเปิดสำหรับพิกัด 3D ของ YOLO)"),
        # กล้องตัวนี้ไม่มีค่าเลนส์จากโรงงาน (get_camera_params ได้ NaN) ไดรเวอร์จึงใช้ค่ากลาง fx=fy=570.34
        # ค่าที่ calibrate เอง (humanoi_vision calibrate_color) อยู่ที่ config/astra_color.yaml — ถ้ายังไม่มีไฟล์
        # ไดรเวอร์เตือนว่าหาไม่เจอแล้วใช้ค่ากลางตามเดิม ไม่พัง (ไฟล์ใหม่ต้อง colcon build แพ็กเกจนี้หนึ่งครั้ง)
        DeclareLaunchArgument("color_info_url", default_value=[
                                  "file://", PathJoinSubstitution([FindPackageShare("humanoi_description"),
                                                                   "config", "astra_color.yaml"])],
                              description="ไฟล์ค่าเลนส์กล้องสี (ว่าง = ค่ากลางของไดรเวอร์)"),
        camera,
        compress,
        foxglove,
    ])
