"""ไดรเวอร์ลิดาร์จริง LDROBOT STL-19P (ชุด Waveshare D500) สำหรับหุ่น Humanoi

  # ต่อเข้ากับหุ่น (มี robot_state_publisher รันอยู่แล้วจาก sim.launch.py / sim_with_arm.launch.py)
  ros2 launch humanoi_description lidar.launch.py

  # ทดสอบลิดาร์เดี่ยวๆ ไม่มีหุ่น — เพิ่ม static TF ให้ RViz มีเฟรมอ้างอิง
  ros2 launch humanoi_description lidar.launch.py standalone:=true

ต่างจาก ldlidar_ros2/launch/ld19.launch.py ของผู้ขาย 3 จุด (ไฟล์นั้นไม่ถูกแก้):
  1. frame_id = "laser_link" ให้ตรงกับ mecanum_base.urdf.xacro ของเรา (ของผู้ขายใช้ "base_laser")
  2. ไม่ publish static TF เวลาต่อกับหุ่นจริง — robot_state_publisher ให้ TF
     base_footprint -> chassis_link -> laser_link อยู่แล้ว ถ้า publish ซ้ำจะชน
  3. range_min = 0.03 ตามสเปกจริงของ STL-19P (ของผู้ขายตั้ง 0.02 ซึ่งต่ำกว่าสเปก)

หมายเหตุ: STL-19P ใช้เฟรมโปรโตคอลและ baud เดียวกับ LD19 จึงใช้ product_name 'LDLiDAR_LD19'
ยืนยันแล้วด้วยของจริง: 501 จุด/รอบ, 0.720°/จุด, 10.000 Hz, วัดได้ 89% ของจุดทั้งหมด

USB: ลิดาร์ใช้ชิป CP2102 -> /dev/ttyUSB0 ใน WSL ต้อง usbipd attach ก่อน (busid ของ 10c4:ea60)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port")
    frame_id = LaunchConfiguration("frame_id")

    lidar = Node(
        package="ldlidar_ros2",
        executable="ldlidar_ros2_node",
        name="ldlidar_publisher",
        output="screen",
        parameters=[{
            "product_name": "LDLiDAR_LD19",
            "laser_scan_topic_name": LaunchConfiguration("scan_topic"),
            "point_cloud_2d_topic_name": "pointcloud2d",
            "frame_id": frame_id,
            "port_name": port,
            "serial_baudrate": 230400,
            "laser_scan_dir": True,
            # ตัดช่วงมุมที่ลำแสงกวาดโดนตัวหุ่นเอง (แขน AR4 / ฝาบนฐาน) — ยังปิดไว้ก่อน
            # เพราะยังไม่ได้วัดฐานจริงว่าจะติดลิดาร์ตรงไหน ถ้าไม่ตัด costmap ของ Nav2
            # จะมองแขนตัวเองเป็นสิ่งกีดขวางถาวรแล้วหุ่นจะขยับไม่ได้เลย
            "enable_angle_crop_func": LaunchConfiguration("angle_crop"),
            "angle_crop_min": LaunchConfiguration("angle_crop_min"),
            "angle_crop_max": LaunchConfiguration("angle_crop_max"),
            "range_min": 0.03,
            "range_max": 12.0,
        }],
    )

    # ใช้เฉพาะตอนทดสอบลิดาร์เดี่ยวๆ ที่ไม่มี robot_state_publisher
    # ค่า xyz คำนวณจาก mecanum_base.urdf.xacro ให้ตรงกัน:
    #   base_footprint -> chassis_link = wheel_radius + clearance = 0.075 + 0.04 = 0.115
    #   chassis_link   -> laser_link   = (wheelbase_half, 0, base_z/2 + 0.04) = (0.16, 0, 0.10)
    standalone_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_footprint_to_laser_link",
        arguments=["--x", "0.16", "--y", "0", "--z", "0.215",
                   "--frame-id", "base_footprint", "--child-frame-id", "laser_link"],
        condition=IfCondition(LaunchConfiguration("standalone")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0",
                              description="พอร์ต serial ของลิดาร์ (CP2102)"),
        DeclareLaunchArgument("scan_topic", default_value="scan",
                              description="topic ที่ส่งสแกนออก — ตอนป้อนหุ่นในซิมใช้ scan_real "
                                          "แล้วให้ scan_restamp แปลงเวลาเป็น /scan"),
        DeclareLaunchArgument("frame_id", default_value="laser_link",
                              description="เฟรมของ /scan ต้องตรงกับ URDF"),
        DeclareLaunchArgument("standalone", default_value="false", choices=["true", "false"],
                              description="true = ทดสอบเดี่ยวๆ ไม่มีหุ่น จะ publish static TF ให้ด้วย"),
        DeclareLaunchArgument("angle_crop", default_value="false", choices=["true", "false"],
                              description="ตัดช่วงมุมที่กวาดโดนตัวหุ่นเองหรือไม่"),
        DeclareLaunchArgument("angle_crop_min", default_value="135.0"),
        DeclareLaunchArgument("angle_crop_max", default_value="225.0"),
        lidar,
        standalone_tf,
    ])
