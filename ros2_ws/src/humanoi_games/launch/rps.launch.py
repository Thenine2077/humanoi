"""เกมเป่ายิ้งฉุบ: rps_game + มือ (amazinghand_node) + ตาดูมือผู้เล่น (rps_vision + กล้อง Astra)

ระบบแขนต้องเปิดอยู่ก่อน (arm start / arm start sim) และสอนท่าตั้งไว้แล้ว (arm teach left rps_ready)

  ros2 launch humanoi_games rps.launch.py                                # ของจริงทั้งหมด
  ros2 launch humanoi_games rps.launch.py player:=random hand:=false     # ทดสอบ: ไม่มีกล้อง ไม่มีมือ
  ros2 launch humanoi_games rps.launch.py camera:=false                  # กล้องเปิดอยู่แล้วจากที่อื่น (เช่น yolo)
  ros2 topic pub --once /rps/play std_msgs/msg/Empty "{}"                # เล่น 1 ตา

  player:=camera  ดูมือผู้เล่นจากกล้อง (ค่าเริ่ม) · random = สุ่มแทนผู้เล่น · topic = ไม่เปิดตา รอข้อความ
                  /rps/player_gesture จากที่อื่น (เช่นพิมพ์เอง: ros2 topic pub -r 5 /rps/player_gesture ...)
  hand:=false     ไม่ต่อมือจริง (amazinghand_node แบบ dry_run แค่ log ท่า)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGS = {
    "arm": ("left", "แขนที่ติดมือ"),
    "player": ("camera", "camera | random | topic"),
    "camera": ("true", "เปิดกล้อง Astra ด้วย (false ถ้าเปิดอยู่แล้ว) — ใช้เฉพาะ player:=camera"),
    "hand": ("true", "false = ไม่ต่อมือจริง"),
    "hand_port": ("auto", "พอร์ตมือ (auto = บอร์ดของแขนนี้ตาม hand_boards.yaml / ไม่มีไฟล์ = หาเอง)"),
    "hand_side": ("auto", "ชุดมือ right/left — auto = ตาม hand_boards.yaml / arm hand-side"),
    "ready_pose": ("rps_ready", "ท่าตั้งก่อนเป่า"),
    "pump_deg": ("5.0", "ข้อ 1 ยกกี่องศา (8° = 11.6 วินาที, 5° = 8.6, 4° = 7.7, 3° = 6.7)"),
    "pump_count": ("3", "ขยับขึ้นลงกี่ครั้ง"),
    "robot_choice": ("random", "random | rock | paper | scissors (บังคับท่าหุ่น ไว้ทดสอบ/โชว์)"),
    "replay_timeout": ("60.0", "จบเกมแล้วรอนิ้วชี้ (= เล่นใหม่) กี่วินาที — 0 = รอตลอด"),
}


def generate_launch_description():
    cfg = {name: LaunchConfiguration(name) for name in ARGS}
    use_camera = PythonExpression(["'", cfg["player"], "' == 'camera'"])
    camera_launch = PathJoinSubstitution([FindPackageShare("humanoi_description"), "launch", "camera.launch.py"])
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=desc)
         for name, (default, desc) in ARGS.items()] + [
            Node(package="humanoi_games", executable="rps_game", output="screen", parameters=[{
                "arm": cfg["arm"],
                "ready_pose": cfg["ready_pose"],
                "pump_deg": cfg["pump_deg"],
                "pump_count": cfg["pump_count"],
                "robot_choice": cfg["robot_choice"],
                "replay_timeout": cfg["replay_timeout"],
                "player_source": PythonExpression(
                    ["'random' if '", cfg["player"], "' == 'random' else 'camera'"]),
            }]),
            Node(package="humanoi_games", executable="amazinghand_node", output="screen", parameters=[{
                "arm": cfg["arm"],
                "port": cfg["hand_port"],
                "side": cfg["hand_side"],
                "dry_run": PythonExpression(["'", cfg["hand"], "' != 'true'"]),
            }]),
            Node(package="humanoi_games", executable="rps_vision", output="screen", condition=IfCondition(use_camera)),
            Node(package="humanoi_games", executable="rps_hud", output="screen"),  # จอสถานะ (Foxglove)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_launch),
                condition=IfCondition(PythonExpression([use_camera, " and '", cfg["camera"], "' == 'true'"]))),
        ])
