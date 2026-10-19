#!/usr/bin/env bash
# hand-eye calibration: หาตำแหน่งกล้องเทียบกับฐานแขนซ้าย มีหน้าต่างภาพบนจอ Pi (ดู humanoi_vision/hand_eye_calibrate.py)
# ก่อนรัน: แปะป้าย ArUco 4x4_50 id 0 บนมือจับ + เปิดระบบแขน (arm start) + MoveIt (arm moveit) — ปิดหน้าต่างกล้องอื่นก่อน
# รันบน Pi นอก container:
#   bash ~/Humanoi/scripts/hand_eye_calibrate.sh                     # ป้าย 40 mm, 15 ท่า
#   bash ~/Humanoi/scripts/hand_eye_calibrate.sh marker_size:=0.05 samples:=20
# ออก: Ctrl+C ในหน้าต่าง terminal นี้
set -euo pipefail

if ! docker exec humanoi_ros pgrep -f robot_state_publisher >/dev/null 2>&1; then
  echo "✗ ยังไม่ได้เปิดระบบแขน — สั่ง  arm start  (และ  arm moveit  ไว้พาแขนไปแต่ละท่า) ก่อน"
  exit 1
fi

# หน้าต่างใน container วาดลงจอ :0 ของ Pi — คัดลอกคุกกี้ X11 ของ session ปัจจุบันทุกครั้ง (เหมือน calibrate_camera.sh)
docker cp "$HOME/.Xauthority" humanoi_ros:/root/.Xauthority

exec docker exec -it -e DISPLAY=:0 -e XAUTHORITY=/root/.Xauthority humanoi_ros /entrypoint.sh \
  ros2 launch humanoi_vision hand_eye_calibrate.launch.py show:=true "$@"
