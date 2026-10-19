#!/usr/bin/env bash
# ตรวจระยะด้วยตลับเมตร ว่า depth หรือค่าเลนส์กล้องสีที่คลาด (ดู humanoi_vision/check_depth_scale.py)
# รันบน Pi นอก container: bash ~/Humanoi/scripts/check_depth_scale.sh
#   ไฟล์ค่าเลนส์อื่น: ... -p calib:=/humanoi_ws/src/humanoi_description/config/<ไฟล์>
#   ขนาดช่องอื่น:    ... -p square:=0.025 -p cols:=9 -p rows:=6   (ค่าเริ่ม = ตาราง 9x6 ช่อง ช่องละ 28.2 mm)
# เปิดกล้องให้เองถ้ายังไม่มีใครเปิด แล้วปิดให้ตอนจบ · ออก: Ctrl+C
set -euo pipefail

# หน้าต่างใน container วาดลงจอ :0 ของ Pi — คัดลอกคุกกี้ X11 ของ session ปัจจุบันทุกครั้ง (เหมือน calibrate_camera.sh)
docker cp "$HOME/.Xauthority" humanoi_ros:/root/.Xauthority

exec docker exec -it -e DISPLAY=:0 -e XAUTHORITY=/root/.Xauthority humanoi_ros /entrypoint.sh \
  ros2 run humanoi_vision check_depth_scale --ros-args -p show:=true "$@"
