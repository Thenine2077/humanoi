#!/usr/bin/env bash
# เปิด MoveIt + RViz ของแขน AR4 บนหน้าจอ Pi (ดูและคลิกผ่าน Raspberry Pi Connect)
# รันบน Pi นอก container:  bash ~/Humanoi/scripts/arm_moveit.sh
#
# ต้องเปิดระบบแขน (ar_hardware.launch.py) ไว้ก่อน — กด Plan & Execute ใน RViz แล้ว "แขนจริง" ขยับ
# ปิด: Ctrl+C ในหน้าต่างนี้ (ระบบแขนกับ arm_program ยังเปิดอยู่ต่อ)
set -euo pipefail

# RViz ใน container วาดลงจอ :0 ของ Pi ผ่าน X11 (network_mode: host ทำให้เห็น socket ของจอ)
# แต่ต้องมีคุกกี้ของ session ที่ล็อกอินอยู่ — คุกกี้เปลี่ยนทุกครั้งที่ล็อกอินใหม่ จึงคัดลอกใหม่ทุกครั้ง
docker cp "$HOME/.Xauthority" humanoi_ros:/root/.Xauthority

exec docker exec -it -e DISPLAY=:0 -e XAUTHORITY=/root/.Xauthority humanoi_ros /entrypoint.sh \
  ros2 launch ar_gz_bringup ar_moveit_jazzy.launch.py \
  rviz_config:=/humanoi_ws/src/ar_gz_bringup/config/ar_moveit.rviz
