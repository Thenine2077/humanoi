#!/usr/bin/env bash
# ส่งโค้ด Humanoi จาก PC ไป Raspberry Pi (~/Humanoi) — ส่งเฉพาะ source ไม่ส่ง build/install/log
# รันใน Git Bash หรือ WSL บน PC:  bash scripts/sync_to_pi.sh [user@host]
# ต้องตั้ง SSH key ไว้แล้ว (เข้า Pi ได้โดยไม่ถามรหัส)
set -euo pipefail

PI="${1:-eitlaser@EIT-Raspberry-Pi-5.local}"
cd "$(dirname "$0")/.."

tar --exclude='*/build' --exclude='*/install' --exclude='*/log' --exclude='__pycache__' \
    -cf - docker docker-compose.yml scripts ros2_ws/src \
  | ssh -o BatchMode=yes "$PI" 'mkdir -p ~/Humanoi && tar -C ~/Humanoi -xf -'

# แขน AR4: บน PC อยู่คนละ workspace (Desktop/ar4/ar4_ws) แต่บน Pi รวมไว้ใน ros2_ws เดียวกัน
# ส่งแพ็กเกจที่ Pi ใช้คุมแขนจริง + ชุด MoveIt (จัดท่าบนจอ Pi ผ่าน Raspberry Pi Connect)
# ar_gz_bringup ส่งไปเพื่อ ar_moveit_jazzy.launch.py กับ config ของมัน (ส่วน Gazebo ไม่ได้ใช้บน Pi) — ไม่ส่ง ar_mirror
# programs.yaml (ท่าที่ teach ไว้) ถูกเขียนทับจากของ PC — สำรองของเดิมบน Pi ไว้เป็น .bak ก่อนทุกครั้ง
# ถ้า teach บน Pi ให้ดึงไฟล์กลับมา PC ก่อน sync ไม่งั้นท่าใหม่บน Pi จะถูกทับ (เหลือแค่ใน .bak)
AR4_SRC="${AR4_SRC:-$(cd .. && pwd)/ar4/ar4_ws/src}"
if [ -d "$AR4_SRC/ar_program" ]; then
  tar -C "$AR4_SRC" --exclude='__pycache__' --exclude='*.tmp' \
      -cf - ar_description ar_hardware_interface ar_program ar_moveit_config ar_gz_bringup ar_dual \
    | ssh -o BatchMode=yes "$PI" '
        f=~/Humanoi/ros2_ws/src/ar_program/config/programs.yaml
        [ -f "$f" ] && cp "$f" "$f.bak"
        mkdir -p ~/Humanoi/ros2_ws/src && tar -C ~/Humanoi/ros2_ws/src -xf -'
  echo "ส่งแพ็กเกจแขน AR4 (ar_description, ar_hardware_interface, ar_program) แล้ว"
else
  echo "ข้ามแพ็กเกจแขน AR4 — ไม่พบ $AR4_SRC"
fi

echo "ส่งไป $PI:~/Humanoi แล้ว — build ใน container ด้วย:"
echo "  ssh $PI 'docker exec humanoi_ros /entrypoint.sh bash -c \"cd /humanoi_ws && colcon build --symlink-install\"'"
