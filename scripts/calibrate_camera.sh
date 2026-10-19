#!/usr/bin/env bash
# calibrate ค่าเลนส์กล้องสี Astra Pro โดยมีหน้าต่างภาพขึ้นบนจอ Pi (เห็นมุมตารางที่จับได้ + แถบความคืบหน้า)
# รันบน Pi นอก container:
#   bash ~/Humanoi/scripts/calibrate_camera.sh                              # ตาราง 10x7 ช่อง ช่องละ 2.5 cm (PDF ของเรา)
#   bash ~/Humanoi/scripts/calibrate_camera.sh square:=0.0282 cols:=8 rows:=5  # ตาราง 9x6 ช่อง ที่พิมพ์ได้ช่องละ 28.2 mm
# cols/rows = จำนวน "มุมด้านใน" = จำนวนช่อง - 1 · ปิด: Ctrl+C ในหน้าต่างนี้
# square = ขนาดช่องที่ "วัดจริง" บนกระดาษ (เมตร) — เครื่องพิมพ์มักย่อ (ตาราง 30 mm พิมพ์ออกมา 28.2 mm) ผิดแล้วระยะจะผิดตาม
set -euo pipefail

# หน้าต่างใน container วาดลงจอ :0 ของ Pi ผ่าน X11 (network_mode: host) — ต้องมีคุกกี้ของ session ที่ล็อกอินอยู่
# คุกกี้เปลี่ยนทุกครั้งที่ล็อกอิน/รีบูต จึงคัดลอกใหม่ทุกครั้ง (เหมือน arm_moveit.sh)
docker cp "$HOME/.Xauthority" humanoi_ros:/root/.Xauthority

exec docker exec -it -e DISPLAY=:0 -e XAUTHORITY=/root/.Xauthority humanoi_ros /entrypoint.sh \
  ros2 launch humanoi_vision calibrate_color.launch.py show:=true "$@"
