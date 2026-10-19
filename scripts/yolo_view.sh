#!/usr/bin/env bash
# กล้อง + YOLO (Hailo) พร้อมหน้าต่างภาพบนจอ Pi: กรอบ + รายการที่เจอ + พิกัด X Y Z (เมตร ในเฟรมกล้อง)
# รันบน Pi นอก container:
#   bash ~/Humanoi/scripts/yolo_view.sh                       # ทุกคลาส ความมั่นใจ >= 0.3
#   bash ~/Humanoi/scripts/yolo_view.sh classes:=cup,bottle conf:=0.5
# ออก: Ctrl+C ในหน้าต่าง terminal นี้ · ยังดูใน Foxglove ได้เหมือนเดิม (/yolo/image/compressed)
set -euo pipefail

# หน้าต่างใน container วาดลงจอ :0 ของ Pi — คัดลอกคุกกี้ X11 ของ session ปัจจุบันทุกครั้ง (เหมือน calibrate_camera.sh)
docker cp "$HOME/.Xauthority" humanoi_ros:/root/.Xauthority

exec docker exec -it -e DISPLAY=:0 -e XAUTHORITY=/root/.Xauthority humanoi_ros /entrypoint.sh \
  ros2 launch humanoi_vision yolo.launch.py show:=true conf:=0.3 "$@"
