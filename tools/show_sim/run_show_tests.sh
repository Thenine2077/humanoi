#!/bin/bash
# ทดสอบตัวคุมโชว์ทั้งชุดแบบจำลอง (ไม่ต่อฮาร์ดแวร์) — WSL Ubuntu-24.04 หรือใน container ของ Pi
#   bash tools/show_sim/run_show_tests.sh
# ใช้ ROS_DOMAIN_ID แยก (SHOW_TEST_DOMAIN ค่าเริ่ม 74) ไม่กวนระบบแขนจริงที่เปิดอยู่ · ใช้เวลาราว 6–8 นาที
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
for ws in /humanoi_ws/install /mnt/c/Users/User/Desktop/ar4/ar4_ws/install /mnt/c/Users/User/Desktop/Humanoi/ros2_ws/install; do
  [ -f "$ws/setup.bash" ] && source "$ws/setup.bash"
done
set -u
exec python3 "$HERE/test_show.py"
