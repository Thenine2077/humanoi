#!/usr/bin/env bash
# โหลด ROS กับ workspace ของ Humanoi (ถ้า build แล้ว) ก่อนรันคำสั่งใน container
set -e
source /opt/ros/jazzy/setup.bash
source /opt/astra_ws/install/setup.bash   # ไดรเวอร์กล้อง Astra Pro (build ไว้ใน image)
if [ -f /humanoi_ws/install/setup.bash ]; then
  source /humanoi_ws/install/setup.bash
fi
exec "$@"
