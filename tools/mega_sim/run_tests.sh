#!/usr/bin/env bash
# ทดสอบกันชน lidar ก่อนแฟลช Mega จริง: build เฟิร์มแวร์ตัวจริงเป็นโปรแกรม Linux แล้วรันกับ relay_bridge ตัวจริง
# ต้องมี: g++ และ ROS 2 Jazzy (WSL Ubuntu-24.04 บน PC หรือใน container humanoi_ros บน Pi)
#   bash tools/mega_sim/run_tests.sh
set -euo pipefail
cd "$(dirname "$0")/../.."
if [ -z "${ROS_DISTRO:-}" ]; then set +u; source /opt/ros/jazzy/setup.bash; set -u; fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"  # แยกจาก ROS ของหุ่นจริงที่อาจเปิดอยู่
out=$(mktemp -d)
g++ -std=c++17 -O1 -Wall -Wextra -Werror tools/mega_sim/guard_test.cpp -o "$out/guard_test"
g++ -std=c++17 -O1 -Wall -Wextra -Werror -I tools/mega_sim tools/mega_sim/mega_sim.cpp -o "$out/mega_sim"
"$out/guard_test"
python3 tools/mega_sim/test_guard_e2e.py "$out/mega_sim" "$out/guard_test"
