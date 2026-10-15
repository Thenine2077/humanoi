#!/usr/bin/env bash
# ติดตั้ง ROS 2 Jazzy บน Raspberry Pi (Ubuntu Server 24.04 LTS 64-bit) สำหรับรันบนตัวหุ่น
# ต่างจาก install_ros_wsl.sh: ไม่ลง Gazebo / RViz (ซิมกับจอดูอยู่ฝั่ง PC), เพิ่ม foxglove_bridge ไว้ดูผ่านเว็บ
# รันบน Pi:  bash install_ros_pi.sh      (ต้องใส่รหัส sudo)
set -euo pipefail

. /etc/os-release
if [[ "$VERSION_ID" != "24.04" || "$(uname -m)" != "aarch64" ]]; then
  echo "ต้องเป็น Ubuntu 24.04 แบบ 64-bit (aarch64) — ตอนนี้: $PRETTY_NAME $(uname -m)" >&2
  echo "Raspberry Pi OS ใช้ ROS 2 Jazzy แบบ apt ไม่ได้ ให้ลง Ubuntu Server 24.04 LTS ด้วย Raspberry Pi Imager" >&2
  exit 1
fi

echo "==> locale"
sudo apt-get update
sudo apt-get install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "==> repo ROS 2"
sudo apt-get install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "==> ติดตั้งแพ็กเกจ (ไม่มี Gazebo / RViz)"
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-ros-base \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-foxglove-bridge \
  python3-serial \
  python3-colcon-common-extensions \
  python3-rosdep \
  rsync

echo "==> rosdep"
sudo rosdep init 2>/dev/null || true
rosdep update

echo "==> สิทธิ์ใช้พอร์ต USB serial (Mega, lidar) — มีผลหลัง logout/login"
sudo usermod -aG dialout "$USER"

echo "==> เพิ่ม source ลง ~/.bashrc (ครั้งเดียว)"
grep -q "/opt/ros/jazzy/setup.bash" ~/.bashrc || \
  echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
grep -q "humanoi_ws/install/setup.bash" ~/.bashrc || \
  echo '[ -f ~/humanoi_ws/install/setup.bash ] && source ~/humanoi_ws/install/setup.bash' >> ~/.bashrc

cat <<'DONE'

เสร็จแล้ว ออกจาก SSH แล้วเข้าใหม่ (ให้ dialout มีผล) จากนั้น build workspace:

  cd ~/humanoi_ws
  rosdep install --from-paths src --ignore-src -r -y --skip-keys "ros_gz_sim ros_gz_bridge rviz2"
  colcon build --symlink-install
  source install/setup.bash

ดูหุ่นจาก PC ผ่านเว็บ: บน Pi รัน
  ros2 launch foxglove_bridge foxglove_bridge_launch.xml
แล้วเปิด Foxglove บน PC ต่อ ws://<ip ของ Pi>:8765
DONE
