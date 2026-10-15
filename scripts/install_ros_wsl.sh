#!/usr/bin/env bash
# ติดตั้ง ROS 2 Jazzy + Gazebo Harmonic + เครื่องมือที่โปรเจกต์ Humanoi ใช้
# รันใน WSL Ubuntu-24.04 :  bash scripts/install_ros_wsl.sh
# ต้องใส่รหัส sudo — สคริปต์นี้แก้ระบบ WSL (ไม่แตะฝั่ง Windows)
set -euo pipefail

if [[ "$(. /etc/os-release && echo "$VERSION_ID")" != "24.04" ]]; then
  echo "ผิด distro — ตอนนี้อยู่บน $(. /etc/os-release && echo "$PRETTY_NAME")" >&2
  echo "ROS 2 Jazzy ต้องใช้ Ubuntu 24.04 — เปิด distro ให้ถูกก่อน:" >&2
  echo "" >&2
  echo "    wsl -d Ubuntu-24.04" >&2
  echo "    cd /mnt/c/Users/User/Desktop/Humanoi && bash scripts/install_ros_wsl.sh" >&2
  echo "" >&2
  echo "(distro ดีฟอลต์ของ WSL คือ Ubuntu 26.04 ซึ่งต้องใช้ ROS 2 Lyrical" >&2
  echo " ที่แพ็กเกจ AR4/เซนเซอร์ยังไม่ port ตาม roadmap)" >&2
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
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "==> ติดตั้งแพ็กเกจ"
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-ros-base \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-rviz2 \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  python3-colcon-common-extensions \
  python3-rosdep

# navigation2 / slam-toolbox = สำหรับ M5-M8 (SLAM + Nav2)

# ros-jazzy-ros-gz ดึง Gazebo Harmonic (gz-harmonic) มาให้อัตโนมัติ

echo "==> rosdep"
sudo rosdep init 2>/dev/null || true
rosdep update

echo "==> เพิ่ม source ลง ~/.bashrc (ครั้งเดียว)"
grep -q "/opt/ros/jazzy/setup.bash" ~/.bashrc || \
  echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

cat <<'DONE'

เสร็จแล้ว เปิด terminal WSL ใหม่ (หรือ `source ~/.bashrc`) จากนั้น:

  cd /mnt/c/Users/User/Desktop/Humanoi/ros2_ws
  rosdep install --from-paths src --ignore-src -r -y
  colcon build --symlink-install
  source install/setup.bash
  ros2 launch humanoi_description sim.launch.py

หน้าต่างใหม่:
  cd /mnt/c/Users/User/Desktop/Humanoi/ros2_ws && source install/setup.bash
  ros2 run humanoi_teleop keyboard_teleop
DONE
