# ROS 2 Jazzy สำหรับรันบน Raspberry Pi 5 (Raspberry Pi OS + Docker)
# ไม่มี Gazebo (ซิมอยู่ฝั่ง PC) มี RViz + MoveIt ไว้จัดท่าแขนบนจอ Pi มี foxglove_bridge ไว้ดูผ่านเว็บ
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-joint-state-publisher \
      ros-jazzy-xacro \
      ros-jazzy-teleop-twist-keyboard \
      ros-jazzy-navigation2 \
      ros-jazzy-nav2-bringup \
      ros-jazzy-slam-toolbox \
      ros-jazzy-foxglove-bridge \
      python3-serial \
    && rm -rf /var/lib/apt/lists/*

# กล้อง Orbbec Astra Pro (ros2_astra_camera): dependency ของไดรเวอร์ + libuvc (build เอง)
# libuvc ต้องเจอ libjpeg ตอน build ไม่งั้นภาพสีแบบ MJPEG จะแปลงไม่ได้
RUN apt-get update && apt-get install -y --no-install-recommends \
      git cmake build-essential pkg-config \
      libusb-1.0-0-dev libjpeg-dev libgflags-dev libgoogle-glog-dev libeigen3-dev \
      ros-jazzy-camera-info-manager \
      ros-jazzy-cv-bridge \
      ros-jazzy-image-geometry \
      ros-jazzy-image-publisher \
      ros-jazzy-image-transport \
      ros-jazzy-compressed-image-transport \
      ros-jazzy-tf2-eigen \
      ros-jazzy-tf2-sensor-msgs \
    && git clone --depth 1 --branch v0.0.7 https://github.com/libuvc/libuvc.git /tmp/libuvc \
    && cmake -S /tmp/libuvc -B /tmp/libuvc/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/libuvc/build -j4 \
    && cmake --install /tmp/libuvc/build \
    && ldconfig \
    && rm -rf /tmp/libuvc /var/lib/apt/lists/*

# ไดรเวอร์ Astra Pro build ไว้ใน /opt/astra_ws (ไม่อยู่ใน ros2_ws เพราะ PC/WSL ไม่มี libuvc)
# ล็อก commit ไว้ + astra_jazzy.patch แก้ header/API ที่ Jazzy เปลี่ยน (ต้นฉบับทำมาสำหรับ Humble)
# เปิดกล้อง: ros2 launch astra_camera astra_pro.launch.xml
ARG ASTRA_COMMIT=f7e71d9ce806e788cb48d8580aac2c778fba4214
COPY docker/astra_jazzy.patch /tmp/astra_jazzy.patch
RUN mkdir -p /opt/astra_ws/src/ros2_astra_camera \
    && cd /opt/astra_ws/src/ros2_astra_camera \
    && git init -q \
    && git fetch -q --depth 1 https://github.com/orbbec/ros2_astra_camera.git ${ASTRA_COMMIT} \
    && git checkout -q FETCH_HEAD \
    && git apply /tmp/astra_jazzy.patch \
    && rm -rf .git /tmp/astra_jazzy.patch \
    && . /opt/ros/jazzy/setup.sh \
    && cd /opt/astra_ws \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf build log

# แขน AR4 (แพ็กเกจ ar_description / ar_hardware_interface / ar_program จาก ar4_ws บน PC)
# ros2_control + controllers: controller_manager, joint_trajectory_controller, joint_state_broadcaster
# libboost-system-dev: ar_hardware_interface ใช้ boost::asio คุย serial กับ Teensy
# วางเป็นชั้นสุดท้ายโดยตั้งใจ — ชั้นก่อนหน้า (รวม build ไดรเวอร์กล้อง) ใช้ cache เดิมได้ทั้งหมด
# MoveIt + RViz: จัดท่าแขนจริงบนจอ Pi ผ่าน Raspberry Pi Connect (scripts/arm_moveit.sh) — เพิ่มแค่ ~73 MB
# teensy-loader-cli: แฟลชเฟิร์มแวร์ Teensy ของแขนจาก Pi ได้โดยไม่ต้องย้ายสาย · avrdude: แฟลช Mega ของฐานจาก Pi
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      libboost-system-dev \
      ros-jazzy-moveit-ros-move-group \
      ros-jazzy-moveit-ros-visualization \
      ros-jazzy-moveit-planners-ompl \
      ros-jazzy-moveit-kinematics \
      ros-jazzy-moveit-simple-controller-manager \
      ros-jazzy-rviz2 \
      teensy-loader-cli \
      avrdude \
    && rm -rf /var/lib/apt/lists/*

# เกมเป่ายิ้งฉุบ (ros2_ws/src/humanoi_games): rustypot = คุมมือ AmazingHand, mediapipe = ดูมือผู้เล่น
# mediapipe ติดแบบ --no-deps: dependency ของมันลาก numpy 2 + opencv-contrib มาทับ numpy 1.26 / cv2 ของ apt
# ที่ ROS ใช้อยู่ (ของที่ build กับ numpy 1 จะพัง) — ตอน import ต้องการ absl-py, flatbuffers, certifi และ
# matplotlib (เอาจาก apt ซึ่ง build คู่กับ numpy 1.26 ของระบบ)
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip python3-matplotlib \
    && pip3 install --break-system-packages --no-cache-dir rustypot==1.13.1 \
    && pip3 install --break-system-packages --no-cache-dir --no-deps \
         mediapipe==1.1.1 "absl-py~=2.3" "flatbuffers~=25.9" certifi \
    && rm -rf /var/lib/apt/lists/*

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh \
    && echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
    && echo 'source /opt/astra_ws/install/setup.bash' >> /root/.bashrc \
    && echo '[ -f /humanoi_ws/install/setup.bash ] && source /humanoi_ws/install/setup.bash' >> /root/.bashrc

WORKDIR /humanoi_ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
