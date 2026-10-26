# ROS 2 บน Raspberry Pi 5 (Raspberry Pi OS + Docker)

Pi 5 (8GB) ยังใช้ Raspberry Pi OS (Debian 13 trixie) ตัวเดิม เพราะงาน Hailo / YOLO อยู่บนนั้น
ROS 2 Jazzy เลยรันใน Docker container ชื่อ `humanoi_ros` (ข้างในเป็น Ubuntu 24.04 แบบเดียวกับ WSL บน PC)

- Pi: `eitlaser@EIT-Raspberry-Pi-5.local` (เรียกด้วยชื่อเครื่อง — IP เปลี่ยนได้ เช่นหลังเปลี่ยนบอร์ด · เข้าด้วย SSH key จาก PC)
- โค้ดบน Pi: `~/Humanoi` (ส่งจาก PC ด้วย `scripts/sync_to_pi.sh`)
- container ใช้ network ของเครื่องจริง (`network_mode: host`) และเห็นพอร์ต USB ทุกตัว (`privileged` + `/dev`)
- workspace `~/Humanoi/ros2_ws` ถูก mount เป็น `/humanoi_ws` ใน container: build ครั้งเดียวแล้วอยู่บน Pi ถาวร

## ใช้งานประจำ (บน Pi)

```bash
cd ~/Humanoi
docker compose up -d            # เปิด container (ตั้งให้เปิดเองหลังรีบูตแล้ว)
docker exec -it humanoi_ros bash  # เข้าไปใช้ ros2 ใน container
docker compose down             # ปิด
```

ใน container:

```bash
cd /humanoi_ws
colcon build --symlink-install   # build หลังแก้โค้ด
source install/setup.bash
ros2 topic list
```

## แก้โค้ดบน PC แล้วส่งไป Pi

```bash
bash scripts/sync_to_pi.sh                       # บน PC (Git Bash / WSL)
ssh eitlaser@EIT-Raspberry-Pi-5.local 'docker exec humanoi_ros /entrypoint.sh bash -c "cd /humanoi_ws && colcon build --symlink-install"'
```

## ขับฐานจริงจาก ROS (Mega เสียบที่ Pi)

Mega ต้องลง `firmware/humanoi_base_mega` (ROS + รีโมทแทรกได้ตลอด) — รีโมทต้องเปิดและ ARM (SWA ขึ้นแล้วลง) ล้อถึงจะฟัง ROS

```bash
# ใน container — เปิดพอร์ตด้วยชื่อ by-id จะไม่สลับกับบอร์ดมือ (CH343 ก็ขึ้นเป็น ttyACM)
ros2 run humanoi_base relay_bridge --ros-args -p port:=/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_858303033393517150C0-if00
# อีกหน้าต่างหนึ่งใน container
ros2 run humanoi_teleop keyboard_teleop
ros2 topic echo /motor_state --field data   # ดู STATE:/MODE: ที่ Mega ตอบ
```

ทดสอบแล้ว 2026-10-12 (ไฟมอเตอร์ถอด ฟังเสียง relay): w/s/a/d/q/e ได้ `FFFF` `BBBB` `BFFB` `FBBF` `BFBF` `FBFB` ตรงสูตร ·
รีโมทปิด/SWA ขึ้น = ROS สั่งไม่ได้ · ขยับคันโยก = รีโมทแย่งคุม ปล่อย 0.7 วิ คืนให้ ROS

## ทำแผนที่ด้วยลิดาร์จริง (ยังไม่มี encoder)

```bash
# ใน container — ลิดาร์ + rf2o (laser odometry) + slam_toolbox ในคำสั่งเดียว
ros2 launch humanoi_nav real_slam.launch.py
# เซฟแผนที่
ros2 run nav2_map_server map_saver_cli -f /humanoi_ws/src/humanoi_nav/maps/real_room
```

- rf2o ไม่มี apt สำหรับ Jazzy — clone `MAPIRlab/rf2o_laser_odometry` (branch ros2) ไว้ใน `ros2_ws/src` แล้ว build ผ่าน (กิน CPU ~18% ของคอร์เดียว)
- Foxglove: Display frame = `map`, เปิด `/map` + `/scan`
- ลิดาร์ STL-19P บน Pi = `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`

## แขน AR4: เล่นท่าที่ตั้งไว้ล่วงหน้า (Teensy เสียบที่ Pi)

> ใช้งานประจำให้ใช้คำสั่งลัด `arm` (เช่น `arm start`, `arm run demo`) — คู่มือ: [AR4_ARM_COMMANDS.md](AR4_ARM_COMMANDS.md)

แพ็กเกจ `ar_description` `ar_hardware_interface` `ar_program` ต้นฉบับอยู่บน PC ที่ `Desktop\ar4\ar4_ws\src`
`scripts/sync_to_pi.sh` ส่งมารวมใน `ros2_ws/src` ให้เอง (รวม ar_moveit_config + ar_gz_bringup สำหรับจัดท่าด้วย MoveIt บนจอ Pi — ไม่ส่ง ar_mirror)
image มี ros2_control + ros2_controllers แล้ว (เพิ่ม 2026-10-14)

ก่อนใช้: Teensy ต้องลงเฟิร์มแวร์ ROS (`C:\ar4_fw\AR4_ROS2\AR4_ROS2.ino`) — เฟิร์มแวร์ HMI ใช้กับ ROS ไม่ได้
เสียบ USB ตรงเข้า Pi ได้เลย ไม่ต้องใช้ usbipd / สะพาน TCP แบบตอนรันบน WSL

```bash
# ใน container — หา path ของ Teensy ก่อน (ขึ้นต้นด้วย usb-Teensyduino)
ls /dev/serial/by-id/
# หน้าต่าง 1: คุมแขน (calibrate:=True ถ้าเพิ่งเปิดไฟแขน)
ros2 launch ar_hardware_interface ar_hardware.launch.py serial_port:=/dev/serial/by-id/<Teensy> calibrate:=True
# หน้าต่าง 2: ตัวเล่นโปรแกรม
ros2 launch ar_program arm_program.launch.py
# สั่งงาน / ดูผล / หยุด
ros2 topic pub --once /arm/run std_msgs/msg/String "{data: demo}"
ros2 topic echo /arm/done
ros2 topic pub --once /arm/stop std_msgs/msg/Empty "{}"
```

- ท่าและโปรแกรมอยู่ใน `ros2_ws/src/ar_program/config/programs.yaml` — ตั้งท่าในซิมบน PC แล้ว sync มา
  (sync สำรองไฟล์เดิมบน Pi เป็น `programs.yaml.bak` ก่อนเขียนทับทุกครั้ง)
- ไม่มีการเช็คการชน (เหมือน Move J ของ HMI) แต่ตรวจลิมิตข้อต่อทุกท่าก่อนขยับ
- ทดสอบบน Pi แล้ว 2026-10-14 โดยยังไม่มีแขน: build ผ่าน, teach หาไฟล์ท่าเจอ, plugin ลงทะเบียน,
  URDF สร้างได้, arm_program ตอบ error ชัดเจนเมื่อยังไม่มี controller
- **ทดสอบกับแขนจริงบน Pi แล้ว 2026-10-14:** `calibrate:=False` (แขน calibrate ไว้แล้ว) + `speed:=0.2` → `demo:ok`
  แขนกลับท่าพักตรงตำแหน่ง Teensy อยู่ที่ `/dev/serial/by-id/usb-Teensyduino_USB_Serial_13173040-if00`

### จัดท่าด้วย MoveIt บนจอ Pi (Raspberry Pi Connect)

RViz ใน container วาดลงจอของ Pi ได้ (network_mode: host + คุกกี้ `~/.Xauthority` ที่สคริปต์คัดลอกให้)
เปิดระบบแขนไว้ก่อน แล้วรันในเทอร์มินัลบนเดสก์ท็อป Pi (นอก container):

```bash
bash ~/Humanoi/scripts/arm_moveit.sh
```

ใน RViz: แท็บ Joints เลื่อนมุม (หรือลากลูกบอลปลายแขน) → Planning → **Plan & Execute = แขนจริงขยับ**
แล้ว `ros2 run ar_program teach <ชื่อท่า>` ใน container — ท่าเก็บใน programs.yaml บน Pi
(ดึงกลับ PC ก่อนรัน sync_to_pi.sh ไม่งั้นถูกทับ) · ทำงานได้ ~31 fps บน Pi 5

### calibrate กับ launch ของผู้ขาย — ต้องรู้

- calibrate ทุกแกนพร้อมกันใช้เวลา **~2 นาที** และ log เงียบตลอด (`Failed to acquire lock` / `Could not contact
  service` ระหว่างนั้นเป็นเรื่องปกติ) — spawner ของ launch ผู้ขายรอแค่ 60 วินาทีแล้วตาย ต้องสั่งเองหลังแขนกลับท่าพัก:
  `ros2 run controller_manager spawner joint_state_broadcaster` และ `... spawner joint_trajectory_controller`
- **ห้าม Ctrl+C ระหว่าง calibrate** — Teensy ทำต่อจนจบแล้วส่งผล `JC…` กลับมา ถ้าตอนนั้นเปิด launch ใหม่อยู่
  ไดรเวอร์ของผู้ขายจะ Segmentation fault (รับ JC ระหว่างรอ ST) ให้รอแขนหยุดที่ท่าพักก่อนค่อยเปิดใหม่
- J1 หมุนไปถึง −170° ตอน calibrate — เดินสาย USB ให้หย่อนและพ้นรัศมีแขน (เคยหลุดทั้ง 2 เส้นพร้อมกันกลางการ calibrate)
- Teensy จำตำแหน่งไว้ตราบที่ยังมีไฟ — รอบถัดไปใช้ `calibrate:=False` ได้ ถ้าปิดไฟแขน/ถอด USB ต้อง calibrate ใหม่
- แฟลชเฟิร์มแวร์ Teensy จาก Pi ได้ (ไม่ต้องย้ายสาย): `teensy_loader_cli --mcu=TEENSY41 -w -v ไฟล์.hex` ใน container
  (ลงด้วย apt ไว้ใน container — หายถ้าสร้าง container ใหม่) ถ้าขึ้น `error writing to Teensy` ให้สั่งซ้ำอีกครั้ง

## กล้อง Astra Pro + YOLO บนการ์ด Hailo-10H (ตรวจจับวัตถุ)

```bash
# ใน container: กล้อง + yolo_node + Foxglove (ตัวคิดบนการ์ด Hailo เปิดเองตอนบูตแล้ว)
ros2 launch humanoi_vision yolo.launch.py
ros2 launch humanoi_vision yolo.launch.py classes:=bottle,cup conf:=0.5   # เฉพาะบางคลาส
```

มีหน้าต่างภาพบนจอ Pi (กรอบ + รายการ X Y Z + ระยะเส้นตรง `dist` ที่เทียบกับตลับเมตรได้): บน Pi นอก container
`bash ~/Humanoi/scripts/yolo_view.sh` (ออก: Ctrl+C) — ช้าลงเหลือ ~17 fps จากการวาดภาพ

`depth_offset` (ค่าเริ่ม 0.03 m) บวกเข้ากับระยะ depth: Astra Pro อ่านสั้นกว่าระยะจากเลนส์สีราว 3 cm คงที่
(ทดสอบ 15 ต.ค.: ขวด 3 จุดตรงตลับเมตรภายใน 1 cm, เทป 2 แถบห่าง 20 cm วัดได้ 19.7) — ปิดได้ด้วย `depth_offset:=0.0`
พิกัดใน `/yolo/detections_3d` อยู่ในเฟรม `camera_color_optical_frame` (X ขวา, Y ลง, Z ไปข้างหน้า)

ของที่ใกล้กว่า ~60 cm (พื้นที่ที่แขนเอื้อมถึง) depth วัดไม่ได้ → `table_mode:=auto` (ค่าเริ่ม) หาผิวโต๊ะจาก depth ส่วนที่ไกล
ทุก 2 วินาที แล้วคิดตำแหน่งของที่ตั้งบนโต๊ะจากกรอบ YOLO (มองเป็นทรงกระบอก ได้ใจกลาง + กว้าง + สูง) ขึ้น `[table]`
ในรายการ · `table_mode:=always` ใช้วิธีนี้กับทุกชิ้น (ไว้เทียบกับ depth) · ใช้ได้กับของที่ตั้งบนโต๊ะเท่านั้น

ตัวคิดบนการ์ด Hailo (`scripts/hailo_yolo_server.py`, yolov11m ~27 ms/ภาพ) เป็น systemd service บนเครื่อง Pi (นอก container)
ถือการ์ดเฉพาะตอน `yolo_node` ต่ออยู่ — HailoRT ให้เปิดการ์ดได้ทีละโปรแกรม ตอนไม่ได้รัน yolo_node งาน Hailo อื่น
(hailo-apps ฯลฯ) ใช้การ์ดได้ปกติ ถ้าโปรแกรมอื่นถือการ์ดอยู่ yolo_node จะเตือน "ใช้ Hailo ไม่ได้" แล้วต่อเองเมื่อการ์ดว่าง

```bash
systemctl status hailo-yolo-server          # สถานะ
journalctl -u hailo-yolo-server -f          # log (ภาพ/วินาที, client ต่อ/ออก)
sudo systemctl restart hailo-yolo-server    # หลังแก้สคริปต์ / เปลี่ยนโมเดล
sudo systemctl disable --now hailo-yolo-server   # เลิกเปิดเองตอนบูต
# ติดตั้งครั้งแรก / หลังแก้ไฟล์ .service
sudo cp ~/Humanoi/scripts/hailo-yolo-server.service /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl enable --now hailo-yolo-server
```

ดูใน Foxglove: Image panel ที่ `/yolo/image/compressed` · กรอบที่เจอ (พิกเซล) อยู่ที่ `/yolo/detections`
HailoRT ต้องตรงกับเฟิร์มแวร์การ์ด (5.1.1 ลงบน Raspberry Pi OS แล้ว) container จึงส่งภาพไปให้ server ผ่าน
`127.0.0.1:5555` แทนการเรียกการ์ดเอง · กล้องต้องเสียบที่ Pi และห้ามเปิด guvcview พร้อมกัน

### calibrate ค่าเลนส์กล้องสี (ทำครั้งเดียว — กล้องตัวนี้ไม่มีค่าจากโรงงาน)

พิมพ์ `docs/calibration/checkerboard_9x6_25mm.pdf` บน A4 แบบ **100%** ติดแผ่นแข็งที่เรียบ แล้วใน container:

```bash
ros2 launch humanoi_vision calibrate_color.launch.py
```

ถือกระดาษห่างกล้อง ~0.4-1.2 m ค่อยๆ ขยับไปทั่วภาพ ทั้งใกล้/ไกล และเอียงหลายทิศ หยุดนิ่งทุกท่า — ดูความคืบหน้าใน
Foxglove ที่ `/calibration/image/compressed` (เก็บภาพเอง 30-40 ภาพแล้วคำนวณ) ได้ไฟล์
`ros2_ws/src/humanoi_description/config/astra_color.yaml` แล้วรัน
`colcon build --symlink-install --packages-select humanoi_description` หนึ่งครั้ง — `camera.launch.py` โหลดไฟล์นี้เอง
(ก่อนมีไฟล์ log จะขึ้น ERROR "Unable to open camera calibration file" — ปกติ ไดรเวอร์ใช้ค่ากลางแทน)
ไฟล์ yaml สร้างบน Pi — คัดลอกกลับมา PC เพื่อเก็บเข้า git (sync_to_pi.sh ไม่ลบไฟล์นี้บน Pi)

- **วัดขนาดช่องที่พิมพ์ออกมาจริง** แล้วใส่ `square:=` ตามนั้น เพราะเครื่องพิมพ์มักย่อ (ตาราง 30 mm พิมพ์ออกมา 28.2 mm)
  ขนาดช่องไม่กระทบค่าเลนส์ แต่ตัวตรวจกับ depth ใช้มันตรงๆ — ผิด 6% ก็ขึ้น `REJECTED`
- มีหน้าต่างภาพบนจอ Pi: `bash ~/Humanoi/scripts/calibrate_camera.sh square:=0.0282 cols:=8 rows:=5`
- ค่าเลนส์ยังค้านกับ depth (เกิน 4%) ทั้งที่ขนาดช่องถูก → ตัดสินด้วยตลับเมตรว่า depth หรือค่าเลนส์ที่คลาด:
  `bash ~/Humanoi/scripts/check_depth_scale.sh` — ตั้งกระดาษ (แผ่นแข็ง ตั้งตรง) ที่ ~60 cm แล้วถอยตรงๆ ไป ~100 cm
  พิมพ์ระยะที่วัดได้ทั้งสองครั้ง จะบอกว่า depth คลาดกี่ % และ fx ที่ถูกควรเป็นเท่าไร (ใช้ผลต่าง 2 ระยะ จึงไม่ต้องรู้
  ว่าจุดศูนย์กล้องอยู่ลึกจากหน้ากล้องเท่าไร)
- ผลที่ใช้อยู่ (15 ต.ค.): fx 632 fy 633 cx 322 cy 230 — ค่ากลาง 570 ของไดรเวอร์คลาด ~10%
  depth ของ Astra Pro อ่านระยะ **สั้นกว่า** ระยะจากจุดศูนย์เลนส์สี ~3 cm คงที่ (สเกลตรงกัน) ตัว calibrate จึงตรวจ
  ด้วยความชันข้ามหลายระยะ ไม่ใช่อัตราส่วน — ระยะเหลื่อมนี้ hand-eye calibration จะดูดซับไปเอง

### hand-eye calibration: กล้องอยู่ตรงไหนเทียบกับฐานแขนซ้าย (ทำหลังติดกล้องที่เสา และทุกครั้งที่กล้องขยับ)

แปะป้าย ArUco 4x4_50 id 0 ขนาด 40 mm (chev.me/arucogen พิมพ์ 100% มีขอบขาว ติดแผ่นแข็ง) บนมือจับ
เปิด `arm start` + `arm moveit` แล้วรันบนเดสก์ท็อป Pi: `bash ~/Humanoi/scripts/hand_eye_calibrate.sh`
พาแขนไป 15 ท่าด้วย MoveIt (ระยะป้ายคิดจากขนาดป้ายในภาพ — ใช้ได้ทุกระยะ แขนซ้ายเอื้อมห่างกล้องได้แค่ ~52 cm
ซึ่งใกล้เกินกว่าที่ depth วัดได้; `measure:=depth` ใช้ depth แทนเมื่อป้ายห่าง > 0.6 m) กระจายซ้าย/ขวา/ใกล้/ไกล/สูง/ต่ำ
และหมุนข้อมือหลายมุม หยุดค้างท่าละ ~2 วินาทีจนขึ้น CAPTURED → ได้ `humanoi_description/config/camera_to_left_arm.yaml`
→ `colcon build --symlink-install --packages-select humanoi_description` → `yolo.launch.py` ส่ง TF
`base_link -> camera_link` ให้เอง (ไม่มีไฟล์ = ข้าม) · ไม่ใช้ผลถ้าคลาดเฉลี่ยเกิน 10 mm หรือข้อมือหมุนต่างกันไม่ถึง 30°

## ดูหุ่นจาก PC

ใน container รัน `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`
แล้วเปิด Foxglove บน PC ต่อ `ws://EIT-Raspberry-Pi-5.local:8765`

## หมายเหตุ

- `Dockerfile` ไม่ลง Gazebo (หนัก และ Pi ไม่ต้องใช้) — RViz + MoveIt ลงไว้สำหรับจัดท่าแขนบนจอ Pi — ไม่ต้องรัน `rosdep install` เพราะ `humanoi_description` อ้าง ros_gz / rviz2 ไว้สำหรับซิม
- Hailo รันบนเครื่องจริงนอก container เสมอ (`scripts/hailo_yolo_server.py`) — `humanoi_vision yolo_node` ใน container คุยกับมันผ่าน socket
- อุปกรณ์ USB: Mega ขึ้นเป็น `/dev/ttyACM*`, lidar มักเป็น `/dev/ttyUSB*`
- สั่งจากข้างนอกแบบไม่ interactive (`docker exec humanoi_ros ...`) ให้นำหน้าด้วย `/entrypoint.sh` เสมอ ไม่งั้นจะหา `ros2` ไม่เจอ (`bash -lc` ไม่โหลด ROS ให้)
- การ์ด 29G เหลือที่ว่างไม่มาก (อิมเมจ ROS กิน 3.6G) — ถ้าเปิด guvcview กับกล้อง Orbbec แล้วกล้องหลุด มันจะเขียน error ลง `~/.xsession-errors` จนการ์ดเต็ม (เคยโตถึง 6G) ให้ปิดโปรแกรมแล้วล้างด้วย `truncate -s 0 ~/.xsession-errors`
