# Humanoi

หุ่นยนต์ฐานเคลื่อนที่ 4 ล้อ mecanum + แขนกล AR4 ด้านบน — ROS 2 Jazzy
ภาพรวมโครงการ: [`docs/roadmap.html`](docs/roadmap.html) · แผนงาน: [`docs/plan.md`](docs/plan.md)

| ส่วน | ฮาร์ดแวร์ | สถานะ (ต.ค. 2026) |
|---|---|---|
| ฐาน | Arduino Mega 2560 + relay 8 ช่อง + HW-687 ×4 + มอเตอร์ ZKE2032 24V ×4 | ขับด้วยรีโมท FlySky และ ROS ได้ |
| สมอง | Raspberry Pi 5 (8GB) — ROS 2 ใน Docker | ใช้งานจริง |
| ลิดาร์ | LDRobot STL-19P (D500 kit) | ทำแผนที่ (SLAM) + ซิม/Nav2 ได้ |
| กล้อง | Orbbec Astra Pro | ไดรเวอร์อยู่ใน Docker image |
| แขน | AR4 (Teensy 4.1 + Nano) | เล่น pre-program, จัดท่าด้วย MoveIt, สั่งจากรีโมทได้ — โค้ดอยู่ repo แยก |
| เอนโค้ดเดอร์ล้อ | YT3806 ×4 (DC 8–24V, NPN) | รอซื้อ LM2596 + บอร์ดออปโต้ DST-1R4P (12V) |

## โครงสร้าง

```
Humanoi/
├── docker/                 Dockerfile ของ Pi (ROS 2 Jazzy + Nav2 + MoveIt + ไดรเวอร์กล้อง) + entrypoint
├── docker-compose.yml      container humanoi_ros บน Pi (network host, privileged, mount ros2_ws)
├── docs/                   คู่มือ / roadmap / งบไฟ / วงจร (ดูตารางข้างล่าง)
├── firmware/               sketch Arduino Mega ของฐาน
│   ├── humanoi_base_mega/  ← ตัวที่ใช้จริง: ROS + รีโมท FlySky + สวิตช์สั่งแขน (SW:)
│   ├── encoder_test_mega/  ทดสอบเอนโค้ดเดอร์
│   └── rc_* / relay_*      รุ่นก่อนหน้า (เก็บไว้อ้างอิง)
├── ros2_ws/src/
│   ├── humanoi_base/        relay_bridge (/cmd_vel → Mega), rc_arm_trigger (สวิตช์รีโมท → แขน), scan_restamp
│   ├── humanoi_description/ URDF ฐาน (+แขน), โลก Gazebo, launch ซิม/ลิดาร์/กล้อง
│   ├── humanoi_nav/         SLAM (slam_toolbox, rf2o) + Nav2
│   ├── humanoi_teleop/      คีย์บอร์ด teleop + โหนดทดสอบ
│   ├── ldlidar_ros2/        ไดรเวอร์ลิดาร์ (third-party, มี patch — ดู THIRD_PARTY.md)
│   └── rf2o_laser_odometry/ laser odometry (third-party — ดู THIRD_PARTY.md)
└── scripts/
    ├── arm.sh              คำสั่งลัด `arm` คุมแขน AR4 บน Pi
    ├── arm_moveit.sh       เปิด MoveIt + RViz บนจอ Pi
    ├── sync_to_pi.sh       ส่งโค้ดจาก PC ไป Pi (รวมแพ็กเกจแขนจาก ../ar4)
    └── install_ros_*.sh    ติดตั้ง ROS บน WSL / Pi
```

## เอกสาร

| ไฟล์ | เนื้อหา |
|---|---|
| [`docs/AR4_ARM_MANUAL.pdf`](docs/AR4_ARM_MANUAL.pdf) | **คู่มือใช้งานแขน AR4** (PDF) — คำสั่ง `arm`, MoveIt, รีโมท, calibrate, แก้ปัญหา |
| [`docs/AR4_ARM_COMMANDS.md`](docs/AR4_ARM_COMMANDS.md) | คู่มือเดียวกันแบบ Markdown |
| [`docs/PI_DOCKER.md`](docs/PI_DOCKER.md) | ROS 2 บน Raspberry Pi (Docker) — ฐาน, SLAM, แขน, ข้อควรรู้ |
| [`docs/power_budget.xlsx`](docs/power_budget.xlsx) | งบไฟทั้งหุ่น (มี [PDF](docs/power_budget.pdf)) |
| [`docs/wiring_relay_motor_4wd.md`](docs/wiring_relay_motor_4wd.md) | วงจร relay 4 ล้อ |
| [`docs/RUN.md`](docs/RUN.md) | วิธีรันซิมบน PC |

## ใช้งานบน Raspberry Pi

```bash
cd ~/Humanoi && docker compose up -d          # เปิด container (เปิดเองหลังรีบูต)
docker exec -it humanoi_ros bash              # เข้าไปใช้ ros2
arm help                                      # คำสั่งลัดแขน AR4 (รันนอก container)
```

ส่งโค้ดจาก PC ไป Pi: `bash scripts/sync_to_pi.sh` แล้ว build ใน container — รายละเอียดใน [`docs/PI_DOCKER.md`](docs/PI_DOCKER.md)

## ซิมบน PC (WSL Ubuntu-24.04)

```bash
cd /mnt/c/Users/User/Desktop/Humanoi/ros2_ws
colcon build --symlink-install && source install/setup.bash
ros2 launch humanoi_description sim.launch.py     # Gazebo + ฐาน
ros2 run humanoi_teleop keyboard_teleop           # อีกหน้าต่าง: w/s/a/d/q/e
```

ติดตั้งครั้งแรก: `bash scripts/install_ros_wsl.sh` · รายละเอียดเพิ่มใน [`docs/RUN.md`](docs/RUN.md)

## แขน AR4 (repo แยก)

โค้ดแขนอยู่ใน repo **humanoi-ar4-arm** — ต้อง clone ไว้ข้างกันในชื่อโฟลเดอร์ `ar4` เพื่อให้ `sync_to_pi.sh` หาเจอ:

```bash
git clone https://github.com/Thenine2077/humanoi-ar4-arm.git ar4   # วางข้าง Humanoi/
```

## หมายเหตุ

- workspace อยู่บน `/mnt/c` เพื่อแก้ไฟล์จาก Windows ได้ (build ช้ากว่าบน filesystem ของ WSL)
- ขนาดฐานใน `mecanum_base.urdf.xacro` ยังเป็นค่าสมมติ รอวัดของจริง
- ไฟล์ท่าแขน (`programs.yaml`) ที่ใช้จริงอยู่บน Pi — ก่อนรัน `sync_to_pi.sh` ให้ดึงกลับมา PC ก่อน
