# Humanoi — แผนงานสาย ROS / simulation

> โปรเจกต์เต็มดูใน [`roadmap.html`](roadmap.html) — เอกสารนี้คือสายซอฟต์แวร์ที่ทำคู่ขนาน
> ระหว่างรอฮาร์ดแวร์ Stage 0–1 (MDDS10, FlySky, แบต ฯลฯ) มาถึง

## เป้าหมายของเฟสนี้

พิสูจน์ว่า pipeline การควบคุมทำงานถูกต้อง **ก่อน** ฮาร์ดแวร์มา และหาว่ายังขาดอะไร:

```
คีย์บอร์ด ──> keyboard_teleop ──/cmd_vel(Twist)──> [ ปลายทาง ]
                                                       ├─ Gazebo (MecanumDrive)   ← ทำก่อน
                                                       └─ serial bridge ──> ESP32  ← ทำต่อ
```

`/cmd_vel` เป็นอินเทอร์เฟซกลาง — สลับปลายทางได้โดยไม่แตะตัว teleop
ภายหลังแค่เปลี่ยน "คีย์บอร์ด" เป็น "iBUS จาก FlySky" ก็ได้ path เดียวกับของจริง

## Milestone

| # | สิ่งที่ต้องได้ | เกณฑ์ผ่าน | สถานะ |
|---|---|---|---|
| M1 | หุ่น mecanum ใน Gazebo ขยับตามคีย์บอร์ด | กด w/a/s/d/q/e แล้วหุ่นเดินหน้า/สไลด์/หมุน ถูกทิศทุกปุ่ม · ปล่อยปุ่ม 0.5 วิ = หยุด | ✅ ผ่าน (2026-09-07) |
| M2 | ตรวจ odometry เทียบคำสั่ง | สั่งวิ่งสี่เหลี่ยม 2×2 m กลับจุดเดิม /odom คลาดเคลื่อน < 5% | ✅ ผ่าน (2026-09-07, VelocityControl body-frame, odom จากตำแหน่งจริง) |
| M3 | serial bridge ↔ Arduino MEGA (relay H-bridge, 1 มอเตอร์) | กด w/s → มอเตอร์เดินหน้า/ถอย · ปล่อยปุ่ม 0.5 วิ = หยุด · ถอดสาย USB = watchdog หยุด · กลับทางมี dead-time 0.5 วิ | ✅ ผ่าน (2026-09-07) |
| M3b | 4 มอเตอร์ mecanum (relay 8 ตัว, ทิศ 8 แบบ) | กด w/s/a/d/q/e → 4 ล้อหมุนตามตาราง mixing · `/wheel_target` ตรงสูตร · watchdog หยุดทั้ง 4 | ✅ ผ่าน (2026-09-07, relay 8 ตัวตอบครบทุกทิศ) |
| M4 | รัน Gazebo + Arduino พร้อมกันจากคีย์บอร์ดเดียว | สั่ง w → หุ่นใน Gazebo เดินหน้า และมอเตอร์จริงเดินหน้า สอดคล้องกัน | ✅ ผ่าน (2026-09-07) |
| M5 | LiDAR จำลองในหุ่น | เปิด `world:=room rviz:=true` เห็น `/scan` วาดเส้นรอบห้อง + กล่อง หมุนตามหุ่น | ✅ ผ่าน (2026-09-07) |
| M6 | SLAM สร้างแผนที่ | `slam_toolbox` + ขับคีย์บอร์ดรอบห้อง → ได้ occupancy map → `map_saver_cli` | ✅ ผ่าน (2026-09-07, `maps/room.pgm`) |
| M7 | Nav2 ไป goal | โหลด map + AMCL → ตั้ง goal → หุ่นวาง global path + ขับไปถึง | ✅ ผ่าน (2026-09-07 ผ่าน CLI action; RViz goal tool = nav2_rviz_plugins/GoalTool + Navigation 2 panel) |
| M8 | หลบสิ่งกีดขวาง | วางกล่องใหม่ขวางทางที่ไม่มีในแผนที่ → local costmap จาก /scan → หุ่นอ้อม | 🔶 พร้อมทดสอบ |
| **M9** | Nav2 holonomic | เปลี่ยน controller เป็น MPPI เปิด `vy` → หุ่นสไลด์เข้า goal / ผ่านช่องแคบ โดยไม่ต้องหมุนก่อน | ⬜ |
| **M10** | `humanoi_mission` orchestrator | โหนดเรียก Nav2 actions — ลาดตระเวน waypoint วนลูป · ข้าม waypoint ถ้าติดเกิน N วิ · จบงานกลับจุดเริ่ม | ⬜ |
| **M11** | odometry หุ่นจริง | `rf2o` laser odometry → `/odom` + TF `odom→base_footprint` บนฐานจริง (ยังไม่ต้องมี encoder) | ⬜ |
| **M12** | motor driver จริง | เปลี่ยน relay → ESP32 + Cytron MDDS10 ×2 · `/cmd_vel` → 4 wheel PWM (คุมความเร็วได้) · mixing ย้ายไปเฟิร์มแวร์ | ⬜ |
| **M13** | Nav2 บนฐานจริง | LiDAR จริง (LD19/RPLIDAR) + ESP32 + laser odom → SLAM + Nav2 ครบ → หุ่นจริงขับไป goal ในห้องจริง หลบสิ่งกีดขวาง | ⬜ |
| **M14** | odometry แม่นขึ้น | dead-wheel pod ×2–3 + IMU → `robot_localization` EKF fuse → odom เชื่อถือได้ตอนวิ่งเร็ว/สไลด์ (roadmap Stage 2) | ⬜ |
| **M15** | แขน AR4 ในซิม | เพิ่ม URDF AR4 ติดบน `base_link` · MoveIt 2 · เล่นท่า pre-program · TF ต่อเนื่อง ล้อ→ปลายแขน · RViz เห็นทั้งฐาน+แขน | ⬜ |
| **M16** | ฐาน + แขน ในซิมเดียว | ขับฐานไป station → หยุดนิ่ง → แขนเล่นท่าหยิบ/วาง → ขับต่อ · **interlock**: แขนต้องพับ (stow) ก่อนฐานเคลื่อน, ฐานต้องนิ่งก่อนแขนทำงาน | ⬜ |
| **M17** | แขน AR4 จริง | Teensy 4.1 + `ar4_ros_driver` + MoveIt คุมแขนจริง (ฐานจอดนิ่ง) · gripper (Arduino Nano) | ⬜ |
| **M18** | Humanoi ทำงานร่วมกัน | Raspberry Pi 5 เป็นสมอง · `humanoi_mission` ร้อย Nav2 + MoveIt เข้าด้วยกัน · interlock ฮาร์ดแวร์ · RC override ตลอดเวลา · Foxglove สั่งผ่านเว็บ | ⬜ |

> M1–M10 = สาย **ซอฟต์แวร์/ซิม** (ทำได้เลย) · M11–M14 ≈ roadmap **Stage 1–2** (รอฮาร์ดแวร์ฐาน) · M15–M18 ≈ roadmap **Stage 3** (Pi + AR4)

### M3 — รายละเอียด (ทำอยู่)

โซ่: `keyboard_teleop → /cmd_vel → relay_bridge → USB serial → MEGA 2560 → relay ×2 → PWM governor (ลูกบิดตั้งความเร็ว) → DC motor`

- โปรโตคอล serial: บรรทัดละคำสั่ง 115200 · PC→MEGA `F`/`B`/`S`/`P` · MEGA→PC `READY`/`STATE:*`/`WARN:*`/`OK`
- `relay_bridge` แปลง `linear.x` เป็น `F`/`B`/`S` (ยังไม่สนใจ `linear.y`, `angular.z` — มอเตอร์เดียว)
- ความปลอดภัยในเฟิร์มแวร์: watchdog 500 ms · dead-time 500 ms ตอนกลับทาง · ปิด relay ทั้งคู่ตอนบูต
- ไฟล์: `firmware/relay_motor_mega/relay_motor_mega.ino` · `ros2_ws/src/humanoi_base/`
- การต่อวงจร + ตารางความจริง relay: `docs/wiring_relay_motor.md`
- ตรรกะเฟิร์มแวร์ยกมาจาก `~/Downloads/ros2_relay_motor.zip` (พิสูจน์แล้ว) ปรับให้เข้ากับอินเทอร์เฟซ `/cmd_vel`

### M3b — 4 มอเตอร์ mecanum

- `relay_bridge` param `protocol=mecanum4` : ทำ mecanum mixing `/cmd_vel` → เครื่องหมายของ 4 ล้อ → ส่ง `W f f f f`
- firmware `firmware/relay_motor_mega_4wd/` : relay 8 ตัว (pin 22–29), state machine + dead-time + watchdog ต่อล้อ
- Gazebo ไม่ต้องแก้ (VelocityControl รับ twist ตรงๆ)
- การต่อวงจร + ตาราง mixing + ขั้นทดสอบ A–E: `docs/wiring_relay_motor_4wd.md`

### M8 — รายละเอียด (พร้อมทดสอบ)

กลไกที่ต้องใช้ทั้งหมด**มีอยู่แล้ว**ใน `nav2.yaml` ตั้งแต่ตอนตั้งค่า M7 (ไม่ต้องแก้โค้ด) :

- `global_costmap` และ `local_costmap` มี `obstacle_layer` อ่าน `/scan` (marking+clearing) และ `inflation_layer`
  (`inflation_radius: 0.45`) — สิ่งกีดขวางใหม่ที่ scan เห็นจะโผล่ในทั้งสอง costmap แม้ static map ไม่รู้จัก
- `bt_navigator` ไม่ได้ตั้ง `default_nav_to_pose_bt_xml_filename` → ใช้ BT ดีฟอลต์ของ Nav2
  (`navigate_to_pose_w_replanning_and_recovery.xml`) ซึ่ง replan เส้นทาง global ทุก ~1 วิ +
  fallback ไป recovery (`spin`/`backup`/`wait`, ตั้งไว้ใน `behavior_server`) ถ้าเส้นทางล้มเหลว
- `FollowPath` (RPP) เช็คชนกับ local costmap ระหว่างวิ่งอยู่แล้ว (`use_collision_detection` ดีฟอลต์ true)

สิ่งที่เพิ่มสำหรับทดสอบ:

- world ใหม่ `ros2_ws/src/humanoi_description/worlds/room_obstacle.sdf` = `room.sdf` + กล่องแดง `box_new`
  ที่ (2.3, 0.3) ขวางเส้นตรงจากจุดสปอว์นไปทาง +x — **ไม่มีใน `maps/room.pgm`** (แผนที่สร้างจาก `room.sdf` เดิม)
  จึงจำลองสถานการณ์ "สิ่งกีดขวางใหม่ไม่อยู่ในแผนที่" ได้ตรงตามเกณฑ์ผ่าน
- ขั้นตอนทดสอบเต็ม: `docs/RUN.md` หัวข้อ 9
- **เสริม — สิ่งกีดขวางเคลื่อนที่ (คนเดินตัดหน้า):** world `room_walker.sdf` มีโมเดล `walker`
  (ทรงกระบอกแทนคน, link ตั้ง `kinematic` กันล้มคว่ำตอนหยุดกะทันหัน) คุมผ่าน `/walker/cmd_vel`
  (bridge ใหม่ใน `config/gz_bridge.yaml`) — โหนด `humanoi_teleop walker_patrol` สั่งให้เดินตัด-กลับ
  วนซ้ำอัตโนมัติ (ไม่ต้องสั่งมือทีละครั้ง) ตอนหุ่นวิ่งไป goal คงที่อยู่แล้ว ทดสอบว่า costmap มาร์ค/เคลียร์
  ตามจริงเรียลไทม์ซ้ำได้หลายรอบ ขั้นตอน: `docs/RUN.md` หัวข้อ 9b

รอผลทดสอบจริงจากผู้ใช้ก่อนขึ้นสถานะ ✅ ผ่าน — ถ้า global path ไม่อ้อม (พุ่งชนกล่อง) หรือ controller ค้าง
ให้ดูหัวข้อ "แก้ปัญหาเร็ว" ใน RUN.md ก่อน แล้วค่อยพิจารณาแก้ `nav2.yaml` (เช่น เพิ่มความถี่ replanning
ใน BT xml เอง แทนตัวดีฟอลต์)

## สแตกที่เลือก

- **ROS 2 Jazzy** + **Gazebo Harmonic** (gz-sim 8) บน **WSL Ubuntu-24.04**
- ขับใน sim ด้วย `VelocityControl` (ขยับตัวถังตาม `/cmd_vel` ตรงๆ แบบ kinematic) + `OdometryPublisher` (odom จากตำแหน่งจริง)
  — ไม่ใช้ `MecanumDrive` เพราะล้อทรงกระบอกไม่มีลูกกลิ้ง 45° จึงสไลด์ข้างไม่ออก และ odom ของมัน dead-reckon จากคำสั่ง
- mecanum mixing จริง (twist → 4 wheel rpm) จะไปเขียนในเฟิร์มแวร์ ESP32 / โหนด bridge ตอนต่อของจริง
- Transport ROS ↔ ESP32 เริ่มด้วย **custom serial bridge** (debug ง่าย) แล้วค่อยพิจารณา micro-ROS ตอน Stage 3 ตาม roadmap

## แพ็กเกจ

| แพ็กเกจ | หน้าที่ |
|---|---|
| `humanoi_description` | URDF/xacro ฐาน mecanum + โลก Gazebo + `sim.launch.py` + ตั้งค่า ros_gz bridge |
| `humanoi_teleop` | โหนด `keyboard_teleop` — อ่านคีย์บอร์ด publish `/cmd_vel` |
| `humanoi_base` | โหนด `relay_bridge` — `/cmd_vel` → คำสั่ง `F`/`B`/`S` → serial ไป Arduino + อ่านสถานะกลับที่ `/motor_state` |

## เฟส navigation (M5–M8) — ทำในซิม

- LiDAR จำลอง 2D (`gpu_lidar` 360° 12 m) ที่ `laser_link` → `/scan` (จำลอง RPLIDAR/LD19 ของ Stage 3)
- โลก `room.sdf` : ห้อง 10×8 m + กล่อง 3 ชิ้น สำหรับ map + หลบ
- SLAM: `slam_toolbox` (async) · Localization: AMCL · Nav stack: `nav2_bringup` + params เฉพาะหุ่น
- controller เริ่มแบบ diff-drive (vx + wz) ให้ตรง preset ก่อน แล้วค่อยเปิด holonomic (vy) ทีหลัง
- odom ในซิมสมบูรณ์แบบ (ground-truth) — บนหุ่นจริงต้องรอ dead-wheel pod (Stage 2) จึงจะ nav ได้จริง
- ติดตั้งเพิ่ม: `ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox`

## เฟสแขน AR4 (M15–M18) — บันทึกออกแบบ

- **การติดตั้ง:** AR4 ติดบน `base_link` (ฝาบนฐาน) — เพิ่ม `<xacro:include>` ของ AR4 ใน URDF ฐาน, joint `fixed` จาก base_link → `ar4_base`
- **ควบคุม:** AR4 ใช้ Teensy 4.1 + `ar4_ros_driver` (ทดสอบบน Jazzy โดยตรง) · gripper = Arduino Nano แยก · วางแผน/เล่นท่าผ่าน **MoveIt 2**
- **TF:** ต้องต่อเนื่อง `odom → base_footprint → base_link → ar4_base → …ข้อต่อแขน… → gripper` เพื่อรู้ว่าปลายแขนอยู่ที่ไหนในโลก
- **ไฟ:** AR4 ต้องมี DC-DC แยกวงจรจากมอเตอร์ฐาน (กัน noise จาก relay/มอเตอร์เข้า stepper driver)
- **ความปลอดภัย (interlock) — สำคัญที่สุด:**
  - แขนยื่นด้านข้างขณะฐานวิ่ง = เสี่ยงพลิกคว่ำ → บังคับให้แขนอยู่ท่า **stow** (พับเก็บ ศูนย์ถ่วงต่ำ อยู่ในกรอบฐาน) ก่อน `humanoi_mission` สั่ง Nav2 ขับ
  - ฐานต้อง `/cmd_vel = 0` และนิ่งจริง (เช็ค `/odom`) ก่อน MoveIt เริ่ม execute
  - RC (หรือคีย์บอร์ด) override หยุดได้ทุกเมื่อ — ทั้งฐานและแขน
- **แพ็กเกจใหม่ที่จะเพิ่ม:** `humanoi_moveit` (config MoveIt ของ AR4+ฐาน) · `humanoi_mission` (orchestrator) · อาจมี `humanoi_arm_bringup`

## รู้แล้วว่าขาดอะไร (อัปเดตเรื่อยๆ)

- ขนาดฐานจริง (ล้อ Ø, track, wheelbase) — ตอนนี้ใน xacro เป็นค่าสมมติ
- วิธีวัด odometry บนของจริง (มอเตอร์ไม่มี encoder) — sim ใช้ ground-truth ไปก่อน
- โปรโตคอล serial (F/B/S/P + STATE/WARN) นิยามแล้วสำหรับ relay 1 มอเตอร์ — ตอนขยับเป็น 4 มอเตอร์
  หรือ ESP32+MDDS10 ต้องออกแบบเฟรมใหม่ (ต่อล้อ + ความเร็ว + ทิศ)
- ยังไม่ตัดสิน micro-ROS vs serial bridge สำหรับของจริงระยะยาว
- ความเร็วมอเตอร์ในเฟส relay ตั้งด้วยลูกบิด PWM ด้วยมือ — ยังคุมจาก ROS ไม่ได้จนกว่าจะถึง MDDS10
