# RUN — คำสั่งรันทดสอบ (cheat sheet)

ทุกหน้าต่าง WSL ขึ้นต้นด้วย:
```bash
wsl -d Ubuntu-24.04
cd /mnt/c/Users/User/Desktop/Humanoi/ros2_ws && source install/setup.bash
```

---

## 1. ซิมอย่างเดียว (M1 / M2)

**WSL 1**
```bash
ros2 launch humanoi_description sim.launch.py
```
**WSL 2**
```bash
ros2 run humanoi_teleop keyboard_teleop
```
(วัด odom: `ros2 run humanoi_teleop square_drive --ros-args -p use_sim_time:=true`)

---

## 2. เชื่อม Arduino เข้า WSL (ต้องทำก่อนทุกครั้งที่จะคุยกับ Mega)

**PowerShell (Windows):**
```
usbipd list
usbipd bind --busid 1-8      # ครั้งเดียวพอ ข้ามได้ถ้าเคยทำ
usbipd attach --wsl --busid 1-8
```
- `not shared` → `usbipd bind --busid 1-8` ก่อน
- จะอัปโหลด sketch ใหม่จาก Arduino IDE → `usbipd detach --busid 1-8` ก่อน แล้ว attach ใหม่หลังอัปโหลด
- เช็คใน WSL: `ls /dev/ttyACM*`

---

## 3. มอเตอร์อย่างเดียว — 1 ตัว (M3)

sketch: `firmware/relay_motor_mega/` · `RELAY_ACTIVE_LOW = true`

**WSL 1**
```bash
ros2 run humanoi_base relay_bridge --ros-args -p port:=/dev/ttyACM0 -p protocol:=single
```
**WSL 2**
```bash
ros2 run humanoi_teleop keyboard_teleop
```

---

## 4. มอเตอร์อย่างเดียว — 4 ล้อ mecanum (M3b / Step D)

sketch: `firmware/relay_motor_mega_4wd/` · `RELAY_ACTIVE_LOW = true`

**WSL 1**
```bash
ros2 run humanoi_base relay_bridge --ros-args -p port:=/dev/ttyACM0 -p protocol:=mecanum4
```
**WSL 2**
```bash
ros2 run humanoi_teleop keyboard_teleop
```
**WSL 3 (ดูคำสั่ง)**
```bash
ros2 topic echo /motor_command
```

---

## 5. Gazebo + 4 มอเตอร์ พร้อมกัน (M4 / Step E)

**WSL 1**
```bash
ros2 launch humanoi_base sim_and_motor.launch.py
```
**WSL 2**
```bash
ros2 run humanoi_teleop keyboard_teleop
```

---

## 6. เช็ค mecanum mixing โดยไม่ต้องมีบอร์ด (dry-run)

**WSL 1**
```bash
ros2 run humanoi_base relay_bridge --ros-args -p port:=none -p protocol:=mecanum4
```
**WSL 2** `keyboard_teleop` · **WSL 3** `ros2 topic echo /motor_command`

---

## 7. SLAM สร้างแผนที่ (M6)

**WSL 1** `ros2 launch humanoi_description sim.launch.py world:=room rviz:=true`
**WSL 2** `ros2 launch humanoi_nav slam.launch.py`
**WSL 3** `ros2 run humanoi_teleop keyboard_teleop`  → ขับทั่วห้อง
**WSL 4** `ros2 run nav2_map_server map_saver_cli -f src/humanoi_nav/maps/room --ros-args -p use_sim_time:=true`

---

## 8. Nav2 ไป goal — ซิมอย่างเดียว (M7)

**WSL 1** `ros2 launch humanoi_description sim.launch.py world:=room rviz:=true`
**WSL 2** `ros2 launch humanoi_nav nav2.launch.py`
RViz: Fixed Frame → `map` → **2D Goal Pose** คลิกจุดหมาย

---

## 9. หลบสิ่งกีดขวางที่ไม่อยู่ในแผนที่ — ซิมอย่างเดียว (M8)

world ใหม่ `room_obstacle` = `room.sdf` เดิม + กล่องแดง `box_new` ที่ (2.3, 0.3) ซึ่ง**ไม่มีใน `maps/room.pgm`**
(แผนที่สร้างจาก `room.sdf` ก่อนเพิ่มกล่องนี้) กีดขวางเส้นตรงจากจุดสปอว์น (0,0) ไปทาง +x พอดี

**WSL 1**
```bash
ros2 launch humanoi_description sim.launch.py world:=room_obstacle rviz:=true
```
**WSL 2** (ยังใช้แผนที่เดิม — ไม่ต้อง SLAM ใหม่)
```bash
ros2 launch humanoi_nav nav2.launch.py
```
RViz: Fixed Frame → `map` → ถ้าตำแหน่งเพี้ยน ใช้ **2D Pose Estimate** ยืนยันที่ (0,0) ก่อน →
**2D Goal Pose** คลิกจุดฝั่งตรงข้ามกล่อง (เช่น x≈3.5, y≈0)

สิ่งที่ควรเห็น:
- global path เส้นแรกพุ่งตรงไปทางกล่อง (เพราะ static map ไม่รู้จักกล่อง)
- พอ `/scan` เห็นกล่อง → global costmap ที่จุดนั้นกลายเป็น obstacle (สีดำ/inflation สีม่วงใน RViz) →
  planner replan (ทุก ~1 วิ ตาม default BT) → เส้นทางหักอ้อมกล่อง
- หุ่นเดินตามเส้นทางใหม่ อ้อมกล่องไปถึง goal โดยไม่ชน

ถ้าเส้นทางไม่อ้อม (พุ่งชนตรงๆ) ให้เช็ค:
- `/scan` ขึ้นใน RViz จริงไหม (topic `LaserScan` → `/scan`, Fixed Frame ต้องไม่ error)
- local/global costmap display เปิดอยู่ใน `view.rviz`
- inflation_layer/obstacle_layer โหลดจาก `nav2.yaml` สำเร็จ (ดู log ตอน controller_server/planner_server ขึ้น)

**อย่า** รัน `map_saver_cli` ทับ `maps/room.pgm` ใน world นี้ — กล่องใหม่ต้องไม่อยู่ในแผนที่ ไม่งั้นเทสต์ M8 ไม่มีความหมาย

---

## 9b. คนเดินตัดหน้าตอนหุ่นวิ่งไป goal คงที่ (M8 เสริม — dynamic obstacle)

ต่างจากหัวข้อ 9 ตรงที่กล่องไม่ได้อยู่ตรงนั้นตั้งแต่แรก — จำลอง "คน" (ทรงกระบอกสีส้ม `walker`)
ที่ยืนอยู่นอกเส้นทาง แล้วค่อยสั่งให้เดินตัดผ่านเส้นทางกลางอากาศตอนหุ่นวิ่งไป goal ที่ตั้งไว้คงที่แล้ว
world ใหม่ `room_walker` — สิ่งกีดขวางแบบเดิม (box_1/box_2/pillar) ตรงกับแผนที่เป๊ะ ใช้แผนที่เดิมได้เลย

**WSL 1**
```bash
ros2 launch humanoi_description sim.launch.py world:=room_walker rviz:=true
```
**WSL 2**
```bash
ros2 launch humanoi_nav nav2.launch.py
```

**ใน RViz:** Fixed Frame → `map` → (ถ้าตำแหน่งเพี้ยน) 2D Pose Estimate ที่ (0,0) → **2D Goal Pose**
คลิกจุด goal คงที่ เช่น x≈4.0, y≈0 (หุ่นเริ่มวิ่งตามเส้นทางที่ยังโล่งอยู่)

**WSL 3** — สั่งให้ walker เดินตัด-กลับวนซ้ำอัตโนมัติด้วยโหนด `walker_patrol`
(เดินขึ้นเหนือ +y ผ่านเส้นทางที่ x≈2.0 แล้วย้อนกลับ -y สลับกันเป็นวงรอบ ไม่ต้องสั่งมือทีละครั้ง):
```bash
ros2 run humanoi_teleop walker_patrol
```
วนไม่รู้จบจนกด Ctrl-C (จะสั่งหยุด walker ให้อัตโนมัติก่อนออก) หรือกำหนดจำนวนรอบ/ความเร็ว/ระยะเอง:
```bash
ros2 run humanoi_teleop walker_patrol --ros-args -p loops:=3 -p speed:=0.6 -p pause:=1.5
```

สิ่งที่ควรเห็น: ตอนแรกหุ่นวิ่งตรงเข้า goal ตามเส้นทางเดิม พอ `/scan` เจอ walker เดินเข้ามาขวาง
local/global costmap อัปเดตทันที (มาร์คจุดที่ walker อยู่ตอนนั้นเป็น obstacle) → หุ่นชะลอ/หยุด หรือ
BT สั่ง replan อ้อม แล้วพอ walker เดินพ้นไป costmap เคลียร์จุดเดิม (clearing จาก scan) → หุ่นกลับไป
วิ่งเส้นทางปกติต่อจนถึง goal เดิมที่ตั้งไว้ (ไม่ต้องตั้ง goal ใหม่) — เกิดซ้ำทุกรอบที่ walker เดินตัดผ่าน

**หมายเหตุ:** link ของ `walker` ตั้งเป็น `kinematic` แล้ว (แก้ปัญหาเดิมที่มันล้มคว่ำตอนสั่งหยุดกะทันหัน
เพราะทรงสูงบาง เป็น dynamic rigid body เต็มรูปแบบ) ถ้าเพิ่งอัปเดตไฟล์ต้อง `colcon build` ใหม่ก่อน

---

## 10. Nav2 + มอเตอร์จริงตามไปด้วย (เสริม — ต่อฮาร์ดแวร์จริงเข้ากับ Nav2 ที่ทดสอบผ่านแล้ว)

sketch: `firmware/relay_motor_mega_4wd/` · usbipd attach ก่อน

**PowerShell** `usbipd list` → `usbipd attach --wsl --busid <X>`
**WSL 1** `ros2 launch humanoi_description sim.launch.py world:=room rviz:=true`
**WSL 2** `ros2 launch humanoi_nav nav2.launch.py`
**WSL 3** `ros2 run humanoi_base relay_bridge --ros-args -p port:=/dev/ttyACM0 -p protocol:=mecanum4`
RViz: Fixed Frame → `map` → **2D Goal Pose**

Nav2 → `/cmd_vel` → ทั้งหุ่นใน Gazebo และ 4 มอเตอร์จริง (ยกล้อลอยพื้น!)

---

## ปุ่มคีย์บอร์ด

`w`/`s` หน้า-หลัง · `a`/`d` สไลด์ซ้าย-ขวา · `q`/`e` หมุน · `space` หยุด · `z`/`c` ปรับความเร็ว

## แก้ปัญหาเร็ว

| อาการ | แก้ |
|---|---|
| `Package not found` | ยังไม่ `source install/setup.bash` |
| `/dev/ttyACM0` permission denied | `sudo usermod -aG dialout $USER` แล้ว `wsl --shutdown` เปิดใหม่ |
| relay ติดหมดตอน idle | `RELAY_ACTIVE_LOW = true` ใน sketch |
| อัปโหลด sketch ไม่ได้ | `usbipd detach --busid 1-8` คืนพอร์ตให้ Windows ก่อน |
| หลัง `wsl --shutdown` | ต้อง `usbipd attach` ใหม่ |
| กด Reset ใน Gazebo แล้วหุ่นหาย | ปกติ — หุ่นถูก spawn ทีหลัง (ไม่ได้อยู่ใน world sdf) Reset จะลบทิ้ง สั่ง `ros2 run ros_gz_sim create -topic robot_description -name humanoi -z 0.0` เพื่อ spawn กลับ (ไม่ต้อง relaunch) แล้วตั้ง 2D Pose Estimate ใหม่ |
