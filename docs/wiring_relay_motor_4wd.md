# การต่อวงจร — 4 มอเตอร์ mecanum (Arduino MEGA + relay 8 ตัว + PWM governor)

ต่อยอดจาก `wiring_relay_motor.md` (1 มอเตอร์) — ทำซ้ำแนวคิดเดิม 4 ชุด

## บล็อกไดอะแกรม

```
                      USB
  ROS 2 PC  <=========>  Arduino MEGA 2560
  (relay_bridge             │
   protocol=mecanum4)       │  pin 22,24,26,28 -> IN เดินหน้า (FL,FR,RL,RR)
                            │  pin 23,25,27,29 -> IN ถอยหลัง  (FL,FR,RL,RR)
                            │  5V,GND -> เลี้ยงโมดูล relay
                            └  pin 13 = LED สถานะ

  แหล่งจ่าย 24V ──► PWM governor (ลูกบิด) ──► บัส OUT+ / OUT−
                                                │
              บัส OUT+/OUT−  แยกเข้า relay 2 ตัว ต่อ 1 ล้อ (รวม 8 ตัว)
              แต่ละคู่:  NO->OUT+  NC->OUT−  COMx2->ขั้วมอเตอร์ A,B ของล้อนั้น
```

> governor 1 ตัวป้อนทั้ง 4 มอเตอร์ขนานกันได้ (ความเร็วเท่ากันทุกล้อ พอสำหรับทิศ 8 แบบ)
> ถ้าอยากคุมความเร็วต่อล้อจริง ต้องรอ MDDS10 (Stage 1 ของ roadmap)

## ตาราง pin ↔ ล้อ

| ล้อ | index | relay FWD (pin) | relay REV (pin) |
|---|:---:|:---:|:---:|
| หน้าซ้าย FL | 0 | 22 | 23 |
| หน้าขวา FR | 1 | 24 | 25 |
| หลังซ้าย RL | 2 | 26 | 27 |
| หลังขวา RR | 3 | 28 | 29 |

ปรับได้ที่ `PIN_FWD[]` / `PIN_REV[]` บนหัวไฟล์ `relay_motor_mega_4wd.ino`

## โปรโตคอล serial

| PC → MEGA | ความหมาย |
|---|---|
| `W f f f f` | ตั้งทิศ 4 ล้อ `f` = `F`\|`B`\|`S` ลำดับ FL FR RL RR (เว้นวรรคหรือไม่ก็ได้) |
| `S` | หยุดทุกล้อ |
| `P` | ping → `OK` |

| MEGA → PC | ความหมาย |
|---|---|
| `READY relay_motor_mega_4wd` | บูตเสร็จ |
| `STATE:FBSF` | ทิศปัจจุบัน FL FR RL RR (ส่งเมื่อมีล้อเปลี่ยน) |
| `WARN:timeout -> STOP` | watchdog ตัด |

## mecanum mixing (ทำใน relay_bridge)

จาก twist ตัวถัง `(vx, vy, wz)` → ความเร็วเชิงมุมเป้าหมายต่อล้อ แล้วเอา**เครื่องหมาย**:

```
k = (track + wheelbase) / 2
FL = vx - vy - k*wz
FR = vx + vy + k*wz
RL = vx + vy - k*wz
RR = vx - vy + k*wz
```

| คำสั่ง | FL | FR | RL | RR |
|---|:---:|:---:|:---:|:---:|
| เดินหน้า `w` | F | F | F | F |
| ถอย `s` | B | B | B | B |
| สไลด์ซ้าย `a` | B | F | F | B |
| สไลด์ขวา `d` | F | B | B | F |
| หมุนซ้าย `q` | B | F | B | F |
| หมุนขวา `e` | F | B | F | B |
| ทแยงหน้า-ซ้าย | S | F | F | S |
| ทแยงหน้า-ขวา | F | S | S | F |

ดูค่า mixing สดๆ : `ros2 topic echo /wheel_target` (Float64MultiArray = [FL, FR, RL, RR] rad/s)

## ทดสอบเป็นขั้น

### ขั้น A — firmware เดี่ยว (ยังไม่ต่อ relay, ตั้ง `RELAY_ACTIVE_LOW=false`)

อัปโหลด → Serial Monitor 115200:
- เห็น `READY relay_motor_mega_4wd`
- พิมพ์ `W F S S S` → `STATE:FSSS` (แค่ FL หน้า) — ทดสอบทีละล้อ
- พิมพ์ `W F F F F` → `STATE:FFFF` + LED pin 13 ติดค้าง
- พิมพ์ `S` → `STATE:SSSS` + LED กะพริบ
- หยุดพิมพ์ 1 วิ หลัง `W F F F F` → `WARN:timeout -> STOP`

### ขั้น B — mixing ฝั่ง ROS (ยังไม่ต้องมีบอร์ด)

```bash
ros2 run humanoi_base relay_bridge --ros-args -p port:=/dev/null -p protocol:=mecanum4
ros2 run humanoi_teleop keyboard_teleop
ros2 topic echo /motor_command      # เห็น "W F B F B" ฯลฯ ตามปุ่ม
ros2 topic echo /wheel_target       # เห็นเลข 4 ตัว
```
เทียบกับตารางด้านบนว่าครบถูก

### ขั้น C — ต่อ relay 8 ตัว (ยังไม่ต่อมอเตอร์)

ตั้ง `RELAY_ACTIVE_LOW` ให้ตรงโมดูล อัปโหลดใหม่ → กดปุ่มแล้วฟังเสียง relay 8 ตัวคลิกตามตาราง

### ขั้น D — ต่อ governor + 4 มอเตอร์

ลูกบิดต่ำสุดก่อน · ยกล้อลอยพื้นตอนเทสแรก · กด `w` → 4 ล้อหมุนหน้า · `a` → ลาย mecanum

### ขั้น E — M4 : Gazebo + 4 มอเตอร์

```bash
ros2 launch humanoi_base sim_and_motor.launch.py
ros2 run humanoi_teleop keyboard_teleop
```
กดปุ่ม → หุ่นใน Gazebo กับล้อจริงไปทางเดียวกัน
