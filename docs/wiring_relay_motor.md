# การต่อวงจร — ทดสอบ M3 (Arduino MEGA + relay ×2 + PWM governor + DC motor 1 ตัว)

## บล็อกไดอะแกรม

```
                 USB
  ROS 2 PC  <========>  Arduino MEGA 2560
  (relay_bridge)         │ pin 2 ──► IN1  โมดูล relay ช่อง FWD
                         │ pin 3 ──► IN2  โมดูล relay ช่อง REV
                         │ 5V,GND ─► VCC,GND โมดูล relay
                         └ pin 13 = LED บนบอร์ด (สถานะ)

  แหล่งจ่ายมอเตอร์ (5–35 V)  ──►  โมดูล PWM governor  ──►  [OUT+ , OUT−]
                                   (ลูกบิด = ตั้งความเร็ว)         │
                                                                  ▼
                        ┌─────────── relay FWD (SPDT) ───────────┐
              OUT+ ──────┤ NO                                     │
              OUT− ──────┤ NC          COM ──► ขั้วมอเตอร์ A       │
                        └────────────────────────────────────────┘
                        ┌─────────── relay REV (SPDT) ───────────┐
              OUT+ ──────┤ NO                                     │
              OUT− ──────┤ NC          COM ──► ขั้วมอเตอร์ B       │
                        └────────────────────────────────────────┘
```

> ⚠️ กราวด์: ต่อ GND ของแหล่งจ่ายมอเตอร์กับ GND ของ Arduino เข้าด้วยกัน (common ground)
> แต่ **อย่า** เอา 5V มอเตอร์ป้อนเข้า Arduino — Arduino จ่ายไฟผ่าน USB จาก PC

## ตารางความจริงของ relay

| relay FWD | relay REV | มอเตอร์ A | มอเตอร์ B | ผล |
|:---:|:---:|:---:|:---:|---|
| OFF | OFF | OUT− | OUT− | **หยุด** (สองขั้วเท่ากัน ไม่มีกระแส) |
| ON  | OFF | OUT+ | OUT− | **เดินหน้า** |
| OFF | ON  | OUT− | OUT+ | **ถอยหลัง** |
| ON  | ON  | OUT+ | OUT+ | หยุด (สองขั้วเท่ากัน) — สถานะนี้ไม่ถูกสั่งโดยเฟิร์มแวร์ |

เฟิร์มแวร์เปิด relay ทีละตัวเท่านั้น และผ่าน STOP ก่อนเสมอเมื่อกลับทาง

## ค่าคงที่ในเฟิร์มแวร์ (`relay_motor_mega.ino`)

| ค่า | ดีฟอลต์ | หมายเหตุ |
|---|---|---|
| `RELAY_FWD` / `RELAY_REV` | 2 / 3 | ขา Arduino ที่ต่อ IN1 / IN2 |
| `RELAY_ACTIVE_LOW` | `true` | โมดูล relay ส่วนใหญ่ LOW = ทำงาน · ทดสอบด้วย LED เปล่าให้ตั้ง `false` |
| `CMD_TIMEOUT` | 500 ms | ไม่มีคำสั่งเกินนี้ → หยุดเอง |
| `REVERSE_DELAY` | 500 ms | หยุดค้างก่อนกลับทาง |

## ทดสอบเป็นขั้น

1. **ยังไม่ต่อ relay** — อัปโหลด sketch (ตั้ง `RELAY_ACTIVE_LOW=false` ชั่วคราว)
   เปิด Serial Monitor 115200 ต้องเห็น `READY relay_motor_mega`
   พิมพ์ `F` / `B` / `S` ดู `STATE:*` ตอบกลับ และ LED pin 13 (กะพริบ=STOP, ติด=หมุน)
2. **ต่อ relay แต่ยังไม่ต่อมอเตอร์** — ฟังเสียง relay คลิกตามคำสั่ง
   ตั้ง `RELAY_ACTIVE_LOW` กลับเป็นค่าที่ตรงกับโมดูล
3. **ต่อ PWM governor + มอเตอร์** — หมุนลูกบิด PWM ไว้ต่ำสุดก่อน
   สั่ง `F` ค่อยๆ เพิ่มลูกบิด · ทดสอบ `B` · ทดสอบถอดสาย USB ระหว่างหมุน (ต้องหยุดใน 0.5 วิ)
4. **ผ่าน ROS** — ดูหัวข้อ "รันทดสอบ" ด้านล่าง

## รันทดสอบผ่าน ROS 2 (ใน WSL Ubuntu-24.04)

MEGA เสียบที่ Windows จะเป็น COMx — ต้อง forward เข้า WSL ก่อน (PowerShell แบบ admin):

```powershell
winget install usbipd            # ครั้งเดียว
usbipd list                      # หาบัส MEGA (Arduino Mega 2560)
usbipd bind    --busid <BUSID>
usbipd attach  --wsl --busid <BUSID>
```

ใน WSL จะเห็น `/dev/ttyACM0` (เช็ค `ls /dev/ttyACM*` และ `sudo usermod -aG dialout $USER` ครั้งแรกแล้ว re-login)

```bash
cd /mnt/c/Users/User/Desktop/Humanoi/ros2_ws && source install/setup.bash

# หน้าต่าง 1
ros2 run humanoi_base relay_bridge --ros-args -p port:=/dev/ttyACM0

# หน้าต่าง 2
ros2 run humanoi_teleop keyboard_teleop
```

ปุ่ม `w` = เดินหน้า, `s` = ถอย, `space` = หยุด (relay_bridge สนใจแค่ `linear.x`)

ดู debug: `ros2 topic echo /motor_command` (F/B/S ที่ส่ง) · `ros2 topic echo /motor_state` (บรรทัดจาก MEGA)

## ข้อควรระวังเฉพาะโมดูล PWM governor ตัวนี้

- ลูกบิดคุมความเร็วเป็น **อนาล็อก** Arduino ไม่ได้คุม — ตั้งด้วยมือ
- มีคาปาซิเตอร์ก้อนใหญ่ที่อินพุต: ตอน relay สลับ อาจมีไฟกระชากเข้ามอเตอร์ → เริ่มที่ความเร็วต่ำ
- โมดูลชนิดนี้ PWM ที่ขา **ลบ** (low-side) การเอา relay ไปสลับขั้วเอาต์พุตทำได้
  แต่ dead-time 500 ms สำคัญมาก อย่าลดค่านี้
- ถ้ามอเตอร์ไม่หมุนแต่ relay คลิก: เช็คว่าลูกบิด PWM ไม่ได้อยู่ที่ศูนย์ และขั้ว OUT+/OUT− ไม่สลับกับ IN
