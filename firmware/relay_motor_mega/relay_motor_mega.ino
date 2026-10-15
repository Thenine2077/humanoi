/*
 * relay_motor_mega.ino
 * Arduino MEGA 2560 - รับคำสั่งจาก ROS 2 ผ่าน USB Serial เพื่อกลับทาง DC motor 1 ตัว
 * ด้วย relay 2 ตัว (H-bridge) ความเร็วตั้งด้วยลูกบิดของโมดูล PWM แยกต่างหาก
 *
 * โซ่การต่อ (ทดสอบ M3):
 *   ROS 2 (โหนด relay_bridge) --USB--> MEGA --2 ขา--> โมดูล relay --> โมดูล PWM governor --> DC motor
 *
 *   pin 2  = Relay เดินหน้า (RELAY_FWD)
 *   pin 3  = Relay ถอยหลัง (RELAY_REV)
 *   pin 13 = LED บนบอร์ด : กะพริบช้า = STOP/พร้อม, ติดค้าง = กำลังหมุน
 *            (ใช้ดู pipeline ROS->Serial ได้ก่อน โดยยังไม่ต้องต่อ relay จริง)
 *
 * การต่อ relay เป็น H-bridge (ต่อ 1 ตัวต่อ 1 relay SPDT):
 *   COM -> ขั้วมอเตอร์
 *   NO  -> ไฟ + ที่ออกจากโมดูล PWM
 *   NC  -> ไฟ - ที่ออกจากโมดูล PWM
 *   FWD ON  + REV OFF -> มอเตอร์ +/-  = เดินหน้า
 *   FWD OFF + REV ON  -> มอเตอร์ -/+  = ถอยหลัง
 *   ทั้งคู่ OFF        -> ทั้งสองขั้วอยู่ฝั่งเดียวกัน = หยุด (มอเตอร์ไม่ถูกจ่ายไฟ)
 *
 * โปรโตคอล serial (บรรทัดละคำสั่ง จบด้วย \n, 115200 baud):
 *   PC -> MEGA :  F = เดินหน้า, B = ถอยหลัง, S = หยุด, P = ping
 *   MEGA -> PC :  "READY relay_motor_mega"
 *                 "STATE:FWD" / "STATE:REV" / "STATE:STOP"   (ส่งเมื่อสถานะเปลี่ยน)
 *                 "WARN:timeout -> STOP"                     (watchdog ตัด)
 *                 "OK"                                       (ตอบ ping)
 *
 * ความปลอดภัยที่ใส่ไว้:
 *   1) Watchdog  - ไม่ได้รับคำสั่งเกิน CMD_TIMEOUT ms -> หยุดเอง (สาย USB หลุด / ROS ตาย)
 *   2) Dead time - ต้องหยุดค้าง REVERSE_DELAY ms ก่อนจึงกลับทางได้
 *                  กันกระแสกระชากและหน้าสัมผัส relay อาร์คจาก back-EMF
 *   3) ปิด relay ทั้งคู่ใน setup() ก่อนเสมอ
 */

// ---------------- ค่าคงที่ที่อาจต้องปรับ ----------------
const uint8_t RELAY_FWD = 2;
const uint8_t RELAY_REV = 3;
const uint8_t LED_PIN   = 13;

// โมดูล relay ส่วนใหญ่เป็น Active LOW (ขา LOW = relay ทำงาน)
// ตอนทดสอบด้วย LED เปล่าให้ตั้ง false ก่อน แล้วค่อยเปลี่ยนเป็น true เมื่อต่อ relay จริง
const bool RELAY_ACTIVE_LOW = true;

const unsigned long CMD_TIMEOUT   = 500;   // ไม่มีคำสั่งเกินนี้ (ms) = หยุด
const unsigned long REVERSE_DELAY = 500;   // หยุดค้างก่อนกลับทาง (ms)
const unsigned long BLINK_MS      = 500;   // คาบกะพริบ LED ตอน STOP
// ------------------------------------------------------

enum Dir : uint8_t { D_STOP = 0, D_FWD = 1, D_REV = 2 };

Dir current = D_STOP;      // สถานะจริงของ relay ตอนนี้
Dir target  = D_STOP;      // สถานะที่อยากให้เป็น

unsigned long lastCmdTime = 0;
unsigned long stopStart   = 0;
unsigned long lastBlink   = 0;
bool waitingDeadTime = false;
bool blinkState = false;

char buf[16];
uint8_t bufLen = 0;

void relayWrite(uint8_t pin, bool on) {
  digitalWrite(pin, RELAY_ACTIVE_LOW ? (on ? LOW : HIGH)
                                     : (on ? HIGH : LOW));
}

void allOff() {
  relayWrite(RELAY_FWD, false);
  relayWrite(RELAY_REV, false);
}

void reportState() {
  Serial.print(F("STATE:"));
  if (current == D_FWD)      Serial.println(F("FWD"));
  else if (current == D_REV) Serial.println(F("REV"));
  else                       Serial.println(F("STOP"));
}

void applyDir(Dir d) {
  if (d == D_FWD) {
    relayWrite(RELAY_REV, false);
    relayWrite(RELAY_FWD, true);
  } else if (d == D_REV) {
    relayWrite(RELAY_FWD, false);
    relayWrite(RELAY_REV, true);
  } else {
    allOff();
  }
  current = d;
  reportState();
}

void handleCommand(char c) {
  switch (c) {
    case 'F': case 'f': target = D_FWD;  lastCmdTime = millis(); break;
    case 'B': case 'b': target = D_REV;  lastCmdTime = millis(); break;
    case 'S': case 's': target = D_STOP; lastCmdTime = millis(); break;
    case 'P': case 'p': Serial.println(F("OK"));                 break;
    default: break;
  }
}

void updateLed(unsigned long now) {
  if (current == D_STOP) {
    if (now - lastBlink >= BLINK_MS) {
      lastBlink = now;
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
    }
  } else {
    digitalWrite(LED_PIN, HIGH);
  }
}

void setup() {
  pinMode(RELAY_FWD, OUTPUT);
  pinMode(RELAY_REV, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  allOff();                        // ปิดทั้งคู่ก่อนเสมอ

  Serial.begin(115200);
  while (!Serial) { ; }
  delay(300);
  Serial.println(F("READY relay_motor_mega"));
  lastCmdTime = millis();
}

void loop() {
  unsigned long now = millis();

  // ---------- อ่านคำสั่งจาก Serial ----------
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (bufLen > 0) { handleCommand(buf[0]); bufLen = 0; }
    } else if (bufLen < sizeof(buf) - 1) {
      buf[bufLen++] = c;
    }
  }

  // ---------- Watchdog ----------
  if (now - lastCmdTime > CMD_TIMEOUT && target != D_STOP) {
    target = D_STOP;
    Serial.println(F("WARN:timeout -> STOP"));
  }

  // ---------- State machine + dead time ----------
  if (waitingDeadTime) {
    if (target == D_STOP) {                  // สั่งหยุดระหว่างรอ -> จบเลย
      waitingDeadTime = false;
    } else if (now - stopStart >= REVERSE_DELAY) {
      waitingDeadTime = false;
      applyDir(target);
    }
  }
  else if (target != current) {
    if (current == D_STOP) {
      applyDir(target);                      // จากหยุด -> ออกตัวได้ทันที
    } else {
      applyDir(D_STOP);                      // หมุนอยู่ -> หยุดก่อน
      if (target != D_STOP) {
        stopStart = now;
        waitingDeadTime = true;              // แล้วรอ REVERSE_DELAY จึงกลับทาง
      }
    }
  }

  updateLed(now);
}
