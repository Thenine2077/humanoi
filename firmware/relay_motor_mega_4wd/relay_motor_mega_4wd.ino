/*
 * relay_motor_mega_4wd.ino
 * Arduino MEGA 2560 - คุมทิศทาง DC motor 4 ตัว (ฐาน mecanum) ด้วย relay 8 ตัว
 * ความเร็วตั้งด้วยลูกบิดโมดูล PWM governor แยกต่างหาก (เท่ากันทุกล้อ)
 *
 * ล้อ index:  0=FL(หน้าซ้าย)  1=FR(หน้าขวา)  2=RL(หลังซ้าย)  3=RR(หลังขวา)
 *
 *   relay เดินหน้า : pin 22, 24, 26, 28   (FL, FR, RL, RR)
 *   relay ถอยหลัง  : pin 23, 25, 27, 29
 *   pin 13 = LED : กะพริบช้า = หยุดทั้งหมด, ติดค้าง = มีล้อหมุนอย่างน้อย 1
 *
 * การต่อ relay ต่อล้อ (SPDT 2 ตัว เป็น H-bridge กลับขั้ว):
 *   COM -> ขั้วมอเตอร์,  NO -> OUT+ ของ governor,  NC -> OUT- ของ governor
 *   FWD ON+REV OFF = หมุนหน้า | FWD OFF+REV ON = หมุนหลัง | ทั้งคู่ OFF = หยุด
 *
 * โปรโตคอล serial (115200, บรรทัดละคำสั่ง จบด้วย \n):
 *   PC -> MEGA :
 *     W f f f f   ตั้งทิศ 4 ล้อ  f = F|B|S  (เว้นวรรคหรือไม่ก็ได้: "W F B S F" หรือ "WFBSF")
 *     S           หยุดทุกล้อ
 *     P           ping -> ตอบ "OK"
 *   MEGA -> PC :
 *     "READY relay_motor_mega_4wd"
 *     "STATE:FBSF"                (ลำดับ FL FR RL RR, ส่งเมื่อมีล้อเปลี่ยนสถานะ)
 *     "WARN:timeout -> STOP"      (watchdog ตัด)
 *     "OK"
 *
 * ความปลอดภัย:
 *   1) Watchdog - ไม่ได้รับ W/S เกิน CMD_TIMEOUT ms -> หยุดทุกล้อ
 *   2) Dead time ต่อล้อ - ล้อที่กำลังหมุนแล้วสั่งกลับทาง ต้องผ่าน STOP ค้าง REVERSE_DELAY ms ก่อน
 *   3) ปิด relay ทั้ง 8 ใน setup()
 */

// ---------------- ค่าคงที่ที่อาจต้องปรับ ----------------
const uint8_t PIN_FWD[4] = {22, 24, 26, 28};   // FL, FR, RL, RR
const uint8_t PIN_REV[4] = {23, 25, 27, 29};
const uint8_t LED_PIN = 13;

const bool RELAY_ACTIVE_LOW = true;            // โมดูล relay ส่วนใหญ่ LOW = ทำงาน

const unsigned long CMD_TIMEOUT   = 500;       // ไม่มีคำสั่งเกินนี้ (ms) = หยุด
const unsigned long REVERSE_DELAY = 500;       // หยุดค้างก่อนกลับทาง (ms)
const unsigned long BLINK_MS      = 500;
// ------------------------------------------------------

enum Dir : uint8_t { D_STOP = 0, D_FWD = 1, D_REV = 2 };

Dir current[4] = {D_STOP, D_STOP, D_STOP, D_STOP};
Dir target[4]  = {D_STOP, D_STOP, D_STOP, D_STOP};
unsigned long stopStart[4] = {0, 0, 0, 0};
bool waitingDeadTime[4] = {false, false, false, false};

unsigned long lastCmdTime = 0;
unsigned long lastBlink = 0;
bool blinkState = false;

char buf[24];
uint8_t bufLen = 0;

void relayWrite(uint8_t pin, bool on) {
  digitalWrite(pin, RELAY_ACTIVE_LOW ? (on ? LOW : HIGH)
                                     : (on ? HIGH : LOW));
}

void wheelOff(uint8_t i) {
  relayWrite(PIN_FWD[i], false);
  relayWrite(PIN_REV[i], false);
}

void allOff() {
  for (uint8_t i = 0; i < 4; i++) wheelOff(i);
}

char dirChar(Dir d) {
  return d == D_FWD ? 'F' : (d == D_REV ? 'B' : 'S');
}

void reportState() {
  Serial.print(F("STATE:"));
  for (uint8_t i = 0; i < 4; i++) Serial.print(dirChar(current[i]));
  Serial.println();
}

void applyWheel(uint8_t i, Dir d) {
  if (d == D_FWD) {
    relayWrite(PIN_REV[i], false);
    relayWrite(PIN_FWD[i], true);
  } else if (d == D_REV) {
    relayWrite(PIN_FWD[i], false);
    relayWrite(PIN_REV[i], true);
  } else {
    wheelOff(i);
  }
  current[i] = d;
}

Dir parseDir(char c) {
  if (c == 'F' || c == 'f') return D_FWD;
  if (c == 'B' || c == 'b') return D_REV;
  return D_STOP;
}

void handleLine(char *s, uint8_t n) {
  if (n == 0) return;

  if (s[0] == 'P' || s[0] == 'p') {
    Serial.println(F("OK"));
    return;
  }
  if (s[0] == 'S' || s[0] == 's') {
    for (uint8_t i = 0; i < 4; i++) target[i] = D_STOP;
    lastCmdTime = millis();
    return;
  }
  if (s[0] == 'W' || s[0] == 'w') {
    Dir tmp[4];
    uint8_t w = 0;
    for (uint8_t k = 1; k < n && w < 4; k++) {
      if (s[k] == ' ') continue;
      if (s[k] != 'F' && s[k] != 'f' && s[k] != 'B' && s[k] != 'b'
          && s[k] != 'S' && s[k] != 's') return;   // อักขระแปลก -> ทิ้งทั้งบรรทัด
      tmp[w++] = parseDir(s[k]);
    }
    if (w != 4) return;                             // ไม่ครบ 4 -> ทิ้ง
    for (uint8_t i = 0; i < 4; i++) target[i] = tmp[i];
    lastCmdTime = millis();
  }
}

void updateLed(unsigned long now) {
  bool moving = false;
  for (uint8_t i = 0; i < 4; i++) if (current[i] != D_STOP) moving = true;

  if (moving) {
    digitalWrite(LED_PIN, HIGH);
  } else if (now - lastBlink >= BLINK_MS) {
    lastBlink = now;
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}

void setup() {
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(PIN_FWD[i], OUTPUT);
    pinMode(PIN_REV[i], OUTPUT);
  }
  pinMode(LED_PIN, OUTPUT);
  allOff();

  Serial.begin(115200);
  while (!Serial) { ; }
  delay(300);
  Serial.println(F("READY relay_motor_mega_4wd"));
  lastCmdTime = millis();
}

void loop() {
  unsigned long now = millis();

  // ---------- อ่าน Serial ----------
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (bufLen > 0) { buf[bufLen] = '\0'; handleLine(buf, bufLen); bufLen = 0; }
    } else if (bufLen < sizeof(buf) - 1) {
      buf[bufLen++] = c;
    }
  }

  // ---------- Watchdog ----------
  bool anyTarget = false;
  for (uint8_t i = 0; i < 4; i++) if (target[i] != D_STOP) anyTarget = true;
  if (now - lastCmdTime > CMD_TIMEOUT && anyTarget) {
    for (uint8_t i = 0; i < 4; i++) target[i] = D_STOP;
    Serial.println(F("WARN:timeout -> STOP"));
  }

  // ---------- State machine ต่อล้อ + dead time ----------
  bool changed = false;
  for (uint8_t i = 0; i < 4; i++) {
    if (waitingDeadTime[i]) {
      if (target[i] == D_STOP) {
        waitingDeadTime[i] = false;
      } else if (now - stopStart[i] >= REVERSE_DELAY) {
        waitingDeadTime[i] = false;
        applyWheel(i, target[i]);
        changed = true;
      }
    }
    else if (target[i] != current[i]) {
      if (current[i] == D_STOP) {
        applyWheel(i, target[i]);            // จากหยุด -> ออกตัวทันที
      } else {
        applyWheel(i, D_STOP);               // หมุนอยู่ -> หยุดก่อน
        if (target[i] != D_STOP) {
          stopStart[i] = now;
          waitingDeadTime[i] = true;
        }
      }
      changed = true;
    }
  }
  if (changed) reportState();

  updateLed(now);
}
