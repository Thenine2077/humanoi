/*
 * humanoi_base_mega.ino
 * Arduino MEGA 2560 - ฐาน mecanum 4 ล้อ (relay 8 ตัว) รับคำสั่งได้ 2 ทาง
 *   1) ROS 2 ผ่าน USB serial (โหนด relay_bridge บน Pi, protocol=mecanum4)
 *   2) รีโมท FlySky FS-ST8 / FS-SR8 (PWM) — มีสิทธิ์เหนือ ROS เสมอ
 *
 * ต่อสายเหมือน rc_mecanum_mega ทุกอย่าง (ไม่ต้องย้ายสาย):
 *   ตัวรับ CH1, CH2, CH4, CH5 (ขา S) -> pin 2, 3, 5, 6
 *   ตัวรับ CH6 (SWB), CH8 (SWD)      -> pin 7, 9   (สวิตช์สั่งแขน — ดู "SW:" ข้างล่าง)
 *   relay เดินหน้า 22, 24, 26, 28 / ถอยหลัง 23, 25, 27, 29  (FL, FR, RL, RR)
 *   USB ของ Mega -> Raspberry Pi (ใน container เห็นเป็น /dev/ttyACM0)
 *   pin 13 = LED: กะพริบเร็ว = ไม่มีสัญญาณรีโมท, กะพริบช้า = ไม่ ARM,
 *                 ติดค้าง = รีโมทคุม, ติดแล้วดับแวบทุก 1 วิ = ROS คุม
 *
 * ใครคุมล้อ:
 *   ไม่มีสัญญาณรีโมท           -> หยุด (รีโมทเป็นสวิตช์ฉุกเฉิน ต้องเปิดไว้เสมอ)
 *   SWA ขึ้น                   -> หยุด ไม่ฟังทั้งรีโมทและ ROS
 *   SWA ลง + ขยับคันโยก        -> รีโมทคุม (คำสั่ง ROS ถูกเมิน)
 *   SWA ลง + คันโยกอยู่กลาง    -> ROS คุม (หลังปล่อยคันโยกนิ่งครบ RC_HOLD_MS)
 *   คันโยก: ขวา ขึ้น/ลง (CH2) = หน้า/ถอย, ขวา ซ้าย/ขวา (CH1) = สไลด์, ซ้าย ซ้าย/ขวา (CH4) = หมุน
 *
 * โปรโตคอล serial (115200, บรรทัดละคำสั่ง — เข้ากับ relay_bridge เดิม):
 *   PC -> MEGA : "W f f f f"  ตั้งทิศ 4 ล้อ f = F|B|S ลำดับ FL FR RL RR ("WFBSF" ก็ได้)
 *                "S" หยุดทุกล้อ | "P" ping -> "OK"
 *   MEGA -> PC : "READY humanoi_base_mega"
 *                "STATE:FBSF"                ล้อเปลี่ยนสถานะ (ลำดับ FL FR RL RR)
 *                "MODE:ROS" / "MODE:RC" / "MODE:DISARMED" / "MODE:NO_SIGNAL"  เปลี่ยนผู้คุม
 *                "WARN:timeout -> STOP"      ROS เงียบเกิน CMD_TIMEOUT
 *                "OK"
 *                "RC CH1 1500 ... | ROS | FL S FR S RL S RR S"  ทุก PRINT_MS
 *                "SW:BxDy"  ตำแหน่งสวิตช์ SWB/SWD  x,y = U (ขึ้น) | D (ลง) | - (ไม่รู้/ไม่มีสัญญาณ)
 *                           ส่งทันทีที่เปลี่ยน + ทุก PRINT_MS — Mega แค่รายงาน ไม่ตัดสินใจเอง
 *                           ฝั่ง Pi (rc_arm_trigger) จับจังหวะโยกแล้วสั่งแขนเล่นโปรแกรม/หยุด
 *
 * ความปลอดภัย:
 *   1) ต้องเห็น SWA ขึ้นก่อนหนึ่งครั้ง หลังเปิดเครื่องหรือหลังสัญญาณหาย ถึงจะ ARM ได้
 *   2) สัญญาณรีโมทหายเกิน SIGNAL_TIMEOUT_MS หรือ SWA ขึ้น -> หยุดทุกล้อทันที
 *      ตัวรับ FS-SR8 ไม่หยุดส่งพัลส์ตอนรีโมทดับ แต่ส่งค่า failsafe ~1490 ทุกช่อง
 *      -> SWA ที่ไม่ใกล้ 1000 หรือ 2000 ถือว่าไม่มีสัญญาณ
 *   3) ROS ไม่ส่ง W/S เกิน CMD_TIMEOUT -> ล้อที่ ROS สั่งหยุด
 *   4) กลับทางต้องหยุดค้าง REVERSE_DELAY ก่อน (รวมตอนสลับผู้คุมด้วย)
 *   5) relay ถูกสั่ง "ปิด" ก่อนตั้งขาเป็น OUTPUT จะได้ไม่กระตุกตอนบูต
 */

// ---------------- ค่าที่อาจต้องปรับ ----------------
const uint8_t PIN_CH1 = 2;  // สไลด์ข้าง
const uint8_t PIN_CH2 = 3;  // เดินหน้า/ถอย
const uint8_t PIN_CH4 = 5;  // หมุนตัว
const uint8_t PIN_CH5 = 6;  // SWA
const uint8_t PIN_CH6 = 7;  // SWB -> สั่งแขนเล่นโปรแกรม (ฝั่ง Pi)
const uint8_t PIN_CH8 = 9;  // SWD -> สั่งแขนหยุด (ฝั่ง Pi)
const uint8_t LED_PIN = 13;

const uint8_t NUM_WHEELS = 4;
const char WHEEL_NAME[NUM_WHEELS][3] = {"FL", "FR", "RL", "RR"};
const uint8_t PIN_FWD[NUM_WHEELS] = {22, 24, 26, 28};
const uint8_t PIN_REV[NUM_WHEELS] = {23, 25, 27, 29};
const bool INVERT[NUM_WHEELS] = {false, true, false, false};  // ล้อไหนหมุนกลับด้าน ตั้ง true

const bool RELAY_ACTIVE_LOW = true;  // โมดูล relay ของโปรเจ็คนี้ LOW = ทำงาน

// true = รีโมทปิดหรือหลุดแล้ว ROS ยังขับได้ (ไม่มีสวิตช์ฉุกเฉินเลย) — เปิดเฉพาะตอนมั่นใจ
const bool ALLOW_ROS_WITHOUT_RC = false;

const float ENGAGE = 0.35;        // ค่ารวมของล้อต้องเกินนี้ถึงหมุน (คันโยก 0-1 จากกลางถึงสุด)
const float RELEASE = 0.20;       // หมุนอยู่แล้ว ลดต่ำกว่านี้ถึงหยุด (กันกระพริบตรงขอบ)
const float STICK_ACTIVE = 0.15;  // คันโยกตัวไหนเกินนี้ = คนจับรีโมทอยู่ -> รีโมทคุม
const int SWA_DOWN_MAX = 1250;    // SWA ลง (~1000) ต้องต่ำกว่านี้
const int SWA_UP_MIN = 1750;      // SWA ขึ้น (~2000) ต้องสูงกว่านี้ ระหว่างกลาง = failsafe
const unsigned long RC_HOLD_MS = 700;         // ปล่อยคันโยกแล้วรอเท่านี้ก่อนคืนให้ ROS
const unsigned long CMD_TIMEOUT = 500;        // ROS เงียบเกินนี้ (ms) = หยุด
const unsigned long REVERSE_DELAY = 500;      // หยุดค้างก่อนกลับทาง (ms)
const unsigned long SIGNAL_TIMEOUT_MS = 400;  // ไม่มีพัลส์ถูกต้องเกินนี้ = ไม่มีสัญญาณ
const unsigned long PULSE_TIMEOUT_US = 60000; // ต้องนานกว่าหนึ่งเฟรม (ดู rc_mecanum_mega)
const unsigned long PRINT_MS = 500;
const uint8_t SWITCH_EVERY = 3;   // อ่าน SWB/SWD ทุก 3 รอบ — สวิตช์ไม่ต้องเร็ว ไม่ให้คันโยกล้อตอบช้าลง
// ---------------------------------------------------

enum Dir : int8_t { D_REV = -1, D_STOP = 0, D_FWD = 1 };
enum Mode : uint8_t { M_NO_SIGNAL, M_DISARMED, M_RC, M_ROS };

Dir current[NUM_WHEELS] = {D_STOP, D_STOP, D_STOP, D_STOP};
Dir target[NUM_WHEELS] = {D_STOP, D_STOP, D_STOP, D_STOP};
Dir rosTarget[NUM_WHEELS] = {D_STOP, D_STOP, D_STOP, D_STOP};
unsigned long stoppedAt[NUM_WHEELS] = {0, 0, 0, 0};
bool stateChanged = false;

bool armed = false;
bool sawDisarmed = false;  // safety latch
unsigned long lastGoodSignal = 0;
unsigned long lastStickActive = 0;
unsigned long lastCmdTime = 0;
unsigned long lastPrint = 0;
int ch1 = 0, ch2 = 0, ch4 = 0, ch5 = 0, ch6 = 0, ch8 = 0;
float strafe = 0, drive = 0, rotate = 0;  // ค่าคันโยกล่าสุดที่อ่านได้ถูกต้อง
char swB = '-', swD = '-';                // ตำแหน่งสวิตช์ที่รายงานล่าสุด
uint8_t loopCount = 0;

Mode mode = M_NO_SIGNAL;
bool modeReported = false;

char buf[24];
uint8_t bufLen = 0;

void relayWrite(uint8_t pin, bool on) {
  digitalWrite(pin, (on != RELAY_ACTIVE_LOW) ? HIGH : LOW);
}

void applyWheel(uint8_t i, Dir d) {
  Dir wired = INVERT[i] ? (Dir)(-d) : d;
  relayWrite(PIN_FWD[i], false);  // ปิดทั้งคู่ก่อน แล้วค่อยเปิดตัวที่ต้องการ
  relayWrite(PIN_REV[i], false);
  if (wired == D_FWD) relayWrite(PIN_FWD[i], true);
  if (wired == D_REV) relayWrite(PIN_REV[i], true);
  if (current[i] != D_STOP && d == D_STOP) stoppedAt[i] = millis();
  if (current[i] != d) stateChanged = true;
  current[i] = d;
}

// ทุกล้อผ่าน STOP ก่อนกลับทาง และรอ REVERSE_DELAY นับจากตอนหยุด
void updateWheel(uint8_t i, unsigned long now) {
  if (target[i] == current[i]) return;
  if (current[i] != D_STOP) {
    applyWheel(i, D_STOP);
  } else if (now - stoppedAt[i] >= REVERSE_DELAY) {
    applyWheel(i, target[i]);
  }
}

char dirChar(Dir d) {
  return d == D_FWD ? 'F' : (d == D_REV ? 'B' : 'S');
}

void reportState() {
  Serial.print(F("STATE:"));
  for (uint8_t i = 0; i < NUM_WHEELS; i++) Serial.print(dirChar(current[i]));
  Serial.println();
}

void reportMode(Mode m) {
  switch (m) {
    case M_NO_SIGNAL: Serial.println(F("MODE:NO_SIGNAL")); break;
    case M_DISARMED: Serial.println(F("MODE:DISARMED")); break;
    case M_RC: Serial.println(F("MODE:RC")); break;
    case M_ROS: Serial.println(F("MODE:ROS")); break;
  }
}

// ---------- คำสั่งจาก ROS ----------
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
    for (uint8_t i = 0; i < NUM_WHEELS; i++) rosTarget[i] = D_STOP;
    lastCmdTime = millis();
    return;
  }
  if (s[0] == 'W' || s[0] == 'w') {
    Dir tmp[NUM_WHEELS];
    uint8_t w = 0;
    for (uint8_t k = 1; k < n && w < NUM_WHEELS; k++) {
      if (s[k] == ' ') continue;
      if (s[k] != 'F' && s[k] != 'f' && s[k] != 'B' && s[k] != 'b'
          && s[k] != 'S' && s[k] != 's') return;   // อักขระแปลก -> ทิ้งทั้งบรรทัด
      tmp[w++] = parseDir(s[k]);
    }
    if (w != NUM_WHEELS) return;                    // ไม่ครบ 4 -> ทิ้ง
    for (uint8_t i = 0; i < NUM_WHEELS; i++) rosTarget[i] = tmp[i];
    lastCmdTime = millis();
  }
}

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (bufLen > 0) { buf[bufLen] = '\0'; handleLine(buf, bufLen); bufLen = 0; }
    } else if (bufLen < sizeof(buf) - 1) {
      buf[bufLen++] = c;
    }
  }
}

// ---------- รีโมท ----------
// pulseIn รอได้ถึง 60 ms ต่อช่อง เลยอ่าน serial ทุกครั้งก่อนรอ ไม่ให้บัฟเฟอร์ล้น
int readPulse(uint8_t pin) {
  pollSerial();
  return pulseIn(pin, HIGH, PULSE_TIMEOUT_US);
}

// พัลส์ 1000-2000 us -> -1..1, ค่าผิดช่วงทำให้ *ok = false
float stickValue(int us, bool *ok) {
  if (us < 900 || us > 2100) {
    *ok = false;
    return 0;
  }
  float v = (us - 1500) / 500.0;
  return v > 1 ? 1 : (v < -1 ? -1 : v);
}

// ค่ารวมของล้อ -> ทิศ พร้อม hysteresis เทียบกับทิศที่หมุนอยู่
Dir toDir(float v, Dir now) {
  if (now == D_FWD && v > RELEASE) return D_FWD;
  if (now == D_REV && v < -RELEASE) return D_REV;
  if (v > ENGAGE) return D_FWD;
  if (v < -ENGAGE) return D_REV;
  return D_STOP;
}

void updateLed(unsigned long now) {
  bool on;
  switch (mode) {
    case M_RC: on = true; break;
    case M_ROS: on = (now % 1000) < 900; break;
    case M_DISARMED: on = (now / 500) % 2; break;
    default: on = (now / 120) % 2; break;
  }
  digitalWrite(LED_PIN, on ? HIGH : LOW);
}

// สวิตช์ 2 ตำแหน่ง: ~1000 = ลง, ~2000 = ขึ้น, แถว 1500 = failsafe ตอนรีโมทดับ -> ไม่รู้
char switchPos(int us) {
  if (us >= 900 && us <= SWA_DOWN_MAX) return 'D';
  if (us >= SWA_UP_MIN && us <= 2100) return 'U';
  return '-';
}

void reportSwitches() {
  Serial.print(F("SW:B"));
  Serial.print(swB);
  Serial.print('D');
  Serial.println(swD);
}

void printStatus() {
  Serial.print(F("RC CH1 ")); Serial.print(ch1);
  Serial.print(F(" CH2 ")); Serial.print(ch2);
  Serial.print(F(" CH4 ")); Serial.print(ch4);
  Serial.print(F(" CH5 ")); Serial.print(ch5);
  Serial.print(F(" CH6 ")); Serial.print(ch6);
  Serial.print(F(" CH8 ")); Serial.print(ch8);
  switch (mode) {
    case M_NO_SIGNAL: Serial.print(F(" | NO SIGNAL |")); break;
    case M_DISARMED: Serial.print(sawDisarmed ? F(" | disarmed  |") : F(" | SWA up first|")); break;
    case M_RC: Serial.print(F(" | RC        |")); break;
    case M_ROS: Serial.print(F(" | ROS       |")); break;
  }
  for (uint8_t i = 0; i < NUM_WHEELS; i++) {
    Serial.print(' ');
    Serial.print(WHEEL_NAME[i]);
    Serial.print(' ');
    Serial.print(dirChar(current[i]));
  }
  Serial.println();
}

void setup() {
  for (uint8_t i = 0; i < NUM_WHEELS; i++) {
    relayWrite(PIN_FWD[i], false);  // ตั้งระดับ "ปิด" ก่อน pinMode จะได้ไม่กระตุก
    relayWrite(PIN_REV[i], false);
    pinMode(PIN_FWD[i], OUTPUT);
    pinMode(PIN_REV[i], OUTPUT);
  }
  pinMode(LED_PIN, OUTPUT);
  pinMode(PIN_CH1, INPUT);
  pinMode(PIN_CH2, INPUT);
  pinMode(PIN_CH4, INPUT);
  pinMode(PIN_CH5, INPUT);
  pinMode(PIN_CH6, INPUT);
  pinMode(PIN_CH8, INPUT);
  Serial.begin(115200);
  delay(300);
  Serial.println(F("READY humanoi_base_mega"));
}

void loop() {
  // อ่านเรียง CH1 -> CH2 -> CH4 -> CH5 ตามลำดับที่ตัวรับส่งพัลส์ (อ่าน CH5 ก่อนทำให้ CH1 หลุดบ่อย)
  // CH1 ไม่มีพัลส์ = รีโมทปิดอยู่ ข้ามช่องที่เหลือ จะได้ไม่เสียเวลารอช่องละ 60 ms
  bool ok = true;
  ch1 = readPulse(PIN_CH1);
  float s = stickValue(ch1, &ok);
  if (ok) {
    ch2 = readPulse(PIN_CH2);
    ch4 = readPulse(PIN_CH4);
    ch5 = readPulse(PIN_CH5);
    float d = stickValue(ch2, &ok);
    float r = stickValue(ch4, &ok);
    // SWA มีแค่ 2 ตำแหน่ง (~1000 / ~2000) ค่าแถว 1500 = ตัวรับส่งค่า failsafe ตอนรีโมทดับ
    // (ทดสอบแล้ว: ปิดรีโมท ทุกช่องค้าง ~1490) -> ถือว่าไม่มีสัญญาณ ไม่งั้น 1490 < 1500 จะกลายเป็น ARM
    bool swaKnown = (ch5 >= 900 && ch5 <= SWA_DOWN_MAX) || (ch5 >= SWA_UP_MIN && ch5 <= 2100);
    if (!swaKnown) ok = false;
    if (ok) { strafe = s; drive = d; rotate = r; }
    // SWB/SWD อ่านต่อท้าย (ลำดับช่องเดิม CH6 -> CH8) แต่ไม่ทุกรอบ จะได้ไม่ถ่วงคันโยกล้อ
    if (ok && ++loopCount >= SWITCH_EVERY) {
      loopCount = 0;
      ch6 = readPulse(PIN_CH6);
      ch8 = readPulse(PIN_CH8);
    }
  } else {
    ch2 = ch4 = ch5 = ch6 = ch8 = 0;
  }
  // ไม่มีสัญญาณ (หรือ SWA เป็นค่า failsafe) -> สวิตช์ทุกตัว "ไม่รู้" ฝั่ง Pi จะไม่สั่งอะไรจากค่านี้
  char b = ok ? switchPos(ch6) : '-';
  char dd = ok ? switchPos(ch8) : '-';
  if (b != swB || dd != swD) {
    swB = b;
    swD = dd;
    reportSwitches();
  }
  pollSerial();
  unsigned long now = millis();

  if (ok) lastGoodSignal = now;
  bool signal = lastGoodSignal != 0 && now - lastGoodSignal <= SIGNAL_TIMEOUT_MS;

  if (!signal) {
    armed = false;
    sawDisarmed = false;  // สัญญาณกลับมาแล้วต้องยก SWA ขึ้นก่อนหนึ่งครั้ง
  } else if (ok) {
    bool swaDown = ch5 <= SWA_DOWN_MAX;
    if (!swaDown) sawDisarmed = true;
    armed = swaDown && sawDisarmed;
  }

  // ---------- Watchdog ของ ROS ----------
  bool rosMoving = false;
  for (uint8_t i = 0; i < NUM_WHEELS; i++) if (rosTarget[i] != D_STOP) rosMoving = true;
  if (rosMoving && now - lastCmdTime > CMD_TIMEOUT) {
    for (uint8_t i = 0; i < NUM_WHEELS; i++) rosTarget[i] = D_STOP;
    Serial.println(F("WARN:timeout -> STOP"));
  }

  // ---------- เลือกผู้คุม ----------
  if (armed && (fabs(strafe) > STICK_ACTIVE || fabs(drive) > STICK_ACTIVE
                || fabs(rotate) > STICK_ACTIVE)) {
    lastStickActive = now;
  }
  bool rcHolding = lastStickActive != 0 && now - lastStickActive < RC_HOLD_MS;

  Mode m;
  if (!signal) m = ALLOW_ROS_WITHOUT_RC ? M_ROS : M_NO_SIGNAL;
  else if (!armed) m = M_DISARMED;
  else if (rcHolding) m = M_RC;
  else m = M_ROS;

  if (m != mode || !modeReported) {
    mode = m;
    modeReported = true;
    reportMode(m);
  }

  // mecanum: ล้อหน้า/หลังฝั่งเดียวกันหมุนสวนกันตอนสไลด์ข้าง
  float mix[NUM_WHEELS] = {
    drive + strafe + rotate,  // FL
    drive - strafe - rotate,  // FR
    drive - strafe + rotate,  // RL
    drive + strafe - rotate,  // RR
  };
  bool hardStop = mode == M_NO_SIGNAL || mode == M_DISARMED;
  for (uint8_t i = 0; i < NUM_WHEELS; i++) {
    if (mode == M_RC) target[i] = toDir(mix[i], current[i]);
    else if (mode == M_ROS) target[i] = rosTarget[i];
    else target[i] = D_STOP;
    if (hardStop && current[i] != D_STOP) applyWheel(i, D_STOP);  // ปลด ARM = หยุดทันที
    updateWheel(i, now);
  }
  if (stateChanged) {
    stateChanged = false;
    reportState();
  }
  updateLed(now);

  if (now - lastPrint >= PRINT_MS) {
    lastPrint = now;
    printStatus();
    reportSwitches();  // ส่งซ้ำเป็นระยะ เผื่อฝั่ง Pi เพิ่งเปิดหรือพลาดบรรทัดตอนเปลี่ยน
  }
}
