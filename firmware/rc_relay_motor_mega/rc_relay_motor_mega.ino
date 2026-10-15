/*
 * rc_relay_motor_mega.ino
 * Arduino MEGA 2560 - FlySky FS-ST8 / FS-SR8 (PWM) คุมทิศ DC motor 2 ตัวผ่าน relay (ทดสอบล้อหน้า)
 *
 * ต่อสาย:
 *   ตัวรับ CH1..CH8 (ขา S) -> pin 2..9   (+/- ของตัวรับจาก 5V/GND ของ Mega)
 *   relay FL (หน้าซ้าย)  : เดินหน้า pin 10, ถอยหลัง pin 11
 *   relay FR (หน้าขวา)   : เดินหน้า pin 12, ถอยหลัง pin 13
 *   relay ต่อแบบเดียวกับ relay_motor_mega_4wd: COM -> ขั้วมอเตอร์, NO -> OUT+, NC -> OUT- ของ governor
 *
 * การบังคับ (ค่าที่วัดจากรีโมทจริง: คันโยก 1000-2000 กลาง 1500, SWA ขึ้น 2000 / ลง 1000):
 *   SWA (CH5, pin 6) ลง = ARM, ขึ้น = หยุดทุกล้อ
 *   คันโยกขวา ขึ้น/ลง (CH2, pin 3) = เดินหน้า / ถอยหลัง
 *   คันโยกขวา ซ้าย/ขวา (CH1, pin 2) = เลี้ยว (หมุนตัว: ล้อซ้ายถอย ล้อขวาเดิน และกลับกัน)
 *   relay เปิดปิดได้อย่างเดียว ความเร็วยังตั้งด้วยลูกบิด governor เหมือนเดิม
 *
 * ความปลอดภัย:
 *   1) ต้องเห็น SWA อยู่ข้างบน (ไม่ ARM) ก่อนหนึ่งครั้งหลังเปิดเครื่องหรือหลังสัญญาณหาย ถึงจะ ARM ได้
 *   2) สัญญาณหายเกิน SIGNAL_TIMEOUT_MS -> หยุดทุกล้อ (ที่รีโมทให้ FAILSAFE เป็น NOT SET / NONE = ไม่มีพัลส์)
 *   3) จะกลับทาง ต้องหยุดค้าง REVERSE_DELAY ก่อน, ไม่เปิด relay สองตัวของล้อเดียวกันพร้อมกัน
 *   4) relay ถูกสั่ง "ปิด" ก่อนตั้งขาเป็น OUTPUT จะได้ไม่กระตุกตอนบูต
 *   !! pin 13 = LED บนบอร์ด, bootloader กะพริบตอนรีเซ็ต/อัปโหลด -> relay FR ถอยหลังอาจกระตุก
 *      ถอดไฟมอเตอร์ตอนอัปโหลด หรือย้าย FR ถอยหลังไปขาอื่น (เช่น 23) แล้วแก้ PIN_REV
 *
 * Serial 115200 (ทุก 250 ms): "CH1 1500 CH2 1500 CH5 2000 | ARMED | FL S FR S"
 */

// ---------------- ค่าที่อาจต้องปรับ ----------------
const uint8_t PIN_CH1 = 2;  // เลี้ยว
const uint8_t PIN_CH2 = 3;  // เดินหน้า/ถอย
const uint8_t PIN_CH5 = 6;  // SWA

const uint8_t NUM_MOTORS = 2;
const char *MOTOR_NAME[NUM_MOTORS] = {"FL", "FR"};
const uint8_t PIN_FWD[NUM_MOTORS] = {10, 12};
const uint8_t PIN_REV[NUM_MOTORS] = {11, 13};
const bool INVERT[NUM_MOTORS] = {false, false};  // ล้อไหนหมุนกลับด้าน ตั้ง true

const bool RELAY_ACTIVE_LOW = true;  // โมดูล relay ของโปรเจ็คนี้ LOW = ทำงาน

const float ENGAGE = 0.35;   // คันโยกต้องเกินนี้ล้อถึงหมุน (0-1 จากกลางถึงสุด)
const float RELEASE = 0.20;  // หมุนอยู่แล้ว ลดต่ำกว่านี้ถึงหยุด (กันกระพริบตรงขอบ)
const unsigned long REVERSE_DELAY = 500;      // หยุดค้างก่อนกลับทาง (ms)
const unsigned long SIGNAL_TIMEOUT_MS = 300;  // ไม่มีพัลส์ถูกต้องเกินนี้ = หยุด
// ตัวรับส่งพัลส์ทุกช่องเกือบพร้อมกันต่อเฟรม อ่านช่องหนึ่งเสร็จ ช่องถัดไปต้องรอเฟรมใหม่
// เลยต้องรอได้นานกว่าหนึ่งเฟรมเผื่อไว้ (25 ms เคยสั้นเกิน อ่านได้สลับช่องไม่เคยครบ)
const unsigned long PULSE_TIMEOUT_US = 60000;
const unsigned long PRINT_MS = 250;
// ---------------------------------------------------

enum Dir : int8_t { D_REV = -1, D_STOP = 0, D_FWD = 1 };

Dir current[NUM_MOTORS] = {D_STOP, D_STOP};
Dir target[NUM_MOTORS] = {D_STOP, D_STOP};
unsigned long stoppedAt[NUM_MOTORS] = {0, 0};

bool armed = false;
bool sawDisarmed = false;  // safety latch
unsigned long lastGoodSignal = 0;
unsigned long lastPrint = 0;
int ch1 = 0, ch2 = 0, ch5 = 0;

void relayWrite(uint8_t pin, bool on) {
  digitalWrite(pin, (on != RELAY_ACTIVE_LOW) ? HIGH : LOW);
}

void applyMotor(uint8_t i, Dir d) {
  Dir wired = INVERT[i] ? (Dir)(-d) : d;
  relayWrite(PIN_FWD[i], false);  // ปิดทั้งคู่ก่อน แล้วค่อยเปิดตัวที่ต้องการ
  relayWrite(PIN_REV[i], false);
  if (wired == D_FWD) relayWrite(PIN_FWD[i], true);
  if (wired == D_REV) relayWrite(PIN_REV[i], true);
  if (current[i] != D_STOP && d == D_STOP) stoppedAt[i] = millis();
  current[i] = d;
}

// ทุกล้อผ่าน STOP ก่อนกลับทาง และรอ REVERSE_DELAY นับจากตอนหยุด
void updateMotor(uint8_t i, unsigned long now) {
  if (target[i] == current[i]) return;
  if (current[i] != D_STOP) {
    applyMotor(i, D_STOP);
  } else if (now - stoppedAt[i] >= REVERSE_DELAY) {
    applyMotor(i, target[i]);
  }
}

// พัลส์ 1000-2000 us -> -1..1, ค่าผิดช่วง = 0 ใน *ok
float stickValue(int us, bool *ok) {
  if (us < 900 || us > 2100) {
    *ok = false;
    return 0;
  }
  float v = (us - 1500) / 500.0;
  return v > 1 ? 1 : (v < -1 ? -1 : v);
}

// แปลงค่าคันโยกเป็นทิศ พร้อม hysteresis เทียบกับทิศที่หมุนอยู่
Dir toDir(float v, Dir now) {
  if (now == D_FWD && v > RELEASE) return D_FWD;
  if (now == D_REV && v < -RELEASE) return D_REV;
  if (v > ENGAGE) return D_FWD;
  if (v < -ENGAGE) return D_REV;
  return D_STOP;
}

char dirChar(Dir d) {
  return d == D_FWD ? 'F' : (d == D_REV ? 'B' : 'S');
}

void setup() {
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    relayWrite(PIN_FWD[i], false);  // ตั้งระดับ "ปิด" ก่อน pinMode จะได้ไม่กระตุก
    relayWrite(PIN_REV[i], false);
    pinMode(PIN_FWD[i], OUTPUT);
    pinMode(PIN_REV[i], OUTPUT);
  }
  pinMode(PIN_CH1, INPUT);
  pinMode(PIN_CH2, INPUT);
  pinMode(PIN_CH5, INPUT);
  Serial.begin(115200);
  Serial.println(F("READY rc_relay_motor_mega (flip SWA up, then down to ARM)"));
}

void loop() {
  ch1 = pulseIn(PIN_CH1, HIGH, PULSE_TIMEOUT_US);
  ch2 = pulseIn(PIN_CH2, HIGH, PULSE_TIMEOUT_US);
  ch5 = pulseIn(PIN_CH5, HIGH, PULSE_TIMEOUT_US);
  unsigned long now = millis();

  bool ok = true;
  float drive = stickValue(ch2, &ok);
  float turn = stickValue(ch1, &ok);
  bool swaValid = ch5 >= 900 && ch5 <= 2100;
  if (ok && swaValid) lastGoodSignal = now;
  bool signal = lastGoodSignal != 0 && now - lastGoodSignal <= SIGNAL_TIMEOUT_MS;

  if (!signal) {
    armed = false;
    sawDisarmed = false;  // สัญญาณกลับมาแล้วต้องยก SWA ขึ้นก่อนหนึ่งครั้ง
  } else if (ok && swaValid) {
    bool swaDown = ch5 < 1500;
    if (!swaDown) sawDisarmed = true;
    armed = swaDown && sawDisarmed;
  }

  float mix[NUM_MOTORS] = {drive + turn, drive - turn};  // FL, FR
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    target[i] = armed ? toDir(mix[i], current[i]) : D_STOP;
    if (!armed && current[i] != D_STOP) applyMotor(i, D_STOP);  // ปลด ARM = หยุดทันที
    updateMotor(i, now);
  }

  if (now - lastPrint >= PRINT_MS) {
    lastPrint = now;
    Serial.print(F("CH1 ")); Serial.print(ch1);
    Serial.print(F(" CH2 ")); Serial.print(ch2);
    Serial.print(F(" CH5 ")); Serial.print(ch5);
    Serial.print(signal ? (armed ? F(" | ARMED   |") : (sawDisarmed ? F(" | disarmed|") : F(" | SWA up first|")))
                        : F(" | NO SIGNAL|"));
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      Serial.print(' ');
      Serial.print(MOTOR_NAME[i]);
      Serial.print(' ');
      Serial.print(dirChar(current[i]));
    }
    Serial.println();
  }
}
