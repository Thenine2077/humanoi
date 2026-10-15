/*
 * encoder_test_mega.ino
 * Arduino MEGA 2560 - อ่าน encoder 2 ตัว (YT3806 / 3806 series, 360 P/R, NPN open collector)
 *
 * ต่อสาย (สมมติว่าเป็นคู่ เขียว 51 + ขาว 50 และ เขียว 53 + ขาว 52):
 *   encoder    แดง      ดำ      เขียว (A)   ขาว (B)
 *   E1         +V       GND     pin 51      pin 50
 *   E2         +V       GND     pin 53      pin 52
 *   GND ของ encoder ต้องต่อร่วมกับ GND ของ Mega
 *   ขา 50-53 อยู่บน PCINT0 ใช้ interrupt จับทุกขอบได้ (x4 = 1440 นับต่อรอบ)
 *   เปิด pull-up ภายในไว้แล้ว ถ้าสายยาวหรือนับเพี้ยน ใส่ตัวต้านทาน 2.2k-4.7k จากขา A/B ไป 5V
 *
 * Serial 115200 (ทุก 200 ms):
 *   "E1 1234 cnt  12.5 rpm  0 err | E2 -56 cnt  -0.6 rpm  0 err"
 *   err = จังหวะที่ A และ B เปลี่ยนพร้อมกัน (นับไม่ทัน หรือสัญญาณรบกวน) ควรเป็น 0
 * คำสั่ง: r = ล้างค่านับ
 */

const uint8_t NUM_ENC = 2;
const uint8_t PIN_A[NUM_ENC] = {51, 53};  // สายเขียว
const uint8_t PIN_B[NUM_ENC] = {50, 52};  // สายขาว
const bool INVERT[NUM_ENC] = {false, false};  // ตัวไหนนับกลับด้าน ตั้ง true
const long COUNTS_PER_REV = 360L * 4;  // 360 P/R อ่านครบทั้ง 4 ขอบ
const unsigned long PRINT_MS = 200;

// ค่าเปลี่ยนตามสถานะ (ก่อน<<2 | หลัง), สถานะ = A<<1 | B
const int8_t QDEC[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

uint8_t maskA[NUM_ENC], maskB[NUM_ENC];
volatile long count[NUM_ENC];
volatile uint16_t errors[NUM_ENC];
volatile uint8_t prevState[NUM_ENC];

uint8_t readState(uint8_t pins, uint8_t i) {
  return ((pins & maskA[i]) ? 2 : 0) | ((pins & maskB[i]) ? 1 : 0);
}

ISR(PCINT0_vect) {
  uint8_t pins = PINB;  // ขา 50-53 อยู่บน port B ทั้งหมด อ่านทีเดียว
  for (uint8_t i = 0; i < NUM_ENC; i++) {
    uint8_t s = readState(pins, i);
    uint8_t p = prevState[i];
    if (s == p) continue;
    if ((s ^ p) == 3) errors[i]++;  // A และ B เปลี่ยนพร้อมกัน = พลาดไปหนึ่งขอบ
    count[i] += QDEC[(p << 2) | s];
    prevState[i] = s;
  }
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < NUM_ENC; i++) {
    if (portInputRegister(digitalPinToPort(PIN_A[i])) != &PINB ||
        portInputRegister(digitalPinToPort(PIN_B[i])) != &PINB) {
      Serial.println(F("ERROR: encoder pins must be on port B (Mega 10-13, 50-53)"));
      while (true) {}
    }
    pinMode(PIN_A[i], INPUT_PULLUP);
    pinMode(PIN_B[i], INPUT_PULLUP);
    maskA[i] = digitalPinToBitMask(PIN_A[i]);
    maskB[i] = digitalPinToBitMask(PIN_B[i]);
    prevState[i] = readState(PINB, i);
    PCMSK0 |= maskA[i] | maskB[i];  // บิตใน PCMSK0 ตรงกับบิตของ port B
  }
  PCIFR |= _BV(PCIF0);
  PCICR |= _BV(PCIE0);
  Serial.println(F("READY encoder_test_mega (r = reset counts)"));
}

void loop() {
  static unsigned long lastPrint = 0;
  static long lastCount[NUM_ENC];

  if (Serial.available() && Serial.read() == 'r') {
    noInterrupts();
    for (uint8_t i = 0; i < NUM_ENC; i++) count[i] = errors[i] = 0;
    interrupts();
    for (uint8_t i = 0; i < NUM_ENC; i++) lastCount[i] = 0;
  }

  unsigned long now = millis();
  if (now - lastPrint < PRINT_MS) return;
  float dt = (now - lastPrint) / 1000.0;
  lastPrint = now;

  for (uint8_t i = 0; i < NUM_ENC; i++) {
    noInterrupts();
    long c = count[i];
    uint16_t e = errors[i];
    interrupts();
    if (INVERT[i]) c = -c;
    float rpm = (c - lastCount[i]) / dt * 60.0 / COUNTS_PER_REV;
    lastCount[i] = c;
    if (i) Serial.print(F(" | "));
    Serial.print('E'); Serial.print(i + 1); Serial.print(' ');
    Serial.print(c); Serial.print(F(" cnt  "));
    Serial.print(rpm, 1); Serial.print(F(" rpm  "));
    Serial.print(e); Serial.print(F(" err"));
  }
  Serial.println();
}
