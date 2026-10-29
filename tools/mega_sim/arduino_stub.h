// Arduino API ปลอมพอให้เฟิร์มแวร์ humanoi_base_mega คอมไพล์และรันบน Linux ได้ (ใช้กับ mega_sim.cpp)
// Serial = pty · millis = นาฬิกาจริง · pulseInLong = ค่ารีโมทจากไฟล์ · digitalWrite = จำสถานะขาไว้
#pragma once
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <unistd.h>

typedef uint8_t byte;
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define F(s) (s)

#define A8 62
#define A9 63
#define A10 64
#define A11 65
#define A12 66
#define A13 67
#define A14 68
#define A15 69

// encoder อยู่ port K — ในซิมไม่มีพัลส์ encoder (ค่าคงที่) ISR ไม่ถูกเรียก
inline uint8_t PINK = 0xFF;
inline uint8_t PCMSK2 = 0, PCICR = 0, PCIFR = 0;
#define PCIF2 2
#define PCIE2 2
#define _BV(b) (1 << (b))
#define ISR(v) void v()
#define PCINT2_vect sim_pcint2_isr
inline uint8_t digitalPinToPort(uint8_t) { return 11; }
inline volatile uint8_t *portInputRegister(uint8_t) { return &PINK; }
inline uint8_t digitalPinToBitMask(uint8_t pin) { return (uint8_t)(1 << ((pin - A8) & 7)); }
inline void noInterrupts() {}
inline void interrupts() {}

namespace sim {
inline int pins[80];
inline int rc[80];  // ความกว้างพัลส์ (us) ต่อขา — 0 = ไม่มีพัลส์
inline std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
void onPulseRead(uint8_t pin);  // mega_sim.cpp: โหลดค่ารีโมทใหม่ตอนอ่าน CH1
void onSerialWrite(const char *s, size_t n);
int serialRead();               // -1 = ไม่มีข้อมูล
}  // namespace sim

inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t pin, uint8_t v) { sim::pins[pin] = v; }
inline unsigned long millis() {
  return (unsigned long)std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now() - sim::t0).count();
}
inline unsigned long micros() { return millis() * 1000UL; }
inline void delay(unsigned long ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

// ตัวรับจริงส่งพัลส์ทุก ~20 ms ต่อช่อง — หน่วงพอให้รอบ loop ใกล้ของจริง ไม่มีพัลส์ = รอนานกว่า (timeout)
inline unsigned long pulseInLong(uint8_t pin, uint8_t, unsigned long) {
  sim::onPulseRead(pin);
  int v = sim::rc[pin];
  std::this_thread::sleep_for(std::chrono::milliseconds(v ? 4 : 20));
  return (unsigned long)v;
}

class SimSerial {
 public:
  void begin(unsigned long) {}
  int available() {
    if (peek_ < 0) peek_ = sim::serialRead();
    return peek_ >= 0;
  }
  int read() {
    if (peek_ < 0) peek_ = sim::serialRead();
    int c = peek_;
    peek_ = -1;
    return c;
  }
  void print(const char *s) { sim::onSerialWrite(s, strlen(s)); }
  void print(char c) { sim::onSerialWrite(&c, 1); }
  void print(int v) { num("%d", v); }
  void print(unsigned int v) { num("%u", v); }
  void print(long v) { num("%ld", v); }
  void print(unsigned long v) { num("%lu", v); }
  void println() { sim::onSerialWrite("\r\n", 2); }
  template <typename T> void println(T v) { print(v); println(); }

 private:
  int peek_ = -1;
  template <typename T> void num(const char *fmt, T v) {
    char b[24];
    int n = snprintf(b, sizeof b, fmt, v);
    sim::onSerialWrite(b, (size_t)n);
  }
};
inline SimSerial Serial;
