// Mega จำลองบน Linux — คอมไพล์เฟิร์มแวร์ firmware/humanoi_base_mega ตัวจริง (ไม่แก้สักบรรทัด) กับ arduino_stub.h
//   Serial  = pty: relay_bridge เปิดพอร์ตนี้ได้เหมือน Mega จริง (ชื่อพอร์ตเขียนลงไฟล์ <port_file>)
//   รีโมท   = ไฟล์ <rc_file> บรรทัดเดียว "ch1 ch2 ch4 ch5 ch6 ch8" (us เช่น 1500 1500 1500 1000 1000 1000)
//             ไม่มีไฟล์ / ว่าง = รีโมทปิด (ไม่มีพัลส์)
//   log     = stdout ทุกบรรทัดที่ Mega ส่ง (ยกเว้น ENC และบรรทัดสถานะ RC CH1 / SW: ถ้าไม่ใส่ -v) นำหน้าด้วยเวลา ms
// build: g++ -std=c++17 -O1 -Wall -Wextra -I tools/mega_sim tools/mega_sim/mega_sim.cpp -o mega_sim
// ใช้:   ./mega_sim /tmp/mega_port /tmp/mega_rc [-v]
#include "arduino_stub.h"

#include <fcntl.h>
#include <termios.h>

#include <cstdlib>
#include <fstream>
#include <sstream>

#include "../../firmware/humanoi_base_mega/humanoi_base_mega.ino"

namespace sim {
static int master = -1;
static std::string rcFile;
static bool verbose = false;
static std::string outLine;

void onPulseRead(uint8_t pin) {
  if (pin != PIN_CH1) return;  // อ่านไฟล์รอบละครั้ง (เฟิร์มแวร์อ่าน CH1 ก่อนทุกรอบ)
  int v[6] = {0, 0, 0, 0, 0, 0};
  std::ifstream f(rcFile);
  for (int i = 0; i < 6 && f >> v[i]; i++) {}
  const uint8_t p[6] = {PIN_CH1, PIN_CH2, PIN_CH4, PIN_CH5, PIN_CH6, PIN_CH8};
  for (int i = 0; i < 6; i++) rc[p[i]] = v[i];
}

void onSerialWrite(const char *s, size_t n) {
  if (master >= 0) {
    ssize_t w = write(master, s, n);  // ยังไม่มีใครเปิดพอร์ต = บัฟเฟอร์เต็มแล้วทิ้ง เหมือน USB ไม่มีคนอ่าน
    (void)w;
  }
  for (size_t i = 0; i < n; i++) {
    if (s[i] == '\r') continue;
    if (s[i] != '\n') {
      outLine += s[i];
      continue;
    }
    bool noisy = outLine.rfind("ENC ", 0) == 0 || outLine.rfind("RC CH1", 0) == 0 || outLine.rfind("SW:", 0) == 0;
    if (verbose || !noisy) {
      printf("%8lu %s\n", millis(), outLine.c_str());
      fflush(stdout);
    }
    outLine.clear();
  }
}

int serialRead() {
  unsigned char c;
  return read(master, &c, 1) == 1 ? c : -1;
}
}  // namespace sim

int main(int argc, char **argv) {
  if (argc < 3) {
    fprintf(stderr, "ใช้: %s <port_file> <rc_file> [-v]\n", argv[0]);
    return 2;
  }
  sim::rcFile = argv[2];
  sim::verbose = argc > 3 && std::string(argv[3]) == "-v";
  sim::master = posix_openpt(O_RDWR | O_NOCTTY);
  if (sim::master < 0 || grantpt(sim::master) || unlockpt(sim::master)) {
    perror("pty");
    return 1;
  }
  const char *slaveName = ptsname(sim::master);
  int slave = open(slaveName, O_RDWR | O_NOCTTY);  // เปิดค้างไว้ pty จะไม่ปิดตอน relay_bridge ปิด/เปิดใหม่
  termios t;
  tcgetattr(slave, &t);
  cfmakeraw(&t);
  tcsetattr(slave, TCSANOW, &t);
  fcntl(sim::master, F_SETFL, O_NONBLOCK);
  std::ofstream(argv[1]) << slaveName;
  fprintf(stderr, "mega_sim: พอร์ต %s · รีโมทจาก %s\n", slaveName, argv[2]);
  setup();
  for (;;) loop();
}
