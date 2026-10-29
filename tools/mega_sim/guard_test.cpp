// ทดสอบ firmware/humanoi_base_mega/guard.h บน PC
//   ./guard_test            รันชุดทดสอบ
//   ./guard_test dirs FBSF  พิมพ์บิตทิศของชุดคำสั่งล้อ (ให้ test_guard_e2e.py เทียบกับสูตรฝั่ง relay_bridge)
#include <cstdio>
#include <cstring>

#include "../../firmware/humanoi_base_mega/guard.h"

static int fails = 0;
#define CHECK(cond)                                              \
  do {                                                           \
    if (!(cond)) {                                               \
      printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);     \
      fails++;                                                   \
    }                                                            \
  } while (0)

static void pattern(const char *s, int8_t w[4]) {
  for (int i = 0; i < 4; i++) w[i] = s[i] == 'F' ? 1 : (s[i] == 'B' ? -1 : 0);
}

static uint8_t dirs(const char *s) {
  int8_t w[4];
  pattern(s, w);
  return motionDirs(w);
}

// คันโยกรีโมท -> ทิศล้อ ตามสูตรใน humanoi_base_mega.ino (drive/strafe/rotate เต็มคันโยก)
static uint8_t rcDirs(int drive, int strafe, int rotate) {
  int mix[4] = {drive + strafe + rotate, drive - strafe - rotate, drive - strafe + rotate, drive + strafe - rotate};
  int8_t w[4];
  for (int i = 0; i < 4; i++) w[i] = mix[i] > 0 ? 1 : (mix[i] < 0 ? -1 : 0);
  return motionDirs(w);
}

int main(int argc, char **argv) {
  if (argc == 3 && !strcmp(argv[1], "dirs")) {
    printf("%u\n", dirs(argv[2]));
    return 0;
  }

  // ทิศจากชุดล้อ
  CHECK(dirs("SSSS") == 0);
  CHECK(dirs("FFFF") == G_FWD);
  CHECK(dirs("BBBB") == G_BACK);
  CHECK(dirs("BFFB") == G_LEFT);
  CHECK(dirs("FBBF") == G_RIGHT);
  CHECK(dirs("BFBF") == G_TURN);   // หมุนซ้าย
  CHECK(dirs("FBFB") == G_TURN);   // หมุนขวา
  CHECK(dirs("FSSF") == (G_FWD | G_RIGHT));  // เฉียงหน้าขวา
  CHECK(dirs("FSSS") == (G_FWD | G_RIGHT | G_TURN));  // ล้อเดียว
  CHECK(dirs("FFBB") == G_ALL);    // ล้อสู้กันเอง ทิศรวมศูนย์ = ไม่รู้

  // คันโยกรีโมท: CH2 ขึ้น = หน้า, CH1 ขวา = สไลด์ขวา, CH4 = หมุน
  CHECK(rcDirs(1, 0, 0) == G_FWD);
  CHECK(rcDirs(-1, 0, 0) == G_BACK);
  CHECK(rcDirs(0, 1, 0) == G_RIGHT);
  CHECK(rcDirs(0, -1, 0) == G_LEFT);
  CHECK(rcDirs(0, 0, 1) == G_TURN);
  CHECK(rcDirs(1, 1, 0) == (G_FWD | G_RIGHT));

  // ทุกชุด 81 แบบ: ห้ามหมด = หยุดทุกชุดที่ล้อหมุน · ไม่ห้าม = ไปได้ทุกชุด
  Guard all;
  all.onMessage(G_ALL, 0);
  Guard none;
  none.onMessage(0, 0);
  for (int n = 0; n < 81; n++) {
    int8_t w[4];
    int k = n;
    bool moving = false;
    for (int i = 0; i < 4; i++, k /= 3) {
      w[i] = (int8_t)(k % 3 - 1);
      moving |= w[i] != 0;
    }
    CHECK((all.blocks(w) != 0) == moving);
    CHECK(none.blocks(w) == 0);
  }

  // สถานะ
  Guard g;
  int8_t fwd[4], back[4];
  pattern("FFFF", fwd);
  pattern("BBBB", back);
  CHECK(g.state == GUARD_OFF && g.blocks(fwd) == 0);            // บูต: ยังไม่ได้ยิน Pi = ไม่กัน
  CHECK(!g.tick(10000, 500, true));                             // OFF ไม่หมดเวลา
  CHECK(g.onMessage(G_FWD, 1000));                              // เปลี่ยน -> รายงาน
  CHECK(!g.onMessage(G_FWD, 1050));                             // ค่าเดิม -> ไม่รายงานซ้ำ
  CHECK(g.blocks(fwd) == G_FWD && g.blocks(back) == 0);
  CHECK(!g.tick(1550, 500, true));                              // 500 ms พอดี ยังไม่หาย
  CHECK(g.tick(1551, 500, true) && g.state == GUARD_LOST);      // เงียบเกิน -> LOST
  CHECK(g.blocks(back) == G_BACK);                              // LOST ห้ามทุกทิศ
  CHECK(g.release() && g.state == GUARD_OFF);                   // SWA ปลด
  CHECK(g.blocks(fwd) == 0);
  CHECK(!g.release());                                          // OFF แล้วปลดซ้ำไม่มีผล
  CHECK(g.onMessage(0, 3000) && g.state == GUARD_ON);           // Pi กลับมา -> ON
  CHECK(!g.release() && g.state == GUARD_ON);                   // ON ปลดด้วย SWA ไม่ได้
  CHECK(g.onMessage(-1, 3100) && g.state == GUARD_OFF);         // "G OFF"
  CHECK(!g.tick(9000, 500, true));                              // OFF ไม่กลายเป็น LOST
  g.onMessage(G_TURN, 10000);
  CHECK(g.tick(10501, 500, false) && g.state == GUARD_OFF);     // GUARD_LOST_STOPS = false -> OFF
  g.onMessage(5, 20000);
  CHECK(g.tick(20000UL + 600, 500, true));

  if (fails) {
    printf("guard_test: %d FAIL\n", fails);
    return 1;
  }
  printf("guard_test: ผ่านทั้งหมด\n");
  return 0;
}
