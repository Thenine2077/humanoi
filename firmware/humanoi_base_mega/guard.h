// guard.h — กันชน lidar ชั้น Mega: Pi (relay_bridge) บอกทิศที่มีของขวางทุก 50 ms ("G <บิต>")
// Mega ไม่ขับไปทิศนั้น ไม่ว่าคำสั่งมาจากรีโมทหรือ ROS
// แยกไฟล์ไม่ใช้ Arduino API — คอมไพล์ทดสอบบน PC ได้ (tools/mega_sim)
#pragma once
#include <stdint.h>

// บิตทิศ — ตรงกับ ros2_ws/src/humanoi_base/humanoi_base/guard.py
const uint8_t G_FWD = 1, G_BACK = 2, G_LEFT = 4, G_RIGHT = 8, G_TURN = 16, G_ALL = 31;

// ทิศที่ชุดคำสั่งล้อ (FL FR RL RR: +1 หน้า / -1 ถอย / 0 หยุด) พาตัวหุ่นไป
// สูตร mecanum เดียวกับ relay_bridge._body_twist (หน้า = +x, ซ้าย = +y, หมุนทวนเข็ม = +)
inline uint8_t motionDirs(const int8_t w[4]) {
  int vx = w[0] + w[1] + w[2] + w[3];
  int vy = -w[0] + w[1] + w[2] - w[3];
  int wz = -w[0] + w[1] - w[2] + w[3];
  uint8_t m = 0;
  if (vx > 0) m |= G_FWD;
  if (vx < 0) m |= G_BACK;
  if (vy > 0) m |= G_LEFT;
  if (vy < 0) m |= G_RIGHT;
  if (wz != 0) m |= G_TURN;
  if (!m && (w[0] || w[1] || w[2] || w[3])) m = G_ALL;  // ล้อหมุนแต่ทิศรวมเป็นศูนย์ (ล้อสู้กันเอง) = ไม่รู้ไปทางไหน
  return m;
}

enum GuardState : uint8_t {
  GUARD_OFF,   // ไม่กัน: ยังไม่เคยได้ยิน Pi หลังบูต / Pi สั่ง "G OFF" / คนปลดด้วย SWA ตอน LOST
  GUARD_ON,    // Pi ส่งมาเรื่อยๆ — ห้ามตาม mask
  GUARD_LOST,  // เคยได้ยินแล้วเงียบเกิน timeout (Pi/relay_bridge ดับ, สายหลุด) — ห้ามทุกทิศ
};

struct Guard {
  GuardState state = GUARD_OFF;
  uint8_t mask = 0;
  unsigned long last = 0;

  // n = 0..31 จาก "G n" · n < 0 = "G OFF" — คืน true ถ้าสถานะหรือ mask เปลี่ยน (ควรรายงาน)
  bool onMessage(int n, unsigned long now) {
    GuardState s = n < 0 ? GUARD_OFF : GUARD_ON;
    uint8_t m = n < 0 ? 0 : (uint8_t)(n & G_ALL);
    bool changed = s != state || m != mask;
    state = s;
    mask = m;
    last = now;
    return changed;
  }

  // เรียกทุกรอบ loop — คืน true ถ้าเพิ่งหาย (ON -> LOST หรือ OFF ถ้า lostStops = false)
  bool tick(unsigned long now, unsigned long timeout, bool lostStops) {
    if (state != GUARD_ON || now - last <= timeout) return false;
    state = lostStops ? GUARD_LOST : GUARD_OFF;
    mask = 0;
    return true;
  }

  // SWA โยกขึ้นตอน LOST = คนยืนยันว่าจะขับต่อโดยไม่มี lidar -> OFF (Pi กลับมาส่งเมื่อไรก็กลับเป็น ON)
  bool release() {
    if (state != GUARD_LOST) return false;
    state = GUARD_OFF;
    return true;
  }

  uint8_t effective() const {
    return state == GUARD_ON ? mask : (state == GUARD_LOST ? G_ALL : 0);
  }

  // ทิศของชุดคำสั่งล้อที่ถูกห้าม (0 = ไปได้)
  uint8_t blocks(const int8_t w[4]) const {
    return motionDirs(w) & effective();
  }
};
