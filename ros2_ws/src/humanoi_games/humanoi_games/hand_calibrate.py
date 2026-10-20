#!/usr/bin/env python3
"""hand_calibrate — ตั้งค่ากลางนิ้ว (MiddlePos) ของมือ AmazingHand ทีละนิ้วด้วยตา แล้วบันทึกให้เกมใช้

  arm hand-calibrate                         (บน Pi — ปิดเกมก่อน: พอร์ตมือเปิดได้ทีละโปรแกรม)
  ros2 run humanoi_games hand_calibrate

นิ้วที่กำลังตั้งจะยืดตรง (มุม 0) นิ้วอื่นกำไว้ไม่ให้บัง — ปรับจนนิ้วตั้งตรงแนวเดียวกับฝ่ามือ:
  l / r         เอียงนิ้วไปทางหนึ่ง / อีกทาง 1°   (l3 = 3°) — servo สองตัวของนิ้วเลื่อนไปทางเดียวกัน
  f / b         งอไปหน้า (ทางฝ่ามือ) / ไปหลัง 1°  (f3 = 3°) — servo สองตัวเลื่อนสวนกัน
  1+ 1- 2+ 2-   ขยับ servo ตัวที่ 1 / 2 ของนิ้วนี้ทีละตัว (แบบ AmazingHand\\tools\\calibrate.py)
  s / o / c     ดูท่า ตรง / แบ / กำ — แบกับกำควรดูตรงแนว ไม่บิดไปข้าง
  n / p         บันทึก แล้วไปนิ้วถัดไป / นิ้วก่อนหน้า
  q             บันทึกแล้วออก        x หรือ Ctrl+C = ออกโดยไม่บันทึกส่วนที่แก้หลังบันทึกครั้งล่าสุด
เกม (amazinghand_node) อ่านไฟล์นี้ตอนเปิด — คาลิเบรตเสร็จแล้วเปิดเกมใหม่
ทิศของ l/r และ f/b ขึ้นกับการประกอบมือ ลองกดแล้วดูว่านิ้วไปทางไหน
"""
import argparse
import datetime
import json
import os
import sys

from humanoi_games import hand_poses
from humanoi_games.amazinghand_node import DEMO_MIDDLE, Hand, calib_file, load_middle

NAMES_TH = {"index": "นิ้วชี้", "middle": "นิ้วกลาง", "ring": "นิ้วนาง", "thumb": "นิ้วโป้ง"}
VIEWS = {"s": ("ตรง", 0.0), "o": ("แบ", float(hand_poses.OPEN[0])), "c": ("กำ", float(hand_poses.CLOSE[0]))}
CURL = float(hand_poses.CLOSE[0])  # นิ้วอื่นกำไว้ระหว่างตั้งนิ้วนี้
MAX_OFFSET = 30.0  # แตร servo คลาดได้ไม่เกินครึ่งฟัน (9°) — เกิน 30 แปลว่ามีอะไรผิด
HELP = ("l/r เอียง · f/b งอหน้า/หลัง (ใส่ตัวเลขต่อท้ายได้ เช่น l3) · 1+ 1- 2+ 2- ทีละ servo · "
        "s/o/c ดูท่า ตรง/แบ/กำ · n/p นิ้วถัดไป/ก่อนหน้า · q บันทึกแล้วออก · x ออกไม่บันทึก")


def parse(cmd):
    """คำสั่ง -> (ชนิด, จำนวนองศา) หรือ None ถ้าอ่านไม่ออก"""
    if cmd in ("s", "o", "c", "n", "p", "q", "x"):
        return cmd, 0
    if cmd[:1] in ("l", "r", "f", "b") and (cmd[1:] == "" or cmd[1:].isdigit()):
        return cmd[0], int(cmd[1:] or 1)
    if cmd[:1] in ("1", "2") and cmd[1:2] in ("+", "-") and (cmd[2:] == "" or cmd[2:].isdigit()):
        return cmd[:2], int(cmd[2:] or 1)
    return None


def adjust(middle, finger, kind, n):
    """แก้ middle (list 8 ค่า) ของนิ้วที่ finger ตามคำสั่ง — คืน False ถ้าเกิน ±MAX_OFFSET (ไม่แก้)"""
    a, b = 2 * finger, 2 * finger + 1
    da, db = {"l": (n, n), "r": (-n, -n), "f": (n, -n), "b": (-n, n),
              "1+": (n, 0), "1-": (-n, 0), "2+": (0, n), "2-": (0, -n)}[kind]
    na, nb = middle[a] + da, middle[b] + db
    if abs(na) > MAX_OFFSET or abs(nb) > MAX_OFFSET:
        return False
    middle[a], middle[b] = na, nb
    return True


def pose_degrees(middle, finger, view):
    """องศาของ servo 8 ตัว: นิ้วที่ตั้งอยู่ในท่า view นิ้วอื่นกำ (ท่า c = กำทั้งมือ)"""
    out = []
    for i in range(len(hand_poses.FINGERS)):
        theta = VIEWS[view][1] if (i == finger or view == "c") else CURL
        out += [middle[2 * i] + theta, middle[2 * i + 1] - theta]
    return out


def save(path, middle):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    data = {"middle_pos": [round(v, 1) for v in middle],
            "updated": datetime.datetime.now().isoformat(timespec="seconds"),
            "note": "MiddlePos ของ servo ID 1-8 (ชี้ a b, กลาง a b, นาง a b, โป้ง a b) — hand_calibrate"}
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(data, fh, ensure_ascii=False, indent=2)
    os.replace(tmp, path)


def main():
    ap = argparse.ArgumentParser(description="ตั้ง MiddlePos ของมือ AmazingHand ทีละนิ้ว")
    ap.add_argument("--port", default="auto")
    ap.add_argument("--baud", type=int, default=1_000_000, help="ลองค่านี้ก่อน แล้วหาเอง")
    ap.add_argument("--file", default=calib_file())
    ap.add_argument("--speed", type=float, default=3.0)
    args = ap.parse_args()

    loaded = load_middle(args.file)
    middle = list(loaded or DEMO_MIDDLE)
    saved = list(middle)
    print(f"ไฟล์: {args.file} ({'ค่าที่บันทึกไว้' if loaded else 'ยังไม่มี — เริ่มจากค่าเดโม'})")
    hand = Hand(args.port, args.baud, list(range(1, 9)), 4.8, 7.4)
    try:
        port = hand.connect()  # เช็กไฟ + ครบ 8 ตัว ก่อนเปิดแรงบิด
    except Exception as exc:
        sys.exit(f"✗ {exc}")
    print(f"มือพร้อม ({port} @ {hand.found_baud}) — {HELP}")

    finger, view = 0, "s"

    def show():
        hand.move(pose_degrees(middle, finger, view), args.speed)

    try:
        show()
        while True:
            name = hand_poses.FINGERS[finger]
            a, b = 2 * finger, 2 * finger + 1
            cmd = input(f"[{NAMES_TH[name]} · ท่า{VIEWS[view][0]}] ID{a + 1}={middle[a]:+.0f} ID{b + 1}={middle[b]:+.0f} > ")
            parsed = parse(cmd.strip().lower())
            if parsed is None:
                print(HELP)
                continue
            kind, n = parsed
            if kind == "x":
                break
            if kind in ("n", "p", "q"):
                save(args.file, middle)
                saved = list(middle)
                print(f"  บันทึกแล้ว: {[round(v) for v in middle]}")
                if kind == "q":
                    break
                finger = (finger + (1 if kind == "n" else -1)) % len(hand_poses.FINGERS)
                view = "s"
            elif kind in VIEWS:
                view = kind
            elif not adjust(middle, finger, kind, n):
                print(f"  เกิน ±{MAX_OFFSET:.0f}° — ไม่แก้ (แตร servo น่าจะใส่ผิดฟัน)")
                continue
            show()
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        hand.relax()
    if saved != list(loaded or DEMO_MIDDLE):
        print(f"MiddlePos เดิม {[round(v) for v in (loaded or DEMO_MIDDLE)]} → ใหม่ {[round(v) for v in saved]}")
        print("เปิดเกมใหม่ (arm rps ...) เพื่อใช้ค่าใหม่")
    else:
        print("ไม่มีอะไรเปลี่ยน")
    return 0


if __name__ == "__main__":
    sys.exit(main())
