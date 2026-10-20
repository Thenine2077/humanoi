#!/usr/bin/env python3
"""hand_calibrate — คาลิเบรตมือ AmazingHand ทีละนิ้วด้วยตา: แบสุด, กำสุด, เอียง แล้วบันทึกให้เกมใช้

  arm hand-calibrate                 (บน Pi — ปิดเกมก่อน: พอร์ตมือเปิดได้ทีละโปรแกรม)
  arm hand-calibrate --reset         เริ่มใหม่จากค่าเดโม (ไฟล์ค่าเดิมเปลี่ยนชื่อเป็น .bak-<เวลา> ไม่ลบ)

แต่ละนิ้ว (นิ้วอื่นกำไว้ไม่ให้บัง):
  1. เริ่มที่ท่าแบสุด:  + / -  แบมากขึ้น / น้อยลง      l / r  เอียงนิ้วจนตั้งตรง ไม่เอียงข้าง
  2. กด c ไปท่ากำสุด:  + / -  กำมากขึ้น / น้อยลง — หยุดตรงที่ปลายนิ้วแตะฝ่ามือพอดี
                        ได้ยินเสียง servo ครางค้าง = กำเกิน กด - จนเงียบ
  3. กด n ไปนิ้วถัดไป (บันทึกให้เลย)          o = กลับไปดูท่าแบสุด
ใส่ตัวเลขต่อท้ายได้ เช่น +5, l3 · p = นิ้วก่อนหน้า · q = บันทึกแล้วออก · x หรือ Ctrl+C = ออกไม่บันทึก
เกม (amazinghand_node) อ่านไฟล์นี้ตอนเปิด — คาลิเบรตเสร็จแล้วเปิดเกมใหม่
ทิศของ l / r ขึ้นกับการประกอบมือ ลองกดแล้วดูว่านิ้วไปทางไหน
"""
import argparse
import datetime
import json
import os
import sys

from humanoi_games import hand_poses
from humanoi_games.amazinghand_node import SERVO_LIMIT, Hand, calib_file, load_calib

NAMES_TH = {"index": "นิ้วชี้", "middle": "นิ้วกลาง", "ring": "นิ้วนาง", "thumb": "นิ้วโป้ง"}
VIEW_TH = {"o": "แบสุด", "c": "กำสุด"}
MIN_GAP = 20.0  # กำสุดต้องมากกว่าแบสุดอย่างน้อยเท่านี้
HELP = ("ท่าแบสุด (o) / กำสุด (c): + / - ปรับ · l / r เอียง · ใส่ตัวเลขต่อท้ายได้ เช่น +5 l3 · "
        "n นิ้วถัดไป (บันทึก) · p นิ้วก่อนหน้า · q บันทึกแล้วออก · x ออกไม่บันทึก")


def parse(cmd):
    """คำสั่ง -> (ชนิด, จำนวนองศา) หรือ None ถ้าอ่านไม่ออก"""
    if cmd in ("o", "c", "n", "p", "q", "x"):
        return cmd, 0
    if cmd[:1] in ("l", "r", "+", "-") and (cmd[1:] == "" or cmd[1:].isdigit()):
        return cmd[0], int(cmd[1:] or 1)
    return None


def fits(c):
    """ค่านิ้วนี้ใช้ได้ไหม: กำสุด > แบสุด อย่างน้อย MIN_GAP และ servo ไม่ทะลุขอบทั้งท่าแบและท่ากำ"""
    if c["close"] - c["open"] < MIN_GAP:
        return False
    return all(abs(v) <= SERVO_LIMIT for bend in (c["open"], c["close"])
               for v in hand_poses.finger_servos(c, bend))


def adjust(c, view, kind, n):
    """แก้ค่านิ้ว c ตามคำสั่ง — คืนข้อความเหตุผลถ้าไม่แก้ (None = แก้แล้ว)"""
    new = dict(c)
    if kind in ("l", "r"):
        new["tilt"] += n if kind == "l" else -n
    elif view == "c":  # + = กำมากขึ้น
        new["close"] += n if kind == "+" else -n
    else:              # + = แบมากขึ้น (งอน้อยลง)
        new["open"] -= n if kind == "+" else -n
    if not fits(new):
        return (f"ไม่แก้ — กำสุดต้องมากกว่าแบสุดอย่างน้อย {MIN_GAP:.0f}° และ servo ต้องไม่เกิน ±{SERVO_LIMIT:.0f}°")
    c.update(new)
    return None


def pose_degrees(calib, finger, view):
    """องศาของ servo 8 ตัว: นิ้วที่ตั้งอยู่ในท่า view นิ้วอื่นกำสุด (ท่า c = กำทั้งมือ)"""
    out = []
    for i, f in enumerate(hand_poses.FINGERS):
        c = calib[f]
        bend = c["open"] if (i == finger and view == "o") else c["close"]
        out += list(hand_poses.finger_servos(c, bend))
    return out


def save(path, calib):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    data = {"version": 2,
            "fingers": {f: {k: round(v, 1) for k, v in c.items()} for f, c in calib.items()},
            "updated": datetime.datetime.now().isoformat(timespec="seconds"),
            "note": "ต่อนิ้ว: open = งอตอนแบสุด, close = งอตอนกำสุด, tilt = เอียง (องศา servo) — hand_calibrate"}
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(data, fh, ensure_ascii=False, indent=2)
    os.replace(tmp, path)


def copy(calib):
    return {f: dict(c) for f, c in calib.items()}


def main():
    ap = argparse.ArgumentParser(description="คาลิเบรตมือ AmazingHand ทีละนิ้ว: แบสุด กำสุด เอียง")
    ap.add_argument("--port", default="auto")
    ap.add_argument("--baud", type=int, default=1_000_000, help="ลองค่านี้ก่อน แล้วหาเอง")
    ap.add_argument("--file", default=calib_file())
    ap.add_argument("--speed", type=float, default=3.0)
    ap.add_argument("--reset", action="store_true", help="สำรองไฟล์ค่าเดิมแล้วเริ่มใหม่จากค่าเดโม")
    args = ap.parse_args()

    if args.reset and os.path.exists(args.file):
        backup = f"{args.file}.bak-{datetime.datetime.now():%Y%m%d-%H%M%S}"
        os.replace(args.file, backup)
        print(f"รีเซ็ตแล้ว — ค่าเดิมสำรองไว้ที่ {backup} (ออกด้วย x = เกมใช้ค่าเดโม)")
    loaded = load_calib(args.file)
    calib = copy(loaded or hand_poses.demo_calib())
    saved = copy(calib)
    print(f"ไฟล์: {args.file} ({'ค่าที่บันทึกไว้' if loaded else 'ยังไม่มี — เริ่มจากค่าเดโม'})")
    hand = Hand(args.port, args.baud, list(range(1, 9)), 4.8, 7.4)
    try:
        port = hand.connect()  # เช็กไฟ + ครบ 8 ตัว ก่อนเปิดแรงบิด
    except Exception as exc:
        sys.exit(f"✗ {exc}")
    print(f"มือพร้อม ({port} @ {hand.found_baud})\n{HELP}")

    finger, view = 0, "o"

    def show():
        hand.move(pose_degrees(calib, finger, view), args.speed)

    try:
        show()
        while True:
            f = hand_poses.FINGERS[finger]
            c = calib[f]
            cmd = input(f"[{NAMES_TH[f]} · ท่า{VIEW_TH[view]}] แบ {c['open']:+.0f} · กำ {c['close']:+.0f} · "
                        f"เอียง {c['tilt']:+.0f} > ")
            parsed = parse(cmd.strip().lower())
            if parsed is None:
                print(HELP)
                continue
            kind, n = parsed
            if kind == "x":
                break
            if kind in ("n", "p", "q"):
                save(args.file, calib)
                saved = copy(calib)
                print(f"  บันทึกแล้ว ({NAMES_TH[f]} แบ {c['open']:+.0f} กำ {c['close']:+.0f} เอียง {c['tilt']:+.0f})")
                if kind == "q":
                    break
                finger = (finger + (1 if kind == "n" else -1)) % len(hand_poses.FINGERS)
                view = "o"
            elif kind in VIEW_TH:
                view = kind
            else:
                reason = adjust(c, view, kind, n)
                if reason:
                    print("  " + reason)
                    continue
            show()
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        hand.relax()
    print("บันทึกแล้ว — เปิดเกมใหม่ (arm rps ...) เพื่อใช้ค่าใหม่" if saved != copy(loaded or hand_poses.demo_calib())
          else "ไม่มีอะไรเปลี่ยน")
    return 0


if __name__ == "__main__":
    sys.exit(main())
