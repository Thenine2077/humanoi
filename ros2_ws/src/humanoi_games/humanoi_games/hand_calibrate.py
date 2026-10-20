#!/usr/bin/env python3
"""hand_calibrate — คาลิเบรตมือ AmazingHand ทีละนิ้วด้วยตา: ท่าตรง (MiddlePos), กำสุด, แบสุด แล้วบันทึกให้เกมใช้

  arm hand-calibrate                         (บน Pi — ปิดเกมก่อน: พอร์ตมือเปิดได้ทีละโปรแกรม)
  arm hand-calibrate --reset                 เริ่มใหม่จากค่าเดโม (ไฟล์ค่าเดิมเปลี่ยนชื่อเป็น .bak-<เวลา> ไม่ลบ)

ทีละนิ้ว 3 ขั้น (นิ้วอื่นกำไว้ไม่ให้บัง):
  1) ท่าตรง (s):  l / r  เอียงนิ้วไปทางหนึ่ง/อีกทาง   f / b  งอไปหน้า/หลัง   จนนิ้วตั้งตรงแนวฝ่ามือ
                  (1+ 1- 2+ 2- = ขยับ servo ทีละตัว)
  2) กำสุด (c):  + / -  กำมากขึ้น / น้อยลง — หยุดตรงที่ปลายนิ้วแตะฝ่ามือพอดี
                  ได้ยินเสียง servo ครางค้าง = กำเกิน ให้กด - จนเงียบ
  3) แบสุด (o):  + / -  แบมากขึ้น / น้อยลง — หยุดตรงที่นิ้วแบสุดแล้วยังดูเป็นธรรมชาติ
ใส่ตัวเลขต่อท้ายได้ทุกคำสั่ง เช่น l3, +5
  n / p   บันทึก แล้วไปนิ้วถัดไป / ก่อนหน้า        q   บันทึกแล้วออก
  x หรือ Ctrl+C = ออกโดยไม่บันทึกส่วนที่แก้หลังบันทึกครั้งล่าสุด
เกม (amazinghand_node) อ่านไฟล์นี้ตอนเปิด — คาลิเบรตเสร็จแล้วเปิดเกมใหม่
ทิศของ l/r และ f/b ขึ้นกับการประกอบมือ ลองกดแล้วดูว่านิ้วไปทางไหน
"""
import argparse
import datetime
import json
import os
import sys

from humanoi_games import hand_poses
from humanoi_games.amazinghand_node import DEMO_MIDDLE, MAX_MIDDLE, Hand, calib_file, load_calib

NAMES_TH = {"index": "นิ้วชี้", "middle": "นิ้วกลาง", "ring": "นิ้วนาง", "thumb": "นิ้วโป้ง"}
VIEW_TH = {"s": "ตรง", "c": "กำสุด", "o": "แบสุด"}
MAX_OFFSET = MAX_MIDDLE  # ±60° — ปกติแตรคลาดไม่เกินครึ่งฟัน (9°) เกินจากนี้ต้องถอดใส่แตรใหม่
CLOSE_RANGE = (45.0, 135.0)  # θ กำสุดที่ยอมให้ตั้ง
OPEN_RANGE = (-80.0, 0.0)    # θ แบสุด (ติดลบ = เลยท่าตรงไปทางแบ)
HELP = ("ท่าตรง: l/r เอียง f/b งอหน้า/หลัง (1+ 1- 2+ 2- ทีละ servo) · c กำสุด / o แบสุด: +/- ปรับ · "
        "s ท่าตรง · ใส่ตัวเลขต่อท้ายได้ เช่น l3 +5 · n/p บันทึกแล้วไปนิ้วถัดไป/ก่อนหน้า · q บันทึกแล้วออก · "
        "x ออกไม่บันทึก")


def parse(cmd):
    """คำสั่ง -> (ชนิด, จำนวนองศา) หรือ None ถ้าอ่านไม่ออก"""
    if cmd in ("s", "o", "c", "n", "p", "q", "x"):
        return cmd, 0
    if cmd[:1] in ("l", "r", "f", "b", "+", "-") and (cmd[1:] == "" or cmd[1:].isdigit()):
        return cmd[0], int(cmd[1:] or 1)
    if cmd[:1] in ("1", "2") and cmd[1:2] in ("+", "-") and (cmd[2:] == "" or cmd[2:].isdigit()):
        return cmd[:2], int(cmd[2:] or 1)
    return None


def adjust(middle, finger, kind, n):
    """แก้ middle (list 8 ค่า) ของนิ้วที่ finger — คืน False ถ้าเกิน ±MAX_OFFSET (ไม่แก้)"""
    a, b = 2 * finger, 2 * finger + 1
    da, db = {"l": (n, n), "r": (-n, -n), "f": (n, -n), "b": (-n, n),
              "1+": (n, 0), "1-": (-n, 0), "2+": (0, n), "2-": (0, -n)}[kind]
    na, nb = middle[a] + da, middle[b] + db
    if abs(na) > MAX_OFFSET or abs(nb) > MAX_OFFSET:
        return False
    middle[a], middle[b] = na, nb
    return True


def adjust_limit(limits, name, view, sign, n):
    """+/- ที่ท่ากำ = กำมาก/น้อย, ที่ท่าแบ = แบมาก/น้อย — คืน False ถ้าออกนอกช่วง (ไม่แก้)"""
    lo, hi = CLOSE_RANGE if view == "c" else OPEN_RANGE
    default = hand_poses.DEFAULT_CLOSE if view == "c" else hand_poses.DEFAULT_OPEN
    step = n if view == "c" else -n  # แบมากขึ้น = θ ติดลบมากขึ้น
    value = limits.get(name, default) + (step if sign == "+" else -step)
    if not lo <= value <= hi:
        return False
    limits[name] = value
    return True


def pose_degrees(middle, finger, view, close, open_):
    """องศาของ servo 8 ตัว: นิ้วที่ตั้งอยู่ในท่า view นิ้วอื่นกำสุด (ท่า c = กำทั้งมือ)"""
    out = []
    for i, name in enumerate(hand_poses.FINGERS):
        if i == finger or view == "c":
            theta = {"s": 0.0, "c": hand_poses.finger_angle("C", name, close, open_),
                     "o": hand_poses.finger_angle("O", name, close, open_)}[view]
        else:
            theta = hand_poses.finger_angle("C", name, close, open_)
        out += [middle[2 * i] + theta, middle[2 * i + 1] - theta]
    return out


def save(path, calib):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    data = {"middle_pos": [round(v, 1) for v in calib["middle_pos"]],
            "close": {k: round(v, 1) for k, v in calib["close"].items()},
            "open": {k: round(v, 1) for k, v in calib["open"].items()},
            "updated": datetime.datetime.now().isoformat(timespec="seconds"),
            "note": "middle_pos = ท่าตรงของ servo ID 1-8 (ชี้ a b, กลาง a b, นาง a b, โป้ง a b); "
                    "close / open = θ กำสุด / แบสุดของแต่ละนิ้ว — hand_calibrate"}
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(data, fh, ensure_ascii=False, indent=2)
    os.replace(tmp, path)


def copy(calib):
    return {"middle_pos": list(calib["middle_pos"]), "close": dict(calib["close"]), "open": dict(calib["open"])}


def main():
    ap = argparse.ArgumentParser(description="คาลิเบรตมือ AmazingHand ทีละนิ้ว")
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
    calib = copy(loaded) if loaded else {"middle_pos": list(DEMO_MIDDLE), "close": {}, "open": {}}
    saved = copy(calib)
    print(f"ไฟล์: {args.file} ({'ค่าที่บันทึกไว้' if loaded else 'ยังไม่มี — เริ่มจากค่าเดโม'})")
    hand = Hand(args.port, args.baud, list(range(1, 9)), 4.8, 7.4)
    try:
        port = hand.connect()  # เช็กไฟ + ครบ 8 ตัว ก่อนเปิดแรงบิด
    except Exception as exc:
        sys.exit(f"✗ {exc}")
    print(f"มือพร้อม ({port} @ {hand.found_baud})\n{HELP}")

    finger, view = 0, "s"

    def show():
        clipped = hand.move(pose_degrees(calib["middle_pos"], finger, view, calib["close"], calib["open"]),
                            args.speed)
        if clipped:
            print(f"  (servo ID {clipped} ถึงขอบ ±140° ในท่านี้ — ไปต่อไม่ได้)")

    try:
        show()
        while True:
            name = hand_poses.FINGERS[finger]
            m = calib["middle_pos"]
            a, b = 2 * finger, 2 * finger + 1
            close = hand_poses.finger_angle("C", name, calib["close"], calib["open"])
            opened = hand_poses.finger_angle("O", name, calib["close"], calib["open"])
            cmd = input(f"[{NAMES_TH[name]} · ท่า{VIEW_TH[view]}] ID{a + 1}={m[a]:+.0f} ID{b + 1}={m[b]:+.0f}"
                        f" · กำสุด {close:.0f} · แบสุด {opened:.0f} > ")
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
                print(f"  บันทึกแล้ว — MiddlePos {[round(v) for v in m]} · กำสุด "
                      f"{ {k: round(v) for k, v in calib['close'].items()} } · แบสุด "
                      f"{ {k: round(v) for k, v in calib['open'].items()} }")
                if kind == "q":
                    break
                finger = (finger + (1 if kind == "n" else -1)) % len(hand_poses.FINGERS)
                view = "s"
            elif kind in VIEW_TH:
                view = kind
            elif kind in ("+", "-"):
                if view == "s":
                    print("  +/- ใช้ที่ท่ากำสุด (c) หรือแบสุด (o) — กด c หรือ o ก่อน")
                    continue
                if not adjust_limit(calib["close" if view == "c" else "open"], name, view, kind, n):
                    lo, hi = CLOSE_RANGE if view == "c" else OPEN_RANGE
                    print(f"  นอกช่วง {lo:.0f}..{hi:.0f}° — ไม่แก้")
                    continue
            elif not adjust(m, finger, kind, n):
                print(f"  เกิน ±{MAX_OFFSET:.0f}° — ไม่แก้ (แตร servo ใส่คลาดหลายฟัน ต้องถอดใส่ใหม่)")
                continue
            show()
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        hand.relax()
    before = loaded or {"middle_pos": list(DEMO_MIDDLE), "close": {}, "open": {}}
    if saved != copy(before):
        print("บันทึกแล้ว — เปิดเกมใหม่ (arm rps ...) เพื่อใช้ค่าใหม่")
    else:
        print("ไม่มีอะไรเปลี่ยน")
    return 0


if __name__ == "__main__":
    sys.exit(main())
