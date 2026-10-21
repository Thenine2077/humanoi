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
from humanoi_games.amazinghand_node import Hand, calib_file, hand_side, load_calib

NAMES_TH = {"index": "นิ้วชี้", "middle": "นิ้วกลาง", "ring": "นิ้วนาง", "thumb": "นิ้วโป้ง"}
VIEW_TH = {"o": "แบสุด", "c": "กำสุด"}
HELP = ("ท่าแบสุด (o) / กำสุด (c): + / - ปรับ · l / r เอียง · ใส่ตัวเลขต่อท้ายได้ เช่น +5 l3 · "
        "n นิ้วถัดไป (บันทึก) · p นิ้วก่อนหน้า · q บันทึกแล้วออก · x ออกไม่บันทึก")


def deg(v):
    """องศาแบบไม่ปัดทิ้ง: +12 หรือ -53.5 (ค่าที่แปลงจากไฟล์เก่าลงท้าย .5 ได้ — ปัดแล้วกด +5 จะดูเหมือนขยับ 4 บ้าง 6 บ้าง)"""
    text = f"{v:+.1f}"
    return text[:-2] if text.endswith(".0") else text


def parse(cmd):
    """คำสั่ง -> (ชนิด, จำนวนองศา) หรือ None ถ้าอ่านไม่ออก"""
    if cmd in ("o", "c", "n", "p", "q", "x"):
        return cmd, 0
    if cmd[:1] in ("l", "r", "+", "-") and (cmd[1:] == "" or cmd[1:].isdigit()):
        return cmd[0], int(cmd[1:] or 1)
    return None


def problem(c, ids, hand):
    """ค่านิ้วนี้ใช้ไม่ได้เพราะอะไร (None = ใช้ได้) — เช็กกับขีดจำกัดที่อ่านจาก servo จริง (hand.limits)"""
    if c["close"] <= c["open"]:
        return "กำสุดต้องมากกว่าแบสุด"
    for bend, pose in ((c["open"], "แบสุด"), (c["close"], "กำสุด")):
        for sid, deg in zip(ids, hand_poses.finger_servos(c, bend)):
            if not hand.within(sid, deg):
                lo, hi = hand.limits[sid]
                return f"ท่า{pose} servo ID{sid} ต้องไป {deg:+.0f}° แต่ servo ตัวนี้หมุนได้ {lo:+.0f}..{hi:+.0f}°"
    return None


def adjust(c, view, kind, n, ids, hand):
    """แก้ค่านิ้ว c ตามคำสั่ง — คืนข้อความเหตุผลถ้าไม่แก้ (None = แก้แล้ว)"""
    new = dict(c)
    if kind in ("l", "r"):
        new["tilt"] += n if kind == "l" else -n
    elif view == "c":  # + = กำมากขึ้น
        new["close"] += n if kind == "+" else -n
    else:              # + = แบมากขึ้น (งอน้อยลง)
        new["open"] -= n if kind == "+" else -n
    why = problem(new, ids, hand)
    if why and not problem(c, ids, hand):  # ค่าเดิมใช้ได้ ค่าใหม่ไม่ได้ = ไม่แก้
        return "ไม่แก้ — " + why
    c.update(new)  # ค่าเดิมเกินอยู่แล้ว (เช่นโหลดมาจากไฟล์) ให้ขยับได้ จะได้ปรับกลับเข้ามา
    return ("ยังเกิน — " + why) if why else None


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
    ap.add_argument("--file", default=calib_file(hand_side()))  # แยกไฟล์ตามชุดมือ (arm hand-side)
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
    print(f"มือพร้อม: ชุดมือ{'ซ้าย' if hand_side() == 'left' else 'ขวา'} ({port} @ {hand.found_baud}) · ไฟล์ {args.file}")
    print(f"ขีดจำกัดที่อ่านจาก servo (องศา): {hand.limits_text()}")
    for w in hand.limit_warnings:
        print(f"  ! {w} — ใช้ลิมิตโรงงาน ±144°")
    print(HELP)

    finger, view = 0, "o"

    def show():
        hand.move(pose_degrees(calib, finger, view), args.speed)

    try:
        show()
        while True:
            f = hand_poses.FINGERS[finger]
            c = calib[f]
            ida, idb = 2 * finger + 1, 2 * finger + 2
            sa, sb = hand_poses.finger_servos(c, c["open"] if view == "o" else c["close"])
            cmd = input(f"[{NAMES_TH[f]} · ท่า{VIEW_TH[view]}] แบ {deg(c['open'])} · กำ {deg(c['close'])} · "
                        f"เอียง {deg(c['tilt'])} · servo ID{ida} {deg(sa)} ID{idb} {deg(sb)} > ")
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
                print(f"  บันทึกแล้ว ({NAMES_TH[f]} แบ {deg(c['open'])} กำ {deg(c['close'])} เอียง {deg(c['tilt'])})")
                if kind == "q":
                    break
                finger = (finger + (1 if kind == "n" else -1)) % len(hand_poses.FINGERS)
                view = "o"
            elif kind in VIEW_TH:
                view = kind
            else:
                reason = adjust(c, view, kind, n, (2 * finger + 1, 2 * finger + 2), hand)
                if reason:
                    print("  " + reason)
                    if reason.startswith("ไม่แก้"):
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
