#!/usr/bin/env python3
"""hand_teach — จัดท่ามือเอง (จ็อกทีละนิ้ว หรือปล่อยนิ้วให้ดัดด้วยมือ) แล้วบันทึกเป็นท่ามือ

  arm hand-teach        (บน Pi — ปิด arm player / arm rps / arm hand-calibrate ก่อน: ใช้พอร์ตมือเดียวกัน)

ท่าเก็บใน programs.yaml (hand_poses — องศา servo 8 ตัวตรงๆ) ใช้ได้ทันทีใน arm program / arm hand
ชื่อเดียวกับท่าในตัว (rock paper scissors point like windup) = ใช้แทนท่าในตัว — เกมเป่ายิ้งฉุบใช้ตามด้วย
นิ้วไปไม่ถึงที่สั่ง (ติดนิ้วอื่น / สุดทาง) = ถือไว้ตรงที่ไปถึงแล้วเตือน — ท่าที่ save เป็นท่าที่นิ้วทำได้จริง
(ดันเลยไปค้างไว้ servo จะตัดแรงบิดตัวเองตอนโหลดเกิน นิ้วอ่อนไปทุกท่าหลังจากนั้น)

  1 2 3 4       เลือกนิ้ว ชี้ กลาง นาง โป้ง · 0 = ทุกนิ้ว
  + / -         กำ / แบ นิ้วที่เลือก 5° (+10 = 10°)
  l / r         เอียงนิ้วที่เลือก 5° (l10 = 10°)
  free          ปล่อยนิ้วที่เลือกให้อ่อน — จับดัดด้วยมือได้
  hold          ล็อกนิ้วที่เลือกไว้ตรงที่ดัด
  load <ชื่อ>    เรียกท่า (ในตัวหรือที่สอนไว้) มาแก้ต่อ — เร็วเท่าตอนเล่นจริง ใช้ดูว่าข้างนอกจะออกมาแบบไหน
  save <ชื่อ>    บันทึกท่าตอนนี้ (นิ้วที่ปล่อยอยู่อ่านตำแหน่งจริงแล้วล็อกให้)
  list          ดูท่าที่สอนไว้        del <ชื่อ>  ลบท่า        q  ออก (นิ้วอ่อนลง)
"""
import re
import sys

from ar_program import programs
from humanoi_games import hand_poses
from humanoi_games.amazinghand_node import FINGERS_TH, SPEED as PLAY_SPEED, Hand, calib_file, load_calib

HELP = ("1-4 เลือกนิ้ว (0 = ทุกนิ้ว) · +/- กำ/แบ · l/r เอียง (ใส่ตัวเลขได้ เช่น +10) · free ปล่อยให้ดัดด้วยมือ · "
        "hold ล็อก · load/save/del <ชื่อ> · list · q ออก")
STEP = 5.0
SPEED = 3.0  # ตอนจ็อก — load ใช้ความเร็วเดียวกับตอนเล่นจริง (PLAY_SPEED)
NAME = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")


def parse(cmd):
    """คำสั่ง -> (ชนิด, ค่า) หรือ None"""
    words = cmd.split()
    if not words:
        return None
    w = words[0].lower()
    if w in ("0", "1", "2", "3", "4"):
        return "pick", int(w)
    if w in ("free", "hold", "list", "q", "help"):
        return w, None
    if w in ("load", "save", "del") and len(words) == 2:
        return w, words[1]
    m = re.fullmatch(r"([+\-lr])(\d*)", w)
    if m:
        return m.group(1), float(m.group(2) or STEP)
    return None


def jog(target, fingers, kind, n):
    """แก้องศาเป้าหมายของนิ้วที่เลือก: + กำ, - แบ (servo สวนกัน), l / r เอียง (servo ไปทางเดียวกัน)"""
    da, db = {"+": (n, -n), "-": (-n, n), "l": (n, n), "r": (-n, -n)}[kind]
    for i in fingers:
        target[2 * i] += da
        target[2 * i + 1] += db


def finger_text(target, i):
    a, b = target[2 * i], target[2 * i + 1]
    return f"{FINGERS_TH[i]} งอ {(a - b) / 2:+.0f} เอียง {(a + b) / 2:+.0f}"


def main():
    path = programs.DEFAULT_FILE
    hand = Hand("auto", 1_000_000, list(range(1, 9)), 4.8, 7.4)
    try:
        port = hand.connect()  # เช็กไฟ + ครบ 8 ตัว + อ่านขีดจำกัด แล้วถือท่าปัจจุบัน
    except Exception as exc:
        sys.exit(f"✗ {exc}")
    calib = load_calib(calib_file()) or hand_poses.demo_calib()
    target = hand.read_degrees()
    free = set()
    picked = [0, 1, 2, 3]
    print(f"มือพร้อม ({port} @ {hand.found_baud}) · ไฟล์ท่า {path}\n{HELP}")

    def ids_of(fingers):
        return [hand.ids[2 * i + k] for i in fingers for k in (0, 1)]

    def hold(fingers):
        """ล็อกนิ้วที่ปล่อยอยู่ ตรงตำแหน่งที่ดัดไว้"""
        fs = [i for i in fingers if i in free]
        if not fs:
            return
        ids = ids_of(fs)
        for sid, deg in zip(ids, hand.read_degrees(ids)):
            target[hand.ids.index(sid)] = deg
        hand.torque(ids, True)
        free.difference_update(fs)

    def show(speed=SPEED):
        held = [i for i in range(4) if i not in free]
        if free:  # นิ้วที่ปล่อยอยู่: เป้าหมาย = ตำแหน่งจริง (servo บางรุ่นเปิดแรงบิดเองเมื่อได้เป้าหมาย — ไม่ให้กระตุก)
            ids = ids_of(sorted(free))
            for sid, deg in zip(ids, hand.read_degrees(ids)):
                target[hand.ids.index(sid)] = deg
        clipped = hand.move(target, speed, enable=ids_of(held)) if held else []
        if free:  # แล้วปิดแรงบิดซ้ำให้อ่อนต่อ
            hand.torque(ids_of(sorted(free)), False)
        bad = [sid for sid in clipped if hand.ids.index(sid) // 2 in held]
        if bad:
            print(f"  ! servo ID {bad} ถึงขอบที่หมุนได้ ({hand.limits_text()}) — จำกัดไว้ที่ขอบ")
            for sid in bad:  # ให้ตัวเลขที่เห็นตรงกับที่ servo ทำได้จริง
                lo, hi = hand.limits[sid]
                k = hand.ids.index(sid)
                target[k] = max(lo + 1, min(hi - 1, target[k]))
        if not held:
            return
        # นิ้วไปไม่ถึงที่สั่ง (ติดนิ้วอื่น / สุดทางกลไก) = หยุดดันตรงนั้น — ดันค้าง servo จะตัดแรงบิดตัวเอง
        ids = ids_of(held)
        present, still = hand.settle(ids)
        off = [sid for sid in hand.unloaded() if sid in ids]
        if off:  # servo รีเซ็ตกลางทาง = ไฟมือตก — เปิดคืนตรงที่อยู่ ตัวเลขให้ตรงของจริง
            print(f"  ! servo ID {off} แรงบิดหลุด/รีเซ็ตระหว่างขยับ — ไฟมือตก (แหล่งจ่ายไม่พอตอนหลายนิ้วดันพร้อมกัน) "
                  "· เปิดคืนแล้ว ดูท่าอีกทีก่อน save")
            hand.torque(off, True)
            for sid, deg in zip(off, hand.read_degrees(off)):
                target[hand.ids.index(sid)] = deg
            return
        stuck = hand.stuck(target, ids, present) if still else {}
        if stuck:
            print(f"  ! นิ้วไปต่อไม่ได้: {hand.stuck_text(stuck, target)} — ถือไว้ตรงที่ไปถึง (อย่าดันต่อ)")
            hand.torque(list(stuck), True)
            for sid, deg in stuck.items():
                target[hand.ids.index(sid)] = deg

    def run(kind, val, label):
        """ทำคำสั่งหนึ่ง"""
        nonlocal picked
        if kind == "help":
            print(__doc__)
        elif kind == "pick":
            picked = [0, 1, 2, 3] if val == 0 else [val - 1]
        elif kind == "free":
            hand.torque(ids_of(picked), False)
            free.update(picked)
            print(f"  ปล่อยนิ้ว{label}แล้ว — ดัดด้วยมือได้ เสร็จแล้วพิมพ์ hold หรือ save <ชื่อ>")
        elif kind == "hold":
            hold(picked)
        elif kind in ("+", "-", "l", "r"):
            hold(picked)
            jog(target, picked, kind, val)
            show()
        elif kind == "list":
            taught = programs.load(path)["hand_poses"]
            print(f"  ท่าในตัว: {', '.join(sorted(programs.HAND_GESTURES))}")
            for name, vals in taught.items():
                mark = " (แทนท่าในตัว)" if name in programs.HAND_GESTURES else ""
                print(f"  {name:<12} [" + ", ".join(f"{v:+.0f}" for v in vals) + f"]{mark}")
            if not taught:
                print("  (ยังไม่มีท่าที่สอนเอง)")
        elif kind == "load":
            taught = programs.load(path)["hand_poses"]
            if val not in taught and val not in hand_poses.GESTURES:
                print(f"  ไม่มีท่า '{val}' — ท่าในตัว: {', '.join(sorted(hand_poses.GESTURES))} · ท่าที่สอน: "
                      f"{', '.join(taught) or '-'}")
                return
            hold(sorted(free))  # ล็อกนิ้วที่ปล่อยอยู่ก่อน แล้วค่อยตั้งท่าที่โหลด (ไม่ให้ตำแหน่งจริงทับ)
            if val in taught:
                target[:] = taught[val]
            else:
                target[:] = hand_poses.servo_degrees(val, calib)
            show(PLAY_SPEED)  # เร็วเท่าตอนเล่นจริง — ท่าที่ load ออกมาเป็นยังไง ใน arm hand / เกมก็เป็นอย่างนั้น
        elif kind == "save":
            if not NAME.match(val) or val == "relax":
                print("  ชื่อต้องเป็นภาษาอังกฤษ ขึ้นต้นด้วยตัวอักษร (a-z 0-9 _) และไม่ใช่ relax")
                return
            hold(sorted(free))  # นิ้วที่ปล่อยอยู่: อ่านตำแหน่งที่ดัดไว้แล้วล็อก
            data = programs.load(path)
            old = data["hand_poses"].get(val)
            data["hand_poses"][val] = [round(v, 1) for v in target]
            programs.save(path, data)
            note = " — ใช้แทนท่าในตัว" if val in programs.HAND_GESTURES else ""
            print(f"  {'แทนที่' if old else 'บันทึก'}ท่า '{val}'{note}: [" + ", ".join(f"{v:+.0f}" for v in target) + "]")
        elif kind == "del":
            data = programs.load(path)
            if data["hand_poses"].pop(val, None) is None:
                print(f"  ไม่มีท่าที่สอนชื่อ '{val}'")
                return
            used = [p for p, steps in data["programs"].items()
                    if any(val in programs.step_hands(st).values() for st in steps)]
            if used and val not in programs.HAND_GESTURES:
                print(f"  ลบไม่ได้ — ยังถูกใช้ในโปรแกรม: {', '.join(used)}")
                return
            programs.save(path, data)
            print(f"  ลบท่า '{val}' แล้ว" + (" (กลับไปใช้ท่าในตัว)" if val in programs.HAND_GESTURES else ""))

    try:
        show()
        while True:
            label = "ทุกนิ้ว" if len(picked) == 4 else FINGERS_TH[picked[0]]
            status = " · ".join(finger_text(target, i) + (" (ปล่อย)" if i in free else "") for i in picked)
            cmd = input(f"[{label}] {status} > ")
            parsed = parse(cmd)
            if parsed is None:
                print(HELP)
                continue
            if parsed[0] == "q":
                break
            try:
                run(*parsed, label)
            except Exception as exc:  # คุยกับมือพลาด (servo รีเซ็ตเพราะไฟตก) — ไม่หลุดโปรแกรม ท่าที่บันทึกแล้วไม่หาย
                print(f"  ✗ คุยกับมือพลาด ({exc}) — เช็กไฟมือ แล้วสั่งใหม่ได้เลย")
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        hand.relax()
    return 0

if __name__ == "__main__":
    sys.exit(main())
