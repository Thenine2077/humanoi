"""ท่ามือ AmazingHand (8× SCS0009: นิ้วชี้ 1-2, กลาง 3-4, นาง 5-6, โป้ง 7-8) สำหรับเกม

แต่ละนิ้วมี servo 2 ตัว (a, b): หมุนสวนกัน = งอ/เหยียด, หมุนไปทางเดียวกัน = เอียง/กาง
  a = งอ + เอียง + กาง        b = -งอ + เอียง + กาง        (องศาของ servo, ระบบเดียวกับ AmazingHand_Demo.py)
ค่าคาลิเบรตต่อนิ้ว (arm hand-calibrate): open = งอตอนแบสุด, close = งอตอนกำสุด, tilt = เอียงให้นิ้วตรง
ในตาราง GESTURES แต่ละนิ้วเป็น (ป้ายงอ, กางเพิ่ม):
  "C" = กำสุด   "O" = แบสุด   "L" = กำหลวมๆ (กึ่งกลางแบสุด-กำสุด)   กางเพิ่ม = องศาเอียงเพิ่มจากนิ้วตรง (ท่า V)
ผลรวมห้ามเกิน ±140° (ลิมิตโรงงาน ±144° — amazinghand_node จำกัดให้)
มือซ้าย (Side = 2): กางเพิ่มกลับทิศ (เหมือนเดโมสะท้อน (a, b) -> (-b, -a)) ค่าคาลิเบรตไม่กลับ
"""
FINGERS = ("index", "middle", "ring", "thumb")
DEMO_MIDDLE = [3.0, 0.0, -5.0, -8.0, -2.0, 5.0, -12.0, 0.0]  # MiddlePos ของมือตัวอย่างใน AmazingHand_Demo.py
DEMO_OPEN, DEMO_CLOSE = -35.0, 90.0                            # OpenHand / CloseHand ของเดโม (เทียบ MiddlePos)
LOOSE_AT = 0.5  # "L" = แบสุด + 0.5 × (กำสุด − แบสุด)

GESTURES = {
    "rock": {f: ("C", 0) for f in FINGERS},                                      # ค้อน = กำสุด
    "fist": {f: ("C", 0) for f in FINGERS},
    "windup": {f: ("L", 0) for f in FINGERS},                                    # ตอนง้าง (เป่า-ยิ้ง)
    "paper": {f: ("O", 0) for f in FINGERS},                                     # กระดาษ = แบสุด
    "scissors": {"index": ("O", 25), "middle": ("O", -25), "ring": ("C", 0), "thumb": ("C", 0)},  # V = Victory()
    "like": {"index": ("C", 0), "middle": ("C", 0), "ring": ("C", 0), "thumb": ("O", 0)},       # ชูนิ้วโป้ง
    "point": {"index": ("O", 0), "middle": ("C", 0), "ring": ("C", 0), "thumb": ("C", 0)},      # ชี้นิ้ว
    # ท่าย่อยของท่าเคลื่อนไหว
    "snip": {"index": ("O", -15), "middle": ("O", 15), "ring": ("C", 0), "thumb": ("C", 0)},    # V หุบ (Scissors())
    "wag_a": {"index": ("O", 35), "middle": ("C", 0), "ring": ("C", 0), "thumb": ("C", 0)},     # ส่ายนิ้ว (Nonono())
    "wag_b": {"index": ("O", -35), "middle": ("C", 0), "ring": ("C", 0), "thumb": ("C", 0)},
}

# ท่าเคลื่อนไหว = [(ท่า, รอกี่วินาทีก่อนท่าถัดไป)] ค้างท่าสุดท้าย
ANIMATIONS = {
    "victory": [("scissors", 0.35)] + [("snip", 0.2), ("scissors", 0.2)] * 3,  # ชู 2 นิ้วแล้วขยับแบบกรรไกร
    "nonono": [("point", 0.3)] + [("wag_a", 0.2), ("wag_b", 0.2)] * 3 + [("point", 0.0)],  # ส่ายนิ้ว "ไม่ ไม่"
}


def names():
    return sorted(set(GESTURES) | set(ANIMATIONS))


def demo_calib():
    """ค่าคาลิเบรตที่ให้ท่าเหมือนเดโมของ Pollen ทุกประการ (ใช้ตอนยังไม่ได้คาลิเบรต)"""
    out = {}
    for i, f in enumerate(FINGERS):
        ma, mb = DEMO_MIDDLE[2 * i], DEMO_MIDDLE[2 * i + 1]
        bend0, tilt0 = (ma - mb) / 2, (ma + mb) / 2
        out[f] = {"open": bend0 + DEMO_OPEN, "close": bend0 + DEMO_CLOSE, "tilt": tilt0}
    return out


def bend_of(tag, c):
    """ป้าย C / O / L -> องศางอของนิ้ว (c = ค่าคาลิเบรตของนิ้วนั้น)"""
    return {"C": c["close"], "O": c["open"], "L": c["open"] + LOOSE_AT * (c["close"] - c["open"])}[tag]


def convert_pose(degrees, calib_from, side_from, calib_to, side_to):
    """ท่าที่สอนเอง (องศา servo 8 ตัว) ของชุดมือหนึ่ง -> ท่าเดียวกันบนอีกชุดมือ

    องศา servo ลอกตรงๆ ไม่ได้: แต่ละชุดค่าคาลิเบรตต่างกัน และมือซ้ายกางกลับทิศ — แปลงเป็นรูปนิ้วก่อน
    (งอกี่ส่วนของช่วงแบสุด-กำสุด + กางเพิ่มจากนิ้วตรง) แล้วสร้างองศาใหม่ด้วยค่าคาลิเบรตของชุดปลายทาง
    """
    out = []
    for i, f in enumerate(FINGERS):
        a, b = degrees[2 * i], degrees[2 * i + 1]
        cf, ct = calib_from[f], calib_to[f]
        frac = ((a - b) / 2 - cf["open"]) / (cf["close"] - cf["open"])
        spread = ((a + b) / 2 - cf["tilt"]) * (-1 if side_from == "left" else 1)
        bend = ct["open"] + frac * (ct["close"] - ct["open"])
        out += [max(-140.0, min(140.0, v)) for v in finger_servos(ct, bend, spread, side_to)]
    return out


def finger_servos(c, bend, spread=0.0, side="right"):
    """(a, b) ของนิ้วหนึ่ง จากองศางอ + เอียงที่คาลิเบรตไว้ + กางเพิ่ม"""
    tilt = c["tilt"] + (spread if side != "left" else -spread)
    return bend + tilt, -bend + tilt


def servo_degrees(gesture, calib=None, side="right"):
    """องศาเป้าหมายของ servo 8 ตัวตามลำดับ ID (นิ้วชี้ a, b, กลาง a, b, นาง a, b, โป้ง a, b)

    calib = {นิ้ว: {open, close, tilt}} จากไฟล์คาลิเบรต (ไม่มี / ขาดนิ้วไหน = ค่าเดโม)
    """
    base = demo_calib()
    out = []
    for f in FINGERS:
        c = (calib or {}).get(f) or base[f]
        tag, spread = GESTURES[gesture][f]
        out += list(finger_servos(c, bend_of(tag, c), spread, side))
    return out
