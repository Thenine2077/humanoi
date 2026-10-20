"""ท่ามือ AmazingHand (8× SCS0009: นิ้วชี้ 1-2, กลาง 3-4, นาง 5-6, โป้ง 7-8) สำหรับเกม

มุมเป็นองศาของ servo คู่ (a, b) ต่อนิ้ว บวกกับ MiddlePos ของแต่ละตัว — ระบบเดียวกับ AmazingHand_Demo.py
ของ Pollen (Side = 1 มือขวา): (a, b) สวนกัน = งอ (θ, -θ), เลื่อนไปทางเดียวกัน = เอียง/กาง
MiddlePos = ท่านิ้วตรง (θ = 0) และมุมกำสุด / แบสุดของแต่ละนิ้ว ตั้งด้วย hand_calibrate (arm hand-calibrate)

ในตาราง GESTURES ใช้ป้ายแทนมุมที่ต่างกันแต่ละนิ้ว:
  "C" = กำสุดของนิ้วนั้น   "O" = แบสุดของนิ้วนั้น   "L" = กำหลวมๆ (กึ่งกลางระหว่างแบสุดกับกำสุด)
หรือใส่ (a, b) ตรงๆ สำหรับท่าที่มีเอียงด้วย (กรรไกร, ส่ายนิ้ว) — ห้ามเกิน ±140° (ลิมิตโรงงาน ±144°)
มือซ้าย (Side = 2) สะท้อนจากมือขวาให้เอง: (a, b) -> (-b, -a) แบบเดียวกับเดโม
"""
FINGERS = ("index", "middle", "ring", "thumb")
DEFAULT_CLOSE = 90.0   # กำของเดโม (CloseHand) — ใช้จนกว่าจะคาลิเบรตกำสุด
DEFAULT_OPEN = -35.0   # แบของเดโม (OpenHand)
LOOSE_AT = 0.5         # ท่าง้าง "L" = แบสุด + 0.5 × (กำสุด − แบสุด)

GESTURES = {
    "rock": {"index": "C", "middle": "C", "ring": "C", "thumb": "C"},           # ค้อน = กำสุด
    "fist": {"index": "C", "middle": "C", "ring": "C", "thumb": "C"},
    "windup": {"index": "L", "middle": "L", "ring": "L", "thumb": "L"},         # ตอนง้าง (เป่า-ยิ้ง)
    "paper": {"index": "O", "middle": "O", "ring": "O", "thumb": "O"},          # กระดาษ = แบสุด
    "scissors": {"index": (-15, 65), "middle": (-65, 15), "ring": "C", "thumb": "C"},  # กรรไกร = Victory()
    "like": {"index": "C", "middle": "C", "ring": "C", "thumb": "O"},           # ชูนิ้วโป้ง
    "point": {"index": (-40, 40), "middle": "C", "ring": "C", "thumb": "C"},    # = Index_Pointing()
    # ท่าย่อยของท่าเคลื่อนไหว
    "snip": {"index": (-50, 20), "middle": (-20, 50), "ring": "C", "thumb": "C"},  # V หุบ (Scissors())
    "wag_a": {"index": (-10, 80), "middle": "C", "ring": "C", "thumb": "C"},     # ส่ายนิ้วชี้ (Nonono())
    "wag_b": {"index": (-80, 10), "middle": "C", "ring": "C", "thumb": "C"},
}

# ท่าเคลื่อนไหว = [(ท่า, รอกี่วินาทีก่อนท่าถัดไป)] ค้างท่าสุดท้าย
ANIMATIONS = {
    "victory": [("scissors", 0.35)] + [("snip", 0.2), ("scissors", 0.2)] * 3,  # ชู 2 นิ้วแล้วขยับแบบกรรไกร
    "nonono": [("point", 0.3)] + [("wag_a", 0.2), ("wag_b", 0.2)] * 3 + [("point", 0.0)],  # ส่ายนิ้ว "ไม่ ไม่"
}


def names():
    return sorted(set(GESTURES) | set(ANIMATIONS))


def finger_angle(tag, finger, close=None, open_=None):
    """ป้าย C / O / L -> θ ของนิ้วนั้น (องศา) ตามค่าคาลิเบรต (ไม่มี = ค่าเดโม)"""
    c = (close or {}).get(finger, DEFAULT_CLOSE)
    o = (open_ or {}).get(finger, DEFAULT_OPEN)
    return {"C": c, "O": o, "L": o + LOOSE_AT * (c - o)}[tag]


def servo_degrees(gesture, middle_pos, side="right", close=None, open_=None):
    """องศาเป้าหมายของ servo 8 ตัวตามลำดับ ID (นิ้วชี้ a, b, กลาง a, b, นาง a, b, โป้ง a, b)

    close / open_ = {นิ้ว: θ} กำสุด / แบสุด จากไฟล์คาลิเบรต (ไม่มี = ค่าเดโม 90 / -35)
    """
    angles = GESTURES[gesture]
    out = []
    for i, finger in enumerate(FINGERS):
        spec = angles[finger]
        if isinstance(spec, str):
            theta = finger_angle(spec, finger, close, open_)
            a, b = theta, -theta
        else:
            a, b = spec
        if side == "left":
            a, b = -b, -a
        out += [middle_pos[2 * i] + a, middle_pos[2 * i + 1] + b]
    return out
