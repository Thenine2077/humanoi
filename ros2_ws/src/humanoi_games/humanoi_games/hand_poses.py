"""ท่ามือ AmazingHand (8× SCS0009: นิ้วชี้ 1-2, กลาง 3-4, นาง 5-6, โป้ง 7-8) สำหรับเกม

มุมเป็นองศาของ servo คู่ (a, b) ต่อนิ้ว บวกกับ MiddlePos ของแต่ละตัว — ระบบเดียวกับ AmazingHand_Demo.py
ของ Pollen (Side = 1 มือขวา): แบ = (-35, 35), กำ = (90, -90), (a, b) ต่างกันมาก = งอ, เลื่อนไปทางเดียวกัน = กาง
แก้ท่าได้ที่ GESTURES ด้านล่าง (ห้ามเกิน ±144° = ลิมิตโรงงานของ servo; นิ้วชนกันเองที่งอราว 100-120°)
มือซ้าย (Side = 2) สะท้อนจากมือขวาให้เอง: (a, b) -> (-b, -a) แบบเดียวกับเดโม
"""
OPEN = (-35, 35)
CLOSE = (90, -90)
FINGERS = ("index", "middle", "ring", "thumb")
# กำสุด: มุมที่นิ้วเริ่มชนฝ่ามือ (วัดด้วย AmazingHand\tools\find_limits.py: ชี้ 106 กลาง 116 นาง 98 โป้ง 122)
# ลบ 4° — กำแน่นที่สุดโดยมอเตอร์ไม่ต้องดันค้าง
TIGHT = {"index": (102, -102), "middle": (112, -112), "ring": (94, -94), "thumb": (118, -118)}
LOOSE = (30, -30)  # กำหลวมๆ นิ้วกางออก — ระหว่างแบ (-35) กับกำ (90)

GESTURES = {
    "rock": dict(TIGHT),                                                            # ค้อน = กำสุด
    "windup": {"index": LOOSE, "middle": LOOSE, "ring": LOOSE, "thumb": LOOSE},     # ตอนง้าง (เป่า-ยิ้ง)
    "fist": {"index": CLOSE, "middle": CLOSE, "ring": CLOSE, "thumb": CLOSE},      # = CloseHand() ของเดโม
    "paper": {"index": OPEN, "middle": OPEN, "ring": OPEN, "thumb": OPEN},          # กระดาษ = OpenHand()
    "scissors": {"index": (-15, 65), "middle": (-65, 15), "ring": CLOSE, "thumb": CLOSE},  # กรรไกร = Victory()
    "like": {"index": CLOSE, "middle": CLOSE, "ring": CLOSE, "thumb": OPEN},        # ชูนิ้วโป้ง
    "point": {"index": (-40, 40), "middle": CLOSE, "ring": CLOSE, "thumb": CLOSE},  # = Index_Pointing()
    # ท่าย่อยของท่าเคลื่อนไหว
    "snip": {"index": (-50, 20), "middle": (-20, 50), "ring": CLOSE, "thumb": CLOSE},  # V หุบ (Scissors())
    "wag_a": {"index": (-10, 80), "middle": CLOSE, "ring": CLOSE, "thumb": CLOSE},     # ส่ายนิ้วชี้ (Nonono())
    "wag_b": {"index": (-80, 10), "middle": CLOSE, "ring": CLOSE, "thumb": CLOSE},
}

# ท่าเคลื่อนไหว = [(ท่า, รอกี่วินาทีก่อนท่าถัดไป)] ค้างท่าสุดท้าย
ANIMATIONS = {
    "victory": [("scissors", 0.35)] + [("snip", 0.2), ("scissors", 0.2)] * 3,  # ชู 2 นิ้วแล้วขยับแบบกรรไกร
    "nonono": [("point", 0.3)] + [("wag_a", 0.2), ("wag_b", 0.2)] * 3 + [("point", 0.0)],  # ส่ายนิ้ว "ไม่ ไม่"
}


def names():
    return sorted(set(GESTURES) | set(ANIMATIONS))


def servo_degrees(gesture, middle_pos, side="right"):
    """องศาเป้าหมายของ servo 8 ตัวตามลำดับ ID (นิ้วชี้ a, b, กลาง a, b, นาง a, b, โป้ง a, b)"""
    angles = GESTURES[gesture]
    out = []
    for i, finger in enumerate(FINGERS):
        a, b = angles[finger]
        if side == "left":
            a, b = -b, -a
        out += [middle_pos[2 * i] + a, middle_pos[2 * i + 1] + b]
    return out
