"""เส้นทางและระยะจอดของโชว์ (config/routes.yaml) + คิดเส้นทางขากลับ — ไม่ใช้ ROS (ทดสอบตรงๆ ได้)

เส้นทาง = ขาตรงๆ ต่อกัน: forward / back / left / right (m) หรือ turn (องศา บวก = หมุนซ้าย)
  arm show route-record <ชื่อ>   ขับรีโมทไปตามทางจริงแล้วจดขาให้เอง (encoder)
  arm show route-set <ชื่อ> "forward 1.2, turn 90, forward 0.5"   พิมพ์เอง
ชื่อแบบ X_to_Y จะได้ Y_to_X (ขากลับ) ให้อัตโนมัติ: หมุน 180° -> ขาเดิมย้อนลำดับ (หมุนกลับทิศ) -> หมุน 180°
  ขากลับใช้แต่เดินหน้า (ฐานถอยหลังยังวิ่งไม่ตรง) และจบหันหน้าทางเดิมพร้อมจอด
ระยะจอด = ระยะจากขอบหน้าตัวหุ่นถึงสิ่งที่ lidar เห็นข้างหน้าตอนจอดถูกที่ (arm show dock-save <ชื่อ>)
ระยะถอย (approach) = ข้างหน้าห่างเท่าไหร่ตอนเส้นทาง X_to_Y เริ่ม (X) / จบ (Y) — route-record จดให้
  ขั้น undock: <ชื่อ> ถอยออกจากโต๊ะจนห่างเท่านี้ = กลับมาที่ปลายเส้นทางพอดี ขากลับจึงตรงทาง
"""
import math
import os

import yaml

LINEAR = ("forward", "back", "left", "right")
KINDS = LINEAR + ("turn",)
OPPOSITE = {"forward": "back", "back": "forward", "left": "right", "right": "left"}
TH = {"forward": "เดินหน้า", "back": "ถอยหลัง", "left": "สไลด์ซ้าย", "right": "สไลด์ขวา", "turn": "หมุน"}
DONE_M = 0.02      # เหลือน้อยกว่านี้ (m) = ถือว่าถึงแล้ว
DONE_DEG = 2.0
SECTIONS = ("routes", "docks", "approach")

HEADER = """\
# เส้นทางและระยะจอดของโชว์ — เขียนโดย arm show route-record / route-set / dock-save (แก้มือได้)
# routes: ชื่อ -> ขาต่อกัน {forward|back|left|right: m} หรือ {turn: องศา บวก = ซ้าย}
# docks:  ชื่อ -> ระยะ m จากขอบหน้าตัวหุ่นถึงสิ่งที่ lidar เห็นตอนจอดถูกที่ (ขั้น dock: <ชื่อ>)
# approach: ชื่อ -> ระยะ m ข้างหน้าตอนเส้นทางเริ่ม/จบที่โต๊ะนั้น (ขั้น undock: <ชื่อ> ถอยจนห่างเท่านี้)
"""


def routes_file():
    """config/routes.yaml ในโฟลเดอร์ต้นฉบับ (src) — อยู่ข้าง missions.yaml (เขียนตอนตั้งหน้างาน ไม่ต้อง build)"""
    from humanoi_mission.mission import missions_file
    return os.path.join(os.path.dirname(missions_file()), "routes.yaml")


def load(path):
    data = {}
    if os.path.exists(path):
        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    return {k: dict(data.get(k) or {}) for k in SECTIONS}


def save(path, data):
    body = yaml.safe_dump({k: data.get(k) or {} for k in SECTIONS},
                          allow_unicode=True, sort_keys=False, default_flow_style=None)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(HEADER + body)
    os.replace(tmp, path)


def leg_of(item):
    """{forward: 1.2} -> ("forward", 1.2) · ระยะติดลบ = กลับทิศ (back 0.3 แทน forward -0.3)"""
    if not isinstance(item, dict) or len(item) != 1:
        raise ValueError(f"ขา {item!r} ต้องเป็น {{ทิศ: ระยะ}} หนึ่งคู่")
    (kind, amount), = item.items()
    if kind not in KINDS:
        raise ValueError(f"ไม่รู้จักทิศ '{kind}' — ใช้ {', '.join(KINDS)}")
    if not isinstance(amount, (int, float)) or isinstance(amount, bool):
        raise ValueError(f"ระยะของ {kind} ต้องเป็นตัวเลข")
    amount = float(amount)
    if kind != "turn" and amount < 0:
        kind, amount = OPPOSITE[kind], -amount
    return kind, amount


def legs_of(items):
    return [leg_of(x) for x in items or []]


def as_items(legs):
    return [{k: round(float(a), 3 if k != "turn" else 1)} for k, a in legs]


def norm_turn(deg):
    """องศา -> ช่วง (-180, 180] (180 พอดีคงเป็น +180)"""
    d = (deg + 180.0) % 360.0 - 180.0
    return 180.0 if abs(d + 180.0) < 1e-9 else d


def merge_turns(legs):
    """หมุนติดกันรวมเป็นครั้งเดียว · หมุนเกือบ 0 ตัดทิ้ง"""
    out = []
    for kind, amount in legs:
        if kind == "turn" and out and out[-1][0] == "turn":
            out[-1] = ("turn", norm_turn(out[-1][1] + amount))
        else:
            out.append((kind, norm_turn(amount) if kind == "turn" else amount))
    return [(k, a) for k, a in out if not (k == "turn" and abs(a) < DONE_DEG)
            and not (k != "turn" and a < DONE_M)]


def reverse(legs):
    """เส้นทางขากลับ: หมุน 180 -> ขาเดิมย้อนลำดับ (หมุนกลับทิศ, ขาตรงทิศเดิมเพราะหันกลับแล้ว) -> หมุน 180
    จบที่จุดเริ่มของขาไป หันหน้าทางเดิม"""
    back = [("turn", 180.0)]
    for kind, amount in reversed(legs):
        back.append((kind, -amount) if kind == "turn" else (kind, amount))
    back.append(("turn", 180.0))
    return merge_turns(back)


def reverse_name(name):
    """A_to_B -> B_to_A · อื่นๆ -> <ชื่อ>_back"""
    a, sep, b = name.partition("_to_")
    return f"{b}_to_{a}" if sep and a and b else f"{name}_back"


def ends_of(name):
    """A_to_B -> ("A", "B") (โต๊ะต้นทาง, ปลายทาง) · ชื่อแบบอื่น -> (None, None)"""
    a, sep, b = name.partition("_to_")
    return (a, b) if sep and a and b else (None, None)


def parse_text(text):
    """"forward 1.2, turn 90, forward 0.5" -> ขา"""
    legs = []
    for part in text.replace(";", ",").split(","):
        words = part.split()
        if not words:
            continue
        if len(words) != 2:
            raise ValueError(f"'{part.strip()}' ต้องเป็น '<ทิศ> <ระยะ>' เช่น forward 1.2 หรือ turn 90")
        try:
            amount = float(words[1])
        except ValueError:
            raise ValueError(f"'{words[1]}' ไม่ใช่ตัวเลข") from None
        legs.append(leg_of({words[0].lower(): amount}))
    if not legs:
        raise ValueError("ไม่มีขาเลย")
    return legs


def leg_text(kind, amount):
    return f"{TH[kind]} {amount:+.0f}°" if kind == "turn" else f"{TH[kind]} {amount:.2f} m"


def describe(legs):
    return " → ".join(leg_text(k, a) for k, a in legs) or "(ไม่มีขา)"


def remaining(kind, amount, done):
    """โดนหยุดกลางขา: ระยะที่เหลือ (หน่วยเดียวกับ amount) หรือ None = ถึงแล้ว
    done = ที่ไปได้จริงจาก base_motion (m หรือ rad ตามทิศที่หมุน) · None = ยังไม่ได้ขยับ"""
    if done is None:
        return amount
    if kind == "turn":
        left = abs(amount) - math.degrees(done)
        return None if left < DONE_DEG else math.copysign(left, amount)
    left = amount - done
    return None if left < DONE_M else left
