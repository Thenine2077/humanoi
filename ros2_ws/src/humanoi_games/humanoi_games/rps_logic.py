"""กติกาเป่ายิ้งฉุบ + การนับท่าผู้เล่น + แผนขยับแขน (เป่า-ยิ้ง-ฉุบ) — ไม่ใช้ ROS ทดสอบแยกได้"""
import math
import random
from collections import Counter

CHOICES = ("rock", "paper", "scissors")
THAI = {"rock": "ค้อน", "paper": "กระดาษ", "scissors": "กรรไกร", None: "ไม่เห็นมือ"}
_BEATS = {"rock": "scissors", "scissors": "paper", "paper": "rock"}  # ท่าทางซ้ายชนะท่าทางขวา


def judge(robot, player):
    """ผลจากมุมผู้เล่น: player_win | robot_win | draw"""
    if robot == player:
        return "draw"
    return "player_win" if _BEATS[player] == robot else "robot_win"


def robot_pick(rng=random):
    return rng.choice(CHOICES)


def vote(samples, t_from, t_to, min_votes=4, min_share=0.6):
    """ท่าผู้เล่นจากหลายเฟรม: ท่าที่เห็นบ่อยสุดในช่วง [t_from, t_to] — None ถ้าเห็นมือน้อยไปหรือท่าไม่นิ่ง

    samples = [(เวลา, ท่า)] ท่าที่ไม่ใช่ rock/paper/scissors (none = ไม่เห็นมือ/ท่าอื่น) ไม่นับ
    ต้องโหวตหลายเฟรมเพราะมือที่กำลังเปลี่ยนท่า (กำ → แบ) จะถูกอ่านผิดไปบางเฟรม
    """
    seen = [g for t, g in samples if t_from <= t <= t_to and g in CHOICES]
    if len(seen) < min_votes:
        return None
    best, n = Counter(seen).most_common(1)[0]
    return best if n / len(seen) >= min_share else None


def held(samples, t_from, t_to, gesture, min_frames=5, min_share=0.8):
    """ผู้เล่นค้างท่า gesture ไว้ตลอดช่วง [t_from, t_to] ไหม (ใช้ชูนิ้วชี้ = เล่นใหม่)

    ต้องเห็นอย่างน้อย min_frames เฟรม และเป็นท่านั้น ≥ min_share — มือที่กำลังลดลง/เปลี่ยนท่า
    ผ่านท่าชี้นิ้วแค่ไม่กี่เฟรม จะไม่นับ
    """
    seen = [g for t, g in samples if t_from <= t <= t_to]
    return len(seen) >= min_frames and seen.count(gesture) / len(seen) >= min_share


def pump_plan(ready, joint, amplitude, count, speed, accel, trapezoid):
    """แผนแขนเป่า-ยิ้ง-ฉุบ: จากท่าตั้ง ยกข้อ joint ขึ้น amplitude (rad) แล้วลงกลับ count ครั้ง

    trapezoid = ar_program.programs.trapezoid (เร่ง → วิ่ง → เบรก ไม่เกิน accel ที่เฟิร์มแวร์ตามทัน)
    คืน (points, beats): points = [(t, ตำแหน่ง, ความเร็ว)] ต่อกันเป็น trajectory เดียว,
    beats = เวลาที่แขนลงถึงท่าตั้งแต่ละครั้ง (ครั้งสุดท้าย = จังหวะ "ฉุบ" ที่ออกท่า)
    """
    up = list(ready)
    up[joint] += amplitude
    points, beats, t0 = [], [], 0.0
    for _ in range(count):
        for start, target in ((ready, up), (up, ready)):
            for t, pos, vel in trapezoid(start, target, speed, accel):
                points.append((t0 + t, pos, vel))
            t0 = points[-1][0]
        beats.append(t0)
    return points, beats


def stroke_time(amplitude, speed, accel):
    """เวลาขาขึ้นหรือขาลงหนึ่งครั้ง (วินาที) — ไว้บอกผู้ใช้ว่าหนึ่งตาใช้เวลาเท่าไหร่"""
    d = abs(amplitude)
    if d >= speed * speed / accel:
        return 2 * speed / accel + (d - speed * speed / accel) / speed
    return 2 * math.sqrt(d / accel)
