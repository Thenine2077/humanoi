#!/usr/bin/env python3
"""wheel_tune — เช็ก encoder 4 ล้อ + จูนลูกบิดความเร็ว (HW-687) ให้ 4 ล้อหมุนเท่ากัน

  arm wheel-check              หมุนทีละล้อ FL FR RL RR ล้อละ 2 วิ -> ดูว่า encoder ตัวไหนนับ ทิศไหน
                               -> ได้ encoder_map / encoder_signs (บันทึกลง config/base.yaml ได้)
  arm wheel-tune [back] [rpm]  หมุนทั้ง 4 ล้อค้างไว้ แสดง rpm แต่ละล้อทุกวินาที + บอกว่าลูกบิดล้อไหนต้องหมุนขึ้น/ลง
                               rpm = ความเร็วเป้าหมาย (ไม่ใส่ = เทียบกับค่าเฉลี่ย 4 ล้อ) · Ctrl+C = หยุด + สรุป
  arm wheel-tune both [rpm]    มอเตอร์มักหมุนสองทิศเร็วไม่เท่ากัน แต่ลูกบิดตัวเดียวคุมทั้งสองทิศ: วัดเดินหน้า+ถอย
                               ครั้งเดียว (ทิศละ 3 วิ) -> คิดเป้าเดินหน้าแยกล้อ (ล้อที่ถอยช้ากว่าเพื่อน เดินหน้าต้อง
                               เร็วกว่าเพื่อนชดเชย) -> หมุนเดินหน้าค้าง จูนตามเป้า อัปเดตทุกวิเหมือน tune ปกติ

ต้องมี: relay_bridge เปิดอยู่ (arm base) + เฟิร์มแวร์ humanoi_base_mega ที่ส่ง ENC
        รีโมทเปิด, SWA ลง, ปล่อยคันโยก (Mega อยู่โหมด ROS — ไม่งั้นล้อไม่หมุน)
ยกฐานให้ล้อลอยพ้นพื้นก่อน (wheel-tune บนพื้น ใส่ --seconds จำกัดเวลา)
สั่งล้อผ่าน /wheel_cmd ("W F F F F") ข้าม mixing — relay_bridge หยุดล้อเองถ้าโปรแกรมนี้ตาย (cmd_timeout)
"""
import argparse
import collections
import os
import sys
import threading
import time

import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Int64MultiArray, String

WHEELS = ("FL", "FR", "RL", "RR")
TH = {"FL": "หน้าซ้าย", "FR": "หน้าขวา", "RL": "หลังซ้าย", "RR": "หลังขวา"}
STOP = "W S S S S"
OK_PCT = 1.5        # ต่างจากเป้าไม่เกินนี้ (%) = พอแล้ว
SPIN_S = 2.0        # wheel-check: หมุนล้อละกี่วินาที
COAST_S = 1.0       # รอล้อหยุดหมุนก่อนอ่านค่า
MIN_COUNTS = 50     # ล้อที่สั่งต้องนับได้อย่างน้อยเท่านี้ ถึงถือว่าหมุน/ต่อ encoder แล้ว
CROSS = 0.2         # encoder อื่นนับเกินสัดส่วนนี้ของตัวหลัก = สายสลับ/สัญญาณรบกวน
SPINUP_S = 1.0      # ช่วงออกตัว ไม่เอามาคิด
BOTH_PHASE_S = 3.0  # wheel-tune both: วัดทิศละกี่วินาที
REVERSE_PAUSE_S = 0.8  # หยุดรอก่อนกลับทิศ — relay กลับขั้วตอนมอเตอร์ยังหมุน = กระแสกระชาก
SAME_PCT = 3.0      # ทุกล้อต่างระหว่างสองทิศพอๆ กัน (ห่างกันไม่เกินนี้ %) = วิ่งตรงทั้งสองทิศ


def config_file():
    """config/base.yaml ของแพ็กเกจ (symlink-install = ไฟล์ใน src)"""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory("humanoi_base"), "config", "base.yaml")
    except Exception:  # noqa: BLE001
        path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "config", "base.yaml")
    return os.path.realpath(path)


class Tuner(Node):
    def __init__(self):
        super().__init__("wheel_tune")
        self.cmd_pub = self.create_publisher(String, "wheel_cmd", 10)
        self.create_subscription(Int64MultiArray, "encoder_raw", self.on_raw, 50)
        self.create_subscription(String, "motor_state", self.on_state, 50)
        self.line = STOP
        self.lock = threading.Lock()
        self.samples = collections.deque(maxlen=2000)  # (t_ms Mega, [c1..c4], [e1..e4])
        self.mode = "?"
        self.create_timer(0.05, lambda: self.cmd_pub.publish(String(data=self.line)))

    def on_raw(self, msg):
        d = list(msg.data)
        if len(d) == 9:
            with self.lock:
                if self.samples and d[0] < self.samples[-1][0]:  # Mega รีเซ็ต
                    self.samples.clear()
                self.samples.append((d[0], d[1:5], d[5:9]))

    def on_state(self, msg):
        s = msg.data
        if s.startswith("MODE:"):
            self.mode = s[5:]
        elif s.startswith("RC CH1"):  # บรรทัดสถานะทุก 0.5 วิ "... | ROS       | FL S ..."
            for key, mode in (("| ROS", "ROS"), ("| RC", "RC"), ("| NO SIGNAL", "NO_SIGNAL"),
                              ("| disarmed", "DISARMED"), ("| SWA up first", "DISARMED")):
                if key in s:
                    self.mode = mode
                    break

    def latest(self):
        with self.lock:
            return self.samples[-1] if self.samples else None

    def window(self, seconds):
        """(dt s, Δค่านับ E1-E4, Δนับพลาด) ช่วง seconds ล่าสุด — ใช้เวลาของ Mega"""
        with self.lock:
            if len(self.samples) < 2:
                return None
            last = self.samples[-1]
            first = next((s for s in self.samples if s[0] >= last[0] - seconds * 1000), self.samples[0])
        dt = (last[0] - first[0]) / 1000.0
        if dt <= 0:
            return None
        return dt, [a - b for a, b in zip(last[1], first[1])], [a - b for a, b in zip(last[2], first[2])]

    def bridge_params(self):
        """encoder_map / encoder_signs / counts_per_rev ที่ relay_bridge ใช้อยู่ (ถามไม่ได้ = None)"""
        client = AsyncParameterClient(self, "relay_bridge")
        if not client.wait_for_services(timeout_sec=3.0):
            return None
        fut = client.get_parameters(["encoder_map", "encoder_signs", "counts_per_rev"])
        t_end = time.monotonic() + 3.0
        while not fut.done() and time.monotonic() < t_end:
            time.sleep(0.05)
        if not fut.done() or fut.result() is None:
            return None
        vals = [parameter_value_to_python(v) for v in fut.result().values]
        return {"map": [int(v) for v in vals[0]], "signs": [int(v) for v in vals[1]], "cpr": float(vals[2])}


def wait_ready(node):
    t_end = time.monotonic() + 4.0
    while node.latest() is None and time.monotonic() < t_end:
        time.sleep(0.1)
    if node.latest() is None:
        sys.exit("✗ ไม่ได้ค่า encoder (/encoder_raw) — เปิด relay_bridge แล้วหรือยัง (arm base) "
                 "และ Mega แฟลชเฟิร์มแวร์ humanoi_base_mega ตัวที่ส่ง ENC หรือยัง")
    if node.mode not in ("ROS", "?"):
        print(f"! Mega อยู่โหมด {node.mode} — ล้อจะไม่หมุนจนกว่าจะเป็นโหมด ROS: เปิดรีโมท, SWA ลง, ปล่อยคันโยก")


def stop(node):
    node.line = STOP
    time.sleep(0.3)  # ให้ timer ส่ง STOP ออกไปสักสองสามครั้ง


def confirm(text):
    try:
        input(text)
    except EOFError:
        sys.exit(1)


def ask_yes(text, default=True):
    try:
        a = input(text).strip().lower()
    except EOFError:  # ไม่มีคนตอบ — ไม่เดาแทน (ไม่บันทึก ไม่ทำต่อ)
        sys.exit("\n✗ ไม่มีคำตอบ — หยุด")
    return default if not a else a in ("y", "yes", "ใช่", "ช")


# ---------------------------------------------------------------- check
def run_check(node):
    params = node.bridge_params()
    confirm("wheel-check: หมุนทีละล้อ FL FR RL RR ล้อละ 2 วิ (เดินหน้า)\n"
            "ยกฐานให้ล้อลอยพ้นพื้นแล้ว ดูล้อที่หมุนไปด้วย — พร้อมกด Enter (ยกเลิก Ctrl+C) ")
    enc_map, signs, problems = [], [], []
    for i, w in enumerate(WHEELS):
        line = ["S"] * 4
        line[i] = "F"
        before = node.latest()
        print(f"  {w} ({TH[w]}) หมุน...", end="", flush=True)
        node.line = "W " + " ".join(line)
        time.sleep(SPIN_S)
        node.line = STOP
        time.sleep(COAST_S)
        after = node.latest()
        d = [a - b for a, b in zip(after[1], before[1])]
        e = [a - b for a, b in zip(after[2], before[2])]
        j = max(range(4), key=lambda k: abs(d[k]))
        if node.mode not in ("ROS", "?"):
            print(f" ✗ Mega อยู่โหมด {node.mode} — ล้อไม่ได้รับคำสั่ง (รีโมทเปิด SWA ลง ปล่อยคันโยก)")
            return
        if abs(d[j]) < MIN_COUNTS:
            print(f" ✗ ไม่มี encoder ไหนนับ (สูงสุด E{j + 1} {d[j]:+d}) — ล้อ{TH[w]}หมุนไหม? ไม่หมุน = relay/มอเตอร์/"
                  "ลูกบิด · หมุน = encoder ล้อนี้ไม่ได้ต่อ/ไม่มีไฟ")
            problems.append(w)
            enc_map.append(None)
            signs.append(1)
            continue
        forward = ask_yes(f" E{j + 1} นับ {d[j]:+d} — ล้อหมุนไปทางเดินหน้าไหม? [Y/n] ")
        # ทิศของ encoder เทียบกับการหมุนจริงของล้อ — ใช้ได้ทั้งก่อนและหลังแก้ INVERT ของมอเตอร์
        sign = (1 if d[j] > 0 else -1) * (1 if forward else -1)
        enc_map.append(j)
        signs.append(sign)
        if not forward:
            print(f"    ! ล้อ{TH[w]}หมุนกลับทาง — แก้ INVERT ของ {w} ในเฟิร์มแวร์ humanoi_base_mega "
                  "(ค่า encoder ที่ได้ใช้ได้เลย ไม่ต้องเช็กใหม่)")
        others = [f"E{k + 1} {d[k]:+d}" for k in range(4) if k != j and abs(d[k]) > CROSS * abs(d[j])]
        if others:
            problems.append(w)
            print(f"    ! encoder ตัวอื่นนับด้วย ({', '.join(others)}) — สาย A/B สลับข้ามล้อ หรือสัญญาณรบกวน")
        if any(e):
            print(f"    ! นับพลาด {e} — สาย/ไฟ encoder หรือบอร์ด opto ช้าไป (ลอง ENC_EDGES = 2)")
    stop(node)
    if None in enc_map or sorted(enc_map) != [0, 1, 2, 3]:
        print(f"\n✗ ได้ encoder_map {enc_map} — ยังไม่ครบ/ซ้ำ ล้อละหนึ่งตัวไม่ได้ แก้สายแล้วเช็กใหม่")
        return
    print(f"\nencoder_map: {enc_map}    (ล้อ FL FR RL RR ใช้ E{', E'.join(str(j + 1) for j in enc_map)})")
    print(f"encoder_signs: {signs}")
    if params and params["map"] == enc_map and params["signs"] == signs:
        print("✓ ตรงกับที่ relay_bridge ใช้อยู่แล้ว")
        return
    if params:
        print(f"(relay_bridge ใช้อยู่: encoder_map {params['map']} · encoder_signs {params['signs']})")
    if problems:
        print("แก้ปัญหาข้างบนก่อน แล้วค่อยบันทึก")
        return
    if ask_yes(f"บันทึกลง {config_file()} ? [Y/n] "):
        save_encoder_config(enc_map, signs)
        print("บันทึกแล้ว — ปิด/เปิด arm base ใหม่ให้ relay_bridge อ่านค่า")


def save_encoder_config(enc_map, signs):
    path = config_file()
    with open(path, encoding="utf-8") as f:
        text = f.read()
    data = yaml.safe_load(text)
    p = data["relay_bridge"]["ros__parameters"]
    p["encoder_map"], p["encoder_signs"] = enc_map, signs
    # แก้แค่สองบรรทัด คอมเมนต์ในไฟล์อยู่ครบ
    out = []
    for ln in text.splitlines():
        key = ln.strip().split(":")[0]
        if key in ("encoder_map", "encoder_signs"):
            indent = ln[:len(ln) - len(ln.lstrip())]
            comment = ln[ln.index("#"):] if "#" in ln else ""
            ln = f"{indent}{key}: {p[key]}" + (f"  {comment}" if comment else "")
        out.append(ln)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write("\n".join(out) + "\n")
    os.replace(tmp, path)


# ---------------------------------------------------------------- tune
def run_tune(node, back, target, seconds, win, ask=True):
    """target: rpm เดียวทุกล้อ / 0 = เทียบค่าเฉลี่ย 4 ล้อ / list = เป้าแยกล้อ (จาก both)"""
    params = node.bridge_params()
    if params is None:
        sys.exit("✗ ถามค่า encoder_map จาก relay_bridge ไม่ได้ — เปิด arm base ก่อน")
    cpr = params["cpr"]
    d = "B" if back else "F"
    where = f"ล้อลอยพ้นพื้น หรือบนพื้นที่โล่งพอวิ่ง {seconds:g} วิ" if seconds else "ยกฐานให้ล้อลอยพ้นพื้น"
    if ask:
        confirm(f"wheel-tune: หมุนทั้ง 4 ล้อ{'ถอยหลัง' if back else 'เดินหน้า'} "
                f"{f'{seconds:g} วิ' if seconds else 'จนกด Ctrl+C'} ({where})\n"
                "ระหว่างหมุนค่อยๆ ปรับลูกบิดล้อที่บอก ทีละนิด แล้วรอดูค่าใหม่ 2-3 วิ — พร้อมกด Enter ")
    node.line = "W " + " ".join([d] * 4)
    t0 = time.monotonic()
    start = end = None
    time.sleep(SPINUP_S)
    try:
        while not seconds or time.monotonic() - t0 < seconds:
            time.sleep(1.0)
            w = node.window(win)
            if w is None:
                continue
            if start is None:
                start = node.latest()
            dt, dc, de = w
            rpm = [params["signs"][i] * dc[params["map"][i]] / cpr / dt * 60.0 for i in range(4)]
            if back:
                rpm = [-v for v in rpm]
            print_rpm(time.monotonic() - t0, rpm, target, de, node.mode)
    except KeyboardInterrupt:
        pass
    finally:
        end = node.latest()  # ก่อนสั่งหยุด — ช่วงล้อหยุดไม่เอามาเฉลี่ย
        stop(node)
    if start is None or end is None or end[0] <= start[0]:
        return
    print(f"\nสรุป {(end[0] - start[0]) / 1000.0:.0f} วิ (ไม่นับช่วงออกตัว):")
    print_rpm(None, wheel_rpm(params, start, end, back), target, [b - a for a, b in zip(start[2], end[2])], None)


def wheel_rpm(params, start, end, back):
    """rpm ล้อ FL FR RL RR ระหว่างตัวอย่าง /encoder_raw start-end (ถอย = กลับเครื่องหมายให้เป็นบวก)"""
    k = 60.0 / params["cpr"] / ((end[0] - start[0]) / 1000.0) * (-1 if back else 1)
    return [params["signs"][i] * (end[1][params["map"][i]] - start[1][params["map"][i]]) * k for i in range(4)]


def spin_phase(node, params, back, secs):
    """หมุนทั้ง 4 ล้อทิศเดียว secs วิ แล้วหยุดรอให้นิ่ง -> (rpm, นับพลาด) ไม่นับช่วงออกตัว · ไม่ได้ค่า = (None, None)"""
    node.line = "W " + " ".join(["B" if back else "F"] * 4)
    spinup = min(SPINUP_S, secs / 2.0)
    time.sleep(spinup)
    start = node.latest()
    time.sleep(secs - spinup)
    end = node.latest()
    node.line = STOP
    time.sleep(REVERSE_PAUSE_S)
    if start is None or end is None or end[0] <= start[0]:
        return None, None
    return wheel_rpm(params, start, end, back), [b - a for a, b in zip(start[2], end[2])]


def run_both(node, target, seconds, win):
    """วัดเดินหน้า/ถอยหลังครั้งเดียว -> เป้าเดินหน้าแยกล้อ -> หมุนเดินหน้าค้างให้จูนตามเป้า (อัปเดตทุกวิ)"""
    params = node.bridge_params()
    if params is None:
        sys.exit("✗ ถามค่า encoder_map จาก relay_bridge ไม่ได้ — เปิด arm base ก่อน")
    phase = seconds or BOTH_PHASE_S
    then = ("แล้วหยุด (บนพื้นที่โล่ง = ไปแล้วกลับที่เดิม)" if seconds
            else "แล้วหมุนเดินหน้าค้างให้จูนตามเป้าแยกล้อ (ยกฐานให้ล้อลอยพ้นพื้น)")
    confirm(f"wheel-tune both: วัดเดินหน้า {phase:g} วิ + ถอยหลัง {phase:g} วิ {then}\n"
            "ช่วงวัดยังไม่ต้องแตะลูกบิด — พร้อมกด Enter ")
    fwd, ef = spin_phase(node, params, False, phase)
    back, eb = spin_phase(node, params, True, phase)
    if fwd is None or back is None:
        print("✗ ไม่ได้ค่า encoder — relay_bridge (arm base) ยังเปิดอยู่ไหม")
        return
    errs = [a + b for a, b in zip(ef, eb)]
    if min(fwd + back) < 5.0:
        print_both(fwd, back, None, errs, node.mode)
        print("✗ มีล้อไม่หมุน (ต่ำกว่า 5 rpm) — เช็กล้อนั้นก่อน")
        return
    goal = both_targets(fwd, back, target)
    print_both(fwd, back, goal, errs, node.mode)
    if all(abs(f - g) / g * 100.0 <= OK_PCT for f, g in zip(fwd, goal)):
        print("✓ เดินหน้าทุกล้อตรงเป้าแล้ว — ไม่ต้องปรับลูกบิด")
        return
    if seconds:
        print("ปรับลูกบิดตามเป้าเดินหน้าข้างบนทีละนิด แล้วรัน both ซ้ำเช็กผล")
        return
    print("\nหมุนเดินหน้าค้าง — ปรับลูกบิดให้ขึ้น ✓ ตามเป้าแยกล้อ แล้ว Ctrl+C · จากนั้นรัน both อีกครั้งเช็กผล")
    run_tune(node, False, goal, 0.0, win, ask=False)


def both_targets(fwd, back, target):
    """เป้า rpm เดินหน้าแยกล้อ: ล้อที่ถอยช้ากว่าเพื่อนต้องเดินหน้าเร็วกว่าเพื่อน ให้ค่าเฉลี่ยสองทิศ 4 ล้อเท่ากัน
    (สัดส่วนถอย/เดินหน้าของแต่ละล้อถือว่าคงที่) · เป้าเดินหน้าเฉลี่ย = target หรือที่หมุนอยู่ตอนนี้"""
    k = [1.0 + b / f for f, b in zip(fwd, back)]  # เฉลี่ยสองทิศ = เดินหน้า x k / 2
    mean_fwd = target or sum(fwd) / 4.0
    c = mean_fwd / (sum(1.0 / x for x in k) / 4.0)
    return [c / x for x in k]


def print_both(fwd, back, goal, errs, mode):
    """rpm สองทิศของแต่ละล้อ + ถอยต่างจากเดินหน้ากี่ % + เป้าเดินหน้าแยกล้อ"""
    print()
    for name, vals in (("เดินหน้า", fwd), ("ถอยหลัง", back)):
        mean = sum(vals) / 4.0
        spread = (max(vals) - min(vals)) / mean * 100.0 if mean > 1e-6 else 0.0
        print(f"{name:<9}" + "  ".join(f"{w} {v:6.1f}" for w, v in zip(WHEELS, vals))
              + f"  | 4 ล้อต่างกันสุด {spread:.1f}%")
    gap = [(b - f) / f * 100.0 if f > 1e-6 else 0.0 for f, b in zip(fwd, back)]
    print("ถอยต่าง   " + "  ".join(f"{w} {g:+5.1f}%" for w, g in zip(WHEELS, gap)) + "  (เทียบเดินหน้า)")
    print_warnings(fwd + back, errs, mode)
    if goal is None:
        return
    width = max(gap) - min(gap)
    if width <= SAME_PCT:
        print(f"  → ทุกล้อถอย{'ช้า' if sum(gap) < 0 else 'เร็ว'}กว่าเดินหน้าพอๆ กัน — วิ่งตรงทั้งสองทิศ "
              "(แค่ถอยกับเดินหน้าเร็วไม่เท่ากัน ไม่เป็นไร) · จูนเดินหน้า 4 ล้อให้เท่ากันพอ")
    else:
        odd = WHEELS[max(range(4), key=lambda i: abs(gap[i] - sum(gap) / 4.0))]
        print(f"  → {odd} ({TH[odd]}) ถอยต่างจากล้ออื่น — ตั้งเดินหน้าล้อนี้ให้ต่างจากเพื่อนชดเชย "
              f"แล้วแต่ละทิศ 4 ล้อจะต่างกันราว {width / 2:.1f}% (ลูกบิดเดียวทำได้แค่นี้)")
    print("เป้าเดินหน้า " + "  ".join(f"{w} {g:5.1f}" for w, g in zip(WHEELS, goal)))


def print_rpm(t, rpm, target, errs, mode):
    """target: rpm เดียวทุกล้อ / 0 = เทียบค่าเฉลี่ย 4 ล้อ / list = เป้าแยกล้อ"""
    per_wheel = isinstance(target, (list, tuple))
    refs = list(target) if per_wheel else [target or sum(rpm) / 4.0] * 4
    cells = []
    for w, v, ref in zip(WHEELS, rpm, refs):
        pct = (v - ref) / ref * 100.0 if abs(ref) > 1e-6 else 0.0
        hint = "✓" if abs(pct) <= OK_PCT else ("ลด" if pct > 0 else "เพิ่ม")
        cells.append(f"{w} {v:6.1f}" + (f"→{ref:.1f}" if per_wheel else "") + f" ({pct:+5.1f}% {hint})")
    if per_wheel:
        tail = "เป้าแยกล้อ (ชดเชยทิศถอย)"
    else:
        ref = refs[0]
        spread = (max(rpm) - min(rpm)) / ref * 100.0 if abs(ref) > 1e-6 else 0.0
        tail = f"{'เป้า' if target else 'เฉลี่ย'} {ref:.1f} rpm ต่างสุด {spread:.1f}%"
    head = f"[{t:4.0f}s] " if t is not None else ""
    print(f"{head}{'  '.join(cells)}  | {tail}")
    print_warnings(rpm, errs, mode)


def print_warnings(rpm, errs, mode):
    if any(errs):
        print(f"        ! นับพลาด E1-E4 {errs} — ค่ารอบนี้อาจต่ำกว่าจริง")
    if mode not in (None, "ROS", "?"):
        print(f"        ! Mega อยู่โหมด {mode} — ROS ไม่ได้คุมล้อ")
    if all(abs(v) < 1.0 for v in rpm):
        print("        ! ไม่มีล้อหมุน — รีโมทเปิด SWA ลง ปล่อยคันโยก? ไฟมอเตอร์เปิด? "
              "กันชน lidar ห้ามอยู่? (ยกล้อ: arm base noguard)")


def main():
    ap = argparse.ArgumentParser(description="เช็ก encoder + จูนความเร็ว 4 ล้อ")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check", help="หมุนทีละล้อ หา encoder_map / encoder_signs")
    t = sub.add_parser("tune", help="หมุนทั้ง 4 ล้อ แสดง rpm แต่ละล้อ")
    t.add_argument("--back", action="store_true", help="ถอยหลัง")
    t.add_argument("--both", action="store_true", help="วัดเดินหน้า+ถอยครั้งเดียว แล้วจูนเดินหน้าตามเป้าแยกล้อ (--seconds = วัดอย่างเดียว ทิศละกี่วิ)")
    t.add_argument("--target", type=float, default=0.0, help="rpm เป้าหมาย (0 = เทียบค่าเฉลี่ย)")
    t.add_argument("--seconds", type=float, default=0.0, help="หมุนกี่วินาที (0 = จนกด Ctrl+C)")
    t.add_argument("--window", type=float, default=1.0, help="วัด rpm จากช่วงกี่วินาทีล่าสุด")
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    # Ctrl+C ต้องมาถึงโค้ดนี้ (ไม่ให้ rclpy ปิด context ก่อน) จะได้ส่ง STOP ได้
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Tuner()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        wait_ready(node)
        if args.cmd == "check":
            run_check(node)
        elif args.both:
            run_both(node, args.target, args.seconds, args.window)
        else:
            run_tune(node, args.back, args.target, args.seconds, args.window)
    except KeyboardInterrupt:
        print()
    finally:
        stop(node)
        executor.shutdown()  # หยุดเธรด spin ก่อนปิด ไม่งั้นโปรแกรมล้มตอนออก
        spinner.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
