#!/usr/bin/env python3
"""wheel_tune — เช็ก encoder 4 ล้อ + จูนลูกบิดความเร็ว (HW-687) ให้ 4 ล้อหมุนเท่ากัน

  arm wheel-check              หมุนทีละล้อ FL FR RL RR ล้อละ 2 วิ -> ดูว่า encoder ตัวไหนนับ ทิศไหน
                               -> ได้ encoder_map / encoder_signs (บันทึกลง config/base.yaml ได้)
  arm wheel-tune [back] [rpm]  หมุนทั้ง 4 ล้อค้างไว้ แสดง rpm แต่ละล้อทุกวินาที + บอกว่าลูกบิดล้อไหนต้องหมุนขึ้น/ลง
                               rpm = ความเร็วเป้าหมาย (ไม่ใส่ = เทียบกับค่าเฉลี่ย 4 ล้อ) · Ctrl+C = หยุด + สรุป

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
def run_tune(node, back, target, seconds, win):
    params = node.bridge_params()
    if params is None:
        sys.exit("✗ ถามค่า encoder_map จาก relay_bridge ไม่ได้ — เปิด arm base ก่อน")
    cpr = params["cpr"]
    d = "B" if back else "F"
    where = f"ล้อลอยพ้นพื้น หรือบนพื้นที่โล่งพอวิ่ง {seconds:g} วิ" if seconds else "ยกฐานให้ล้อลอยพ้นพื้น"
    confirm(f"wheel-tune: หมุนทั้ง 4 ล้อ{'ถอยหลัง' if back else 'เดินหน้า'} "
            f"{f'{seconds:g} วิ' if seconds else 'จนกด Ctrl+C'} ({where})\n"
            "ระหว่างหมุนค่อยๆ ปรับลูกบิดล้อที่บอก ทีละนิด แล้วรอดูค่าใหม่ 2-3 วิ — พร้อมกด Enter ")
    node.line = "W " + " ".join([d] * 4)
    t0 = time.monotonic()
    start = end = None
    time.sleep(1.0)  # ช่วงออกตัว ไม่เอามาคิด
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
    dt = (end[0] - start[0]) / 1000.0
    total = [params["signs"][i] * (end[1][params["map"][i]] - start[1][params["map"][i]]) / cpr / dt * 60.0
             * (-1 if back else 1) for i in range(4)]
    print(f"\nสรุป {dt:.0f} วิ (ไม่นับช่วงออกตัว):")
    print_rpm(None, total, target, [b - a for a, b in zip(start[2], end[2])], None)


def print_rpm(t, rpm, target, errs, mode):
    ref = target if target else sum(rpm) / 4.0
    cells = []
    for w, v in zip(WHEELS, rpm):
        pct = (v - ref) / ref * 100.0 if abs(ref) > 1e-6 else 0.0
        hint = "✓" if abs(pct) <= OK_PCT else ("ลด" if pct > 0 else "เพิ่ม")
        cells.append(f"{w} {v:6.1f} ({pct:+5.1f}% {hint})")
    spread = (max(rpm) - min(rpm)) / ref * 100.0 if abs(ref) > 1e-6 else 0.0
    head = f"[{t:4.0f}s] " if t is not None else ""
    tail = f"เป้า {target:.1f}" if target else f"เฉลี่ย {ref:.1f}"
    print(f"{head}{'  '.join(cells)}  | {tail} rpm ต่างสุด {spread:.1f}%")
    if any(errs):
        print(f"        ! นับพลาด E1-E4 {errs} — ค่ารอบนี้อาจต่ำกว่าจริง")
    if mode not in (None, "ROS", "?"):
        print(f"        ! Mega อยู่โหมด {mode} — ROS ไม่ได้คุมล้อ")
    if all(abs(v) < 1.0 for v in rpm):
        print("        ! ไม่มีล้อหมุน — รีโมทเปิด SWA ลง ปล่อยคันโยก? ไฟมอเตอร์เปิด?")


def main():
    ap = argparse.ArgumentParser(description="เช็ก encoder + จูนความเร็ว 4 ล้อ")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check", help="หมุนทีละล้อ หา encoder_map / encoder_signs")
    t = sub.add_parser("tune", help="หมุนทั้ง 4 ล้อ แสดง rpm แต่ละล้อ")
    t.add_argument("--back", action="store_true", help="ถอยหลัง")
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
