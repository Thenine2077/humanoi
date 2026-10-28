#!/usr/bin/env python3
"""mission — ลำดับงานของหุ่น: หยิบกล่อง → พับแขน → รอคนยืนยัน → ฐานวิ่ง → วาง … ตามไฟล์ config/missions.yaml

  arm mission                 ดูรายชื่อภารกิจ
  arm mission demo_ab         เล่นภารกิจ (เปิดค้าง — SWD / Ctrl+C = หยุดทุกอย่าง)

ขั้นที่ใช้ได้ (หนึ่งบรรทัดต่อขั้น):
  confirm: "ข้อความ"             รอคนยืนยัน: โยก SWB บนรีโมท (หรือกด Enter ที่หน้าต่างนี้)
  program: <ชื่อ>                เล่นโปรแกรมแขน (arm_program) · "@left:ท่า" / "@right:ท่า" = ไปท่าเดียว
  hand: {left: ท่า, right: ท่า}   ท่ามือ
  pick: left                     หยิบกล่องตามกล้อง (ท่าจับจาก arm tray-teach — ต้องเปิด arm tray)
  place_back: left               วางกล่องคืนที่หยิบมา
  move: {forward: 1.5}           ฐานวิ่ง/หมุน: forward back left right (m) · turn (องศา บวก = ซ้าย)
  wait: 2.0                      รอ (วินาที)

ความปลอดภัย
  - เช็กทุกชื่อท่า/โปรแกรม/ไฟล์ท่าจับ และโปรแกรมที่ต้องเปิด ก่อนเริ่ม — ขาดอะไร = ไม่เริ่มเลย
  - ฐานไม่วิ่งถ้าแขนไม่อยู่ท่าพับ (stow ในไฟล์) · ของขวางทาง = หยุดรอ (lidar)
  - SWD โยกลง / Ctrl+C = หยุดฐาน + หยุดแขน + จบภารกิจทันที (แขนค้างท่าปัจจุบัน ไม่ตัดไฟ)
  - ขยับคันโยก / ปลด SWA = Mega ตัดฐานเอง แล้วภารกิจหยุดตาม
  ห้ามเปิดคู่กับ arm remote (SWB จะสั่งโปรแกรมซ้อน)
"""
import argparse
import math
import os
import queue
import sys
import threading
import time

import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String

from ar_program import programs
from humanoi_base.base_motion import KINDS, TH, BaseMotion, MoveError, declare
from humanoi_base.rc_arm_trigger import Switch

STEPS = ("confirm", "program", "hand", "pick", "place_back", "move", "wait")
HAND = {"left": "/hand", "right": "/right_hand"}
ARM_TH = {"left": "ซ้าย", "right": "ขวา"}
PROGRAM_TIMEOUT = 900.0


class Abort(Exception):
    pass


class StepError(Exception):
    pass


def missions_file():
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory("humanoi_mission"), "config", "missions.yaml")
    except Exception:  # noqa: BLE001
        path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "config", "missions.yaml")
    return os.path.realpath(path)


def step_of(step):
    """ขั้นในไฟล์ -> (ชนิด, ค่า)"""
    if not isinstance(step, dict) or len(step) != 1:
        raise StepError(f"ขั้น {step!r} ต้องเป็น 'ชนิด: ค่า' หนึ่งคู่")
    (kind, val), = step.items()
    if kind not in STEPS:
        raise StepError(f"ไม่รู้จักขั้น '{kind}' — ใช้ {', '.join(STEPS)}")
    return kind, val


def describe(kind, val):
    if kind == "move":
        (k, v), = val.items()
        return f"ฐาน{TH[k]} {v}{'°' if k == 'turn' else ' m'}"
    return {"confirm": lambda: f"รอยืนยัน: {val}", "program": lambda: f"แขน: {val}",
            "hand": lambda: "มือ: " + ", ".join(f"{ARM_TH[a]} {g}" for a, g in val.items()),
            "pick": lambda: f"หยิบกล่องด้วยแขน{ARM_TH.get(val, val)}",
            "place_back": lambda: f"วางกล่องคืนด้วยแขน{ARM_TH.get(val, val)}",
            "wait": lambda: f"รอ {val} วิ"}[kind]()


class Mission(Node):
    def __init__(self):
        super().__init__("mission")
        self.declare_parameter("missions_file", "")
        self.file = self.get_parameter("missions_file").value or missions_file()
        self.motion = BaseMotion(self, declare(self))
        self.lock = threading.Lock()
        self.joints, self.done, self.hand_status = {}, [], {a: [] for a in HAND}
        self.arm_state = "?"
        self.abort, self.abort_reason = threading.Event(), ""
        self.swb, self.swd, self.swb_fired = Switch(), Switch(), threading.Event()
        self.pickers, self.ex, self.keys = {}, None, queue.Queue()
        self.run_pub = self.create_publisher(String, "/arm/run", 10)
        self.stop_pub = self.create_publisher(Empty, "/arm/stop", 10)
        self.hand_pubs = {a: self.create_publisher(String, f"{t}/gesture", 10) for a, t in HAND.items()}
        self.state_pub = self.create_publisher(String, "/mission/state", 10)
        self.create_subscription(JointState, "/joint_states", self._on_js, 50)
        self.create_subscription(String, "/arm/done", self._on_done, 10)
        self.create_subscription(String, "/arm/state", lambda m: setattr(self, "arm_state", m.data), 10)
        self.create_subscription(String, "/motor_state", self._on_state, 50)
        for a, t in HAND.items():
            self.create_subscription(String, f"{t}/status", lambda m, a=a: self._on_hand(a, m), 10)

    # ---------- รับข้อมูล ----------
    def _on_js(self, msg):
        with self.lock:
            self.joints.update(zip(msg.name, msg.position))

    def _on_done(self, msg):
        with self.lock:
            self.done.append(msg.data)

    def _on_hand(self, arm, msg):
        with self.lock:
            self.hand_status[arm].append(msg.data)

    def _on_state(self, msg):
        """สวิตช์รีโมทจาก Mega ("SW:BxDy") — กติกาเดียวกับ rc_arm_trigger: นับเฉพาะจังหวะโยก ขึ้น -> ลง"""
        line = msg.data.strip()
        if not line.startswith("SW:B") or len(line) < 7 or line[5] != "D":
            return
        b, d = line[4], line[6]
        lost = b == "-" and d == "-"
        b = None if (b == "-" and not lost) else b
        d = None if (d == "-" and not lost) else d
        if d is not None and self.swd.update(d):
            self.stop_all("โยก SWD บนรีโมท")
        if b is not None and self.swb.update(b) and self.swd.pos != "D":
            self.swb_fired.set()

    # ---------- หยุด ----------
    def stop_all(self, reason):
        if not self.abort.is_set():
            self.abort_reason = reason
            self.abort.set()
        self.motion.stop_event.set()
        for p in self.pickers.values():
            p.stop_event.set()
        self.stop_pub.publish(Empty())
        self.motion.send()

    def check_abort(self):
        if self.abort.is_set():
            raise Abort(self.abort_reason)

    def sleep(self, seconds):
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            self.check_abort()
            time.sleep(0.05)

    def state(self, text):
        self.state_pub.publish(String(data=text))

    # ---------- ไฟล์ ----------
    def load(self):
        if not os.path.exists(self.file):
            raise StepError(f"ไม่มีไฟล์ภารกิจ {self.file}")
        with open(self.file, encoding="utf-8") as f:
            return yaml.safe_load(f) or {}

    def validate(self, cfg, steps):
        """ปัญหาทั้งหมดของภารกิจ (ว่าง = เริ่มได้) — เช็กก่อนขยับอะไรทั้งนั้น"""
        from humanoi_vision.tray_pick import CONFIG
        data = programs.load(programs.DEFAULT_FILE)
        hands = programs.hand_names(data)
        out, picked = [], set()
        for i, step in enumerate(steps, 1):
            try:
                kind, val = step_of(step)
            except StepError as exc:
                out.append(f"ขั้น {i}: {exc}")
                continue
            where = f"ขั้น {i} ({kind})"
            if kind == "program":
                name = str(val)
                if name.startswith("@"):
                    arm, _, pose = name[1:].rpartition(":")
                    arm = arm or "left"
                    if arm not in data["poses"] or pose not in data["poses"][arm]:
                        out.append(f"{where}: แขน{ARM_TH.get(arm, arm)}ไม่มีท่า '{pose}' — สอน: arm teach {arm} {pose}")
                elif name not in data["programs"]:
                    out.append(f"{where}: ไม่มีโปรแกรม '{name}' — สร้าง: arm program {name} ...")
            elif kind == "hand":
                if not isinstance(val, dict) or not val or any(a not in HAND for a in val):
                    out.append(f"{where}: ใช้ {{left: ท่า, right: ท่า}}")
                else:
                    out += [f"{where}: ไม่มีท่ามือ '{g}'" for g in val.values() if g not in hands]
            elif kind in ("pick", "place_back"):
                if val not in HAND:
                    out.append(f"{where}: ใช้ left หรือ right")
                elif kind == "pick":
                    picked.add(val)
                    if not os.path.exists(os.path.join(CONFIG, f"tray_grasp_{val}.yaml")):
                        out.append(f"{where}: แขน{ARM_TH[val]}ยังไม่ได้สอนท่าจับกล่อง (arm tray-teach)")
                elif val not in picked:
                    out.append(f"{where}: ยังไม่ได้หยิบด้วยแขน{ARM_TH[val]}ก่อนหน้านี้")
            elif kind == "move":
                if not isinstance(val, dict) or len(val) != 1 or next(iter(val)) not in KINDS:
                    out.append(f"{where}: ใช้ {{forward|back|left|right: m}} หรือ {{turn: องศา}}")
                elif not isinstance(next(iter(val.values())), (int, float)):
                    out.append(f"{where}: ระยะต้องเป็นตัวเลข")
            elif kind == "wait" and not (isinstance(val, (int, float)) and val >= 0):
                out.append(f"{where}: ใส่จำนวนวินาที")
        if any(step_of(s)[0] == "move" for s in steps if isinstance(s, dict) and len(s) == 1
               and next(iter(s)) in STEPS) and cfg.get("stow_check", True):
            stow = cfg.get("stow") or {}
            if set(stow) != set(HAND):
                out.append("มีขั้น move แต่ยังไม่ได้ตั้ง stow ของทั้งสองแขน (ท่าพับที่ยอมให้ฐานวิ่ง)")
            for arm, poses in stow.items():
                for pose in ([poses] if isinstance(poses, str) else poses or []):
                    if arm in data["poses"] and pose not in data["poses"][arm]:
                        out.append(f"stow: แขน{ARM_TH.get(arm, arm)}ไม่มีท่า '{pose}' — สอน: arm teach {arm} {pose}")
        return out

    def check_running(self, steps):
        """โปรแกรมที่ภารกิจนี้ต้องใช้ เปิดอยู่ครบไหม"""
        kinds = {step_of(s)[0]: step_of(s)[1] for s in steps}
        all_kinds = [step_of(s) for s in steps]
        out = []
        if "rc_arm_trigger" in self.get_node_names():
            out.append("arm remote เปิดอยู่ — SWB จะสั่งโปรแกรมซ้อน ปิดก่อน")
        if "program" in kinds and self.count_subscribers("/arm/run") == 0:
            out.append("ตัวเล่นโปรแกรมแขนยังไม่เปิด — arm player")
        for arm in {a for k, v in all_kinds if k == "hand" for a in v} | {v for k, v in all_kinds
                                                                         if k in ("pick", "place_back")}:
            if self.count_subscribers(f"{HAND[arm]}/gesture") == 0:
                out.append(f"ตัวคุมมือ{ARM_TH[arm]}ยังไม่เปิด — arm player")
        if "pick" in kinds and self.count_publishers("/tray/pose") == 0:
            out.append("ตัวหากล่องยังไม่เปิด — arm tray")
        if "move" in kinds:
            if self.count_publishers(self.motion.cfg["odom_topic"]) == 0:
                out.append(f"ไม่มี odometry ({self.motion.cfg['odom_topic']}) — arm base")
            if self.motion.cfg["guard"] and self.count_publishers(self.motion.cfg["scan_topic"]) == 0:
                out.append(f"ไม่มี lidar ({self.motion.cfg['scan_topic']})")
        return out

    # ---------- ขั้นต่างๆ ----------
    def picker(self, arm):
        if arm not in self.pickers:
            from humanoi_vision.tray_pick import TrayPick
            p = TrayPick(f"tray_pick_{arm}", parameter_overrides=[
                Parameter("arm", value=arm), Parameter("hand_topic", value=f"{HAND[arm]}/gesture")])
            self.ex.add_node(p)
            self.pickers[arm] = p
        return self.pickers[arm]

    def do_confirm(self, text):
        print(f"  ⏸ {text} — โยก SWB บนรีโมท (หรือกด Enter ที่หน้าต่างนี้)", flush=True)
        self.swb_fired.clear()
        while not self.keys.empty():
            self.keys.get_nowait()
        while True:
            self.check_abort()
            if self.swb_fired.wait(0.1):
                return
            try:
                self.keys.get_nowait()
                return
            except queue.Empty:
                pass

    def wait_player_idle(self, timeout=5.0):
        """ตัวเล่นโปรแกรมรายงานผลก่อนกลับเป็น idle นิดหนึ่ง — สั่งต่อทันทีจะโดน rejected:busy"""
        t_end = time.monotonic() + timeout
        while not self.arm_state.startswith("idle") and time.monotonic() < t_end:
            self.check_abort()
            time.sleep(0.05)

    def do_program(self, name):
        name = str(name)
        self.wait_player_idle()
        with self.lock:
            self.done.clear()
        self.run_pub.publish(String(data=name))
        t_end = time.monotonic() + PROGRAM_TIMEOUT
        while True:
            self.check_abort()
            with self.lock:
                got = [d for d in self.done if d.startswith(name + ":")]
            if got:
                if got[-1].endswith(":ok"):
                    self.wait_player_idle()
                    return
                raise StepError(f"แขนรายงาน {got[-1]}")
            if time.monotonic() > t_end:
                raise StepError(f"โปรแกรม {name} ไม่จบใน {PROGRAM_TIMEOUT:.0f} วิ")
            time.sleep(0.05)

    def do_hand(self, val):
        for arm, g in val.items():
            with self.lock:
                self.hand_status[arm].clear()
            self.hand_pubs[arm].publish(String(data=str(g)))
        for arm, g in val.items():
            t_end = time.monotonic() + 3.0
            while True:
                self.check_abort()
                with self.lock:
                    got = list(self.hand_status[arm])
                if any(s.startswith("error") for s in got):
                    raise StepError(f"มือ{ARM_TH[arm]}: {[s for s in got if s.startswith('error')][-1]}")
                if f"gesture:{g}" in got:
                    break
                if time.monotonic() > t_end:
                    raise StepError(f"มือ{ARM_TH[arm]}ไม่ตอบรับท่า {g} ใน 3 วินาที")
                time.sleep(0.05)
        self.sleep(max(programs.hand_duration(str(g)) for g in val.values()) + 0.5)

    def do_pick(self, arm, back=False):
        from humanoi_vision.tray_pick import PickError
        p = self.picker(arm)
        try:
            if back:
                p.place_back()
                return
            pl = p.plan()
            print(p.describe(pl), flush=True)
            p.pick(pl)
        except PickError as exc:
            self.check_abort()
            raise StepError(str(exc))

    def check_stow(self, cfg):
        """แขนทั้งสองต้องอยู่ท่าพับ (stow) ก่อนฐานวิ่ง — ศูนย์ถ่วงต่ำ ไม่ยื่นออกนอกตัวฐาน"""
        if not cfg.get("stow_check", True):
            return
        tol = math.radians(float(cfg.get("stow_tolerance_deg", 5.0)))
        data = programs.load(programs.DEFAULT_FILE)
        with self.lock:
            js = dict(self.joints)
        for arm, poses in cfg["stow"].items():
            names = programs.joint_names(arm)
            if any(n not in js for n in names):
                raise StepError(f"ไม่มี joint_states ของแขน{ARM_TH[arm]} — ฐานไม่วิ่ง (arm start เปิดอยู่ไหม)")
            best = None
            for pose in ([poses] if isinstance(poses, str) else poses):
                errs = [abs(js[n] - t) for n, t in zip(names, data["poses"][arm][pose])]
                worst = max(range(6), key=lambda k: errs[k])
                if best is None or errs[worst] < best[0]:
                    best = (errs[worst], pose, worst)
            if best[0] > tol:
                raise StepError(f"แขน{ARM_TH[arm]}ไม่อยู่ท่าพับ '{best[1]}' (ข้อ {best[2] + 1} ห่าง "
                                f"{math.degrees(best[0]):.0f}°) — ฐานไม่วิ่ง")

    def do_move(self, cfg, val):
        self.check_stow(cfg)
        (kind, amount), = val.items()
        try:
            res = self.motion.move(kind, amount, log=lambda s: print(s, flush=True))
        except MoveError as exc:
            self.check_abort()
            raise StepError(str(exc))
        print("  ✓ " + self.motion.describe(res), flush=True)

    # ---------- เล่น ----------
    def list(self):
        cfg = self.load()
        print(f"ภารกิจใน {self.file}:")
        for name, steps in (cfg.get("missions") or {}).items():
            print(f"  {name:<14} {len(steps)} ขั้น")
        print("เล่น: arm mission <ชื่อ>")

    def run(self, name):
        cfg = self.load()
        steps = (cfg.get("missions") or {}).get(name)
        if not steps:
            print(f"✗ ไม่มีภารกิจ '{name}' ใน {self.file}")
            return 1
        time.sleep(1.5)  # ให้เห็นโหนดอื่นๆ ก่อนเช็ก (discovery)
        problems = self.validate(cfg, steps)
        if not problems:
            problems = self.check_running(steps)
        if problems:
            print(f"✗ เริ่มภารกิจ '{name}' ไม่ได้:")
            for p in problems:
                print(f"  - {p}")
            return 1
        for kind, val in map(step_of, steps):
            if kind == "pick":
                self.picker(val)  # สร้างก่อน ให้เก็บตำแหน่งกล่องจากกล้องไว้
        print(f"ภารกิจ '{name}' — {len(steps)} ขั้น · SWD / Ctrl+C = หยุดทุกอย่าง")
        for i, step in enumerate(steps, 1):
            kind, val = step_of(step)
            text = describe(kind, val)
            self.state(f"running:{name}:{i}/{len(steps)}:{text}")
            print(f"[{i}/{len(steps)}] {text}", flush=True)
            try:
                self.check_abort()
                if kind == "confirm":
                    self.do_confirm(val)
                elif kind == "program":
                    self.do_program(val)
                elif kind == "hand":
                    self.do_hand(val)
                elif kind in ("pick", "place_back"):
                    self.do_pick(val, back=kind == "place_back")
                elif kind == "move":
                    self.do_move(cfg, val)
                else:
                    self.sleep(float(val))
            except Abort as exc:
                print(f"✗ หยุดภารกิจ: {exc} — ฐานและแขนหยุดอยู่กับที่", flush=True)
                self.state(f"stopped:{name}:{i}")
                return 1
            except StepError as exc:
                self.stop_all(f"ขั้น {i} ไม่สำเร็จ")
                print(f"✗ ขั้น {i} ไม่สำเร็จ: {exc} — หยุดภารกิจ (ฐานและแขนหยุดอยู่กับที่)", flush=True)
                self.state(f"error:{name}:{i}:{exc}")
                return 1
        print(f"✓ จบภารกิจ '{name}'")
        self.state(f"done:{name}")
        return 0


def main():
    ap = argparse.ArgumentParser(description="เล่นภารกิจตามไฟล์ config/missions.yaml")
    ap.add_argument("name", nargs="?", help="ชื่อภารกิจ (ไม่ใส่ = ดูรายชื่อ)")
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    # Ctrl+C ต้องมาถึงโค้ดนี้ (ไม่ให้ rclpy ปิด ROS ก่อน) จะได้ส่งคำสั่งหยุดฐาน/แขนออกไปทัน
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Mission()
    ex = SingleThreadedExecutor()  # callback สั้นทั้งหมด — รอผลจากเธรดหลัก (เหมือน tray_pick)
    ex.add_node(node)
    node.ex = ex
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()

    def read_keys():
        for line in sys.stdin:
            node.keys.put(line)
    threading.Thread(target=read_keys, daemon=True).start()
    rc = 0
    try:
        if args.name:
            rc = node.run(args.name)
        else:
            node.list()
    except StepError as exc:
        print(f"✗ {exc}")
        rc = 1
    except KeyboardInterrupt:
        node.stop_all("Ctrl+C")
        print("\n✗ หยุดภารกิจ (Ctrl+C) — ฐานและแขนหยุดอยู่กับที่")
        rc = 1
    finally:
        node.motion.halt(0.3)
        ex.shutdown(timeout_sec=1.0)
        spin.join(timeout=1.0)
        for p in list(node.pickers.values()):
            p.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
