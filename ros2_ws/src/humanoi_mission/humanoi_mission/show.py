#!/usr/bin/env python3
"""show — ตัวคุมโชว์ให้คนคุมเล่นทีละรอบ (หยิบกล่อง A → B ฯลฯ) ตาม config/shows.yaml — ต่อยอด mission

  arm show                  ดูรายชื่อโชว์ เส้นทาง ระยะจอด
  arm show check <ชื่อ>     เช็กระบบอย่างเดียว ไม่ขยับ
  arm show <ชื่อ>           เปิดโชว์ (หน้าต่างค้าง) — รอคนคุมสั่งเริ่มทีละรอบ

คุมด้วยรีโมทอย่างเดียวได้ (ไม่ต้องมีโน้ตบุ๊ก / เน็ต):
  SWB โยกลง = เริ่มรอบ · ยืนยันขั้นรอ · ทำต่อหลังหยุด / ลองใหม่หลังพัง
  SWD โยกลง = หยุดทันที (ฐาน + แขนค้างท่า ไม่ตัดไฟ) · โยกอีกครั้งตอนหยุดอยู่ = จบรอบนี้
แป้นพิมพ์: Enter = SWB · s = SWD · q = ออก   ·   topic /show/cmd (String): go | stop | quit

ขั้นในรอบ = ขั้นของ mission (confirm program hand pick place_back move wait) + ขั้นเพิ่ม:
  say: "ข้อความ"        ขึ้นข้อความให้คนดู (/show/say)
  route: A_to_B         ฐานวิ่งตามเส้นทางใน config/routes.yaml (arm show route-record / route-set)
  dock: B               เข้าจอดหน้าโต๊ะ — เดินหน้าจน lidar เห็นโต๊ะห่างเท่าที่จดไว้ (arm show dock-save B)
  undock: B             ถอยออกจากโต๊ะ B กลับไปปลายเส้นทาง (ห่างโต๊ะเท่าตอน route-record) ให้ขาต่อไปตรงทาง
                        ไม่มีระยะจาก route-record = ถอยเท่าที่ dock B เข้าไปล่าสุด / 0.25 m
ตัวเลือกใส่ได้ทุกขั้น: retry: N (พังแล้วลองใหม่เองกี่ครั้ง) · optional: true (พังแล้วข้าม) · prep: <โปรแกรม/@ท่า>
  (เล่นก่อนทุกครั้งที่ทำขั้นนั้น เช่นยกแขนขึ้นก่อนหยิบ)
หยุดกลางเส้นทางแล้วทำต่อ = วิ่งเฉพาะระยะที่เหลือ (ไม่เริ่มขาใหม่)
บันทึกทุกรอบ: <ARM_LOG_DIR>/show-YYYY-MM-DD.jsonl · สถานะ: /show/state (JSON)
แบบจำลอง (-p mock:=true): หยิบกล่องจำลอง ใช้คู่กับ show_mock (ฐาน + lidar + รีโมทจำลอง)
"""
import argparse
import json
import os
import queue
import sys
import threading
import time
from datetime import datetime

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String, UInt8

from ar_program import arm_log, programs
from humanoi_base.base_motion import MoveError
from humanoi_mission import show_routes
from humanoi_mission.mission import ARM_TH, HAND, STEPS, Abort, Mission, StepError, describe, missions_file

SHOW_STEPS = STEPS + ("say", "route", "dock", "undock")
OPTS = ("retry", "optional", "prep", "note")
DRIVE = ("move", "route", "dock", "undock")
UNDOCK_DEFAULT = 0.25  # m — undock ที่ยังไม่มีระยะจาก route-record และยังไม่เคย dock ในรอบนี้
UNDOCK_MAX = 1.0       # ถอยเกินนี้ = น่าจะผิด (เห็นของอื่นแทนโต๊ะ) ไม่ถอย
RESULT_TH = {"done": "จบครบ", "ended": "จบกลางคัน (คนคุมสั่ง)", "quit": "ออกจากโชว์"}


class Quit(Exception):
    pass


def parse_step(item):
    """ขั้นในไฟล์ -> (ชนิด, ค่า, ตัวเลือก)"""
    if not isinstance(item, dict):
        raise StepError(f"ขั้น {item!r} ต้องเป็น 'ชนิด: ค่า'")
    kinds = [k for k in item if k in SHOW_STEPS]
    extra = [k for k in item if k not in SHOW_STEPS and k not in OPTS]
    if len(kinds) != 1 or extra:
        raise StepError(f"ขั้น {item!r}: ใส่ชนิดขั้นเดียว ({', '.join(SHOW_STEPS)}) + ตัวเลือก ({', '.join(OPTS)})")
    return kinds[0], item[kinds[0]], {k: item[k] for k in OPTS if k in item}


def step_text(kind, val):
    if kind == "say":
        return f"ข้อความ: {val}"
    if kind == "route":
        return f"ฐานวิ่งเส้นทาง {val}"
    if kind == "dock":
        return f"เข้าจอด {val}" if isinstance(val, str) else f"เข้าจอด ห่าง {val.get('front')} m"
    if kind == "undock":
        return f"ถอยออกจาก {val}"
    return describe(kind, val)


def show_log_file(ts):
    return os.path.join(arm_log.log_dir(), f"show-{datetime.fromtimestamp(ts, arm_log.LOCAL_TZ):%Y-%m-%d}.jsonl")


class MockPick:
    """แทน TrayPick ตอนทดสอบ (ไม่มีกล้อง / ท่าจับ) — แขนไม่ขยับ · /show_mock/cmd "pick fail N" = N ครั้งถัดไปหากล่องไม่เจอ"""

    def __init__(self, show, arm):
        self.show, self.arm, self.stop_event, self.holding = show, arm, threading.Event(), False

    def plan(self):
        from humanoi_vision.tray_pick import PickError
        if self.show.mock_fail > 0:
            self.show.mock_fail -= 1
            raise PickError("ไม่เห็นกล่องนิ่ง (จำลอง) — กล้องเห็นป้ายทั้งแผ่นไหม")
        return {"info": "กล่องจำลอง"}

    def describe(self, pl):
        return f"  {pl['info']} (จำลอง — แขนไม่ขยับ)"

    def _act(self, seconds, label):
        from humanoi_vision.tray_pick import PickError
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            if self.stop_event.is_set():
                raise PickError(f"{label}: หยุดแล้ว (สั่งหยุด)")
            time.sleep(0.05)

    def pick(self, pl):
        self._act(1.5, "หยิบ")
        self.holding = True

    def place_back(self):
        self._act(1.5, "วางคืน")
        self.holding = False

    def destroy_node(self):
        pass


class Show(Mission):
    def __init__(self):
        super().__init__("show")
        for name, default in (("shows_file", ""), ("routes_file", ""), ("mock", False)):
            self.declare_parameter(name, default)
        self.file = self.get_parameter("shows_file").value or os.path.join(
            os.path.dirname(missions_file()), "shows.yaml")
        self.routes_path = self.get_parameter("routes_file").value or show_routes.routes_file()
        self.mock = bool(self.get_parameter("mock").value)
        self.cmds = queue.Queue()
        self.phase, self.quitting, self.info = "idle", False, {}
        self.round_no, self.mock_fail = 0, 0
        self.docked = {}  # ชื่อจุดจอด -> ระยะที่ dock เดินหน้าเข้าไปล่าสุด (m) — undock ถอยเท่านี้ถ้าไม่มี approach
        self.seen, self.tray = {}, ("", 0.0)
        self.show_state_pub = self.create_publisher(String, "/show/state", 10)
        self.say_pub = self.create_publisher(String, "/show/say", 10)
        self.create_subscription(String, "/show/cmd", lambda m: self.command(m.data), 10)
        self.create_subscription(UInt8, "/base_guard", lambda m: self.seen.__setitem__("guard", time.monotonic()), 10)
        self.create_subscription(String, "/tray/status", lambda m: setattr(self, "tray", (m.data, time.monotonic())), 10)
        if self.mock:
            self.create_subscription(String, "/show_mock/cmd", self._on_mock, 10)
        self.create_timer(1.0, self._republish)

    # ---------- คนคุม ----------
    def command(self, text):
        """go / stop / quit จากรีโมท แป้นพิมพ์ หรือ /show/cmd — หยุดระหว่างทำขั้นมีผลทันที"""
        c = {"": "go", "go": "go", "g": "go", "s": "stop", "stop": "stop", "q": "quit", "quit": "quit"}.get(
            text.strip().lower())
        if c is None:
            return
        if self.phase in ("running", "confirm"):
            if c == "stop":
                self.stop_all("คนคุมสั่งหยุด (SWD / s)")
                return
            if c == "quit":
                self.quitting = True
                self.stop_all("ออกจากโชว์ (q)")
                return
        self.cmds.put(c)

    def _on_state(self, msg):
        """สวิตช์รีโมทจาก Mega ("SW:BxDy") — SWB โยกลง = go · SWD โยกลง = stop"""
        line = msg.data.strip()
        if not line.startswith("SW:B") or len(line) < 7 or line[5] != "D":
            return
        b, d = line[4], line[6]
        lost = b == "-" and d == "-"
        b = None if (b == "-" and not lost) else b
        d = None if (d == "-" and not lost) else d
        if d is not None and self.swd.update(d):
            self.command("stop")
        if b is not None and self.swb.update(b) and self.swd.pos != "D":
            self.command("go")

    def _on_mock(self, msg):
        words = msg.data.split()
        if len(words) >= 2 and words[0] == "pick":
            self.mock_fail = int(words[2]) if words[1] == "fail" and len(words) > 2 else (
                1 if words[1] == "fail" else 0)

    def drain(self):
        while not self.cmds.empty():
            self.cmds.get_nowait()

    def wait_cmd(self, allowed):
        self.drain()
        while True:
            if self.quitting:
                return "quit"
            try:
                c = self.cmds.get(timeout=0.2)
            except queue.Empty:
                continue
            if c in allowed:
                return c

    def report(self, phase=None, **kw):
        if phase:
            self.phase = phase
        self.info.update(kw)
        self.info["phase"] = self.phase
        self._republish()

    def _republish(self):
        if self.info:
            self.show_state_pub.publish(String(data=json.dumps(self.info, ensure_ascii=False)))

    def clear_stop(self):
        self.abort.clear()
        self.abort_reason = ""
        self.motion.stop_event.clear()
        for p in self.pickers.values():
            p.stop_event.clear()

    # ---------- ขั้น ----------
    def picker(self, arm):
        if not self.mock:
            return super().picker(arm)
        if arm not in self.pickers:
            self.pickers[arm] = MockPick(self, arm)
        return self.pickers[arm]

    def grasp_ok(self, arm, config_dir):
        return True if self.mock else super().grasp_ok(arm, config_dir)

    def do_confirm(self, text):
        self.report("confirm", message=str(text))
        print(f"  ⏸ {text} — โยก SWB (หรือ Enter) เมื่อพร้อม · SWD / s = หยุด", flush=True)
        self.drain()
        try:
            while True:
                self.check_abort()
                try:
                    if self.cmds.get(timeout=0.1) == "go":
                        return
                except queue.Empty:
                    pass
        finally:
            if self.phase == "confirm":
                self.phase = "running"

    def do_say(self, text):
        self.say_pub.publish(String(data=str(text)))
        self.report(message=str(text))
        print(f"  💬 {text}", flush=True)

    def run_legs(self, cfg, st):
        """ขยับทีละขาที่ยังเหลือใน st["legs"] — โดนหยุดกลางขา = เก็บระยะที่เหลือไว้ ทำต่อครั้งหน้า"""
        self.check_stow(cfg)
        while st["legs"]:
            kind, amount = st["legs"][0]
            print(f"    → {show_routes.leg_text(kind, amount)}", flush=True)
            try:
                res = self.motion.move(kind, amount, log=lambda s: print("  " + s, flush=True))
            except MoveError as exc:
                rest = show_routes.remaining(kind, amount, exc.done)
                if rest is None:
                    st["legs"].pop(0)
                else:
                    st["legs"][0] = (kind, rest)
                self.check_abort()
                raise StepError(str(exc))
            print("      ✓ " + self.motion.describe(res), flush=True)
            st["legs"].pop(0)

    @staticmethod
    def dock_target(val, routes):
        if isinstance(val, str):
            return float(routes["docks"][val]), None
        return float(val["front"]), val.get("max_travel")

    def do_dock(self, cfg, val, routes):
        self.check_stow(cfg)
        target, travel = self.dock_target(val, routes)
        try:
            res = self.motion.approach(target, travel, log=lambda s: print("  " + s, flush=True))
        except MoveError as exc:
            self.check_abort()
            raise StepError(str(exc))
        print("  ✓ " + self.motion.describe(res), flush=True)
        if isinstance(val, str):
            self.docked[val] = float(res.get("travel") or 0.0)

    def undock_legs(self, val, routes):
        """ถอยเท่าไหร่: approach (ห่างโต๊ะเท่าตอนเส้นทางเริ่ม/จบ วัดด้วย lidar) > ระยะที่ dock เข้าไป > ค่าเริ่ม"""
        if not isinstance(val, str):
            dist, how = float(val), "ตามที่ตั้ง"
        elif val in routes["approach"]:
            target = float(routes["approach"][val])
            try:
                g = self.motion.front_gap(0.5)
            except MoveError as exc:
                raise StepError(str(exc))
            if g is None:
                raise StepError(f"ไม่เห็นโต๊ะ {val} ข้างหน้า — วัดระยะถอยไม่ได้ (หันผิด / ไม่ได้จอดที่ {val}?)")
            dist, how = target - g, f"ให้ห่างโต๊ะ {target:.2f} m (ตอนนี้ {g:.2f})"
        elif val in self.docked:
            dist, how = self.docked[val], "เท่าที่ dock เข้าไป"
        else:
            dist, how = UNDOCK_DEFAULT, "ค่าเริ่ม — ยังไม่มีระยะจาก route-record"
        if dist > UNDOCK_MAX:
            raise StepError(f"ต้องถอย {dist:.2f} m ({how}) — มากผิดปกติ ไม่ถอย: เช็กว่าหุ่นจอดที่ {val} จริง")
        print(f"    ถอย {max(dist, 0.0):.2f} m ({how})", flush=True)
        return [("back", round(dist, 3))] if dist >= show_routes.DONE_M else []

    def exec_step(self, cfg, routes, kind, val, st):
        if kind == "confirm":
            self.do_confirm(val)
        elif kind == "program":
            self.do_program(val)
        elif kind == "hand":
            self.do_hand(val)
        elif kind in ("pick", "place_back"):
            self.do_pick(val, back=kind == "place_back")
        elif kind in ("move", "route"):
            self.run_legs(cfg, st)
        elif kind == "undock":
            if "legs" not in st:  # คิดครั้งเดียว — หยุดกลางทางแล้วทำต่อ = ถอยเฉพาะที่เหลือ
                self.check_stow(cfg)
                st["legs"] = self.undock_legs(val, routes)
            self.run_legs(cfg, st)
        elif kind == "dock":
            self.do_dock(cfg, val, routes)
        elif kind == "say":
            self.do_say(val)
        else:
            self.sleep(float(val))

    # ---------- เช็กก่อนเริ่มรอบ ----------
    def preflight(self, name, show, cfg, routes):
        """[(ok, ข้อความ)] — ok False = เริ่มไม่ได้ · None = เตือน (เริ่มได้) · True = ผ่าน"""
        try:
            steps = [parse_step(s) for s in show.get("round") or []]
        except StepError as exc:
            return [(False, str(exc))]
        if not steps:
            return [(False, f"โชว์ '{name}' ไม่มีขั้นใน round")]
        def like(keep):
            """ขั้นแบบ mission ให้ validate — ขั้นอื่นแทนด้วย wait: 0 ให้เลขขั้นในข้อความตรงกับไฟล์โชว์"""
            return [{k: v} if k in STEPS and keep(o) else {"wait": 0} for k, v, o in steps]

        drives = any(k in DRIVE for k, _, _ in steps)
        problems = self.validate(cfg, like(lambda o: not o.get("optional")))
        for i, (k, _, o) in enumerate(steps, 1):  # prep = โปรแกรมที่เล่นก่อนขั้นนั้น
            if "prep" in o and not o.get("optional"):
                problems += [p.replace("ขั้น 1 (program)", f"ขั้น {i} (prep)") for p in
                             self.validate(cfg, [{"program": o["prep"]}])]
        if drives and not any(k == "move" for k, _, _ in steps):
            problems += self.validate(cfg, [{"move": {"forward": 0.0}}])  # ท่าพับ (stow) ที่ฐานวิ่งได้
        # ขั้น optional ที่ยังไม่พร้อม = เตือน (ตอนเล่นพังแล้วข้ามเอง)
        warn = [f"{w} (optional — ตอนเล่นจะข้าม)" for w in self.validate(cfg, like(lambda o: o.get("optional")))]
        warn += [f"ไม่เช็กท่าพับแขน{ARM_TH.get(a, a)} (stow: skip) — พับ / รองแขนไว้เองก่อนฐานวิ่ง"
                 for a, p in (cfg.get("stow") or {}).items() if drives and p == "skip"]
        running = [{k: v} for k, v, o in steps if k in STEPS and not o.get("optional")]
        running += [{"program": o["prep"]} for _, _, o in steps if "prep" in o]
        if drives:
            running.append({"move": {"forward": 0.0}})  # odometry + lidar
        for i, (k, v, o) in enumerate(steps, 1):
            where = f"ขั้น {i} ({k})"
            if k == "route":
                if v not in routes["routes"]:
                    rev = show_routes.reverse_name(v)
                    problems.append(f"{where}: ไม่มีเส้นทาง '{v}' — สอน: arm show route-record {v}"
                                    + (f" (หรือสอน {rev} แล้วได้ขานี้ให้เอง)" if "_to_" in v else ""))
                else:
                    try:
                        show_routes.legs_of(routes["routes"][v])
                    except ValueError as exc:
                        problems.append(f"{where}: เส้นทาง '{v}' ผิด — {exc}")
            elif k == "dock":
                if isinstance(v, str):
                    if v not in routes["docks"]:
                        problems.append(f"{where}: ไม่มีระยะจอด '{v}' — จอดหุ่นถูกที่แล้ว: arm show dock-save {v}")
                elif not isinstance(v, dict) or not isinstance(v.get("front"), (int, float)):
                    problems.append(f"{where}: ใช้ dock: <ชื่อ> หรือ dock: {{front: m}}")
            elif k == "undock":
                if isinstance(v, bool) or not isinstance(v, (str, int, float)) or (
                        not isinstance(v, str) and not 0 < v <= UNDOCK_MAX):
                    problems.append(f"{where}: ใช้ undock: <ชื่อจุดจอด> หรือ undock: <m ไม่เกิน {UNDOCK_MAX}>")
                elif isinstance(v, str) and v not in routes["approach"] and v not in self.docked:
                    warn.append(f"{where}: ยังไม่มีระยะถอยของ {v} จาก route-record — ถอยเท่าที่ dock {v} เข้าไป "
                                f"(ยังไม่เคย dock = {UNDOCK_DEFAULT} m)")
            elif k == "say" and not str(v).strip():
                problems.append(f"{where}: ใส่ข้อความ")
            if "retry" in o and not (isinstance(o["retry"], int) and o["retry"] >= 0):
                problems.append(f"{where}: retry ต้องเป็นจำนวนเต็ม ≥ 0")
        if problems:
            return [(False, p) for p in problems] + [(None, w) for w in warn]
        out = [(None, w) for w in warn] + [(False, p) for p in self.check_running(running)]
        now = time.monotonic()
        with self.lock:
            js = dict(self.joints)
        for arm in HAND:
            ok = all(n in js for n in programs.joint_names(arm))
            out.append((ok, f"แขน{ARM_TH[arm]}: ตำแหน่งข้อต่อ" + ("" if ok else " — ไม่มี /joint_states (arm start)")))
        if drives:
            o = self.motion.pose()
            ok = bool(o and now - o[0] < 1.0)
            out.append((ok, "ฐาน: odometry" + ("" if ok else " เงียบ — arm base เปิดอยู่ไหม encoder ต่ออยู่ไหม")))
            sc = self.motion.scan
            ok = bool(sc and now - sc[0] < 1.0)
            out.append((ok, "lidar" + ("" if ok else " เงียบ — arm lidar")))
            g = self.seen.get("guard")
            ok = bool(g and now - g < 1.5)
            out.append((True if ok else None, "กันชนชั้น Mega" + ("" if ok else
                        " — ไม่เห็น /base_guard (arm base แบบ noguard?) ฐานจะมีแค่กันชนชั้นซอฟต์แวร์")))
            mode = self.motion.mode
            ok = mode in (None, "ROS")
            out.append((ok, f"Mega โหมด {mode or '-'}" + ("" if ok else
                        " — ROS ขับไม่ได้: ยก SWA ขึ้นแล้วลง (ARM) ปล่อยคันโยก")))
        if any(k == "pick" for k, _, _ in steps) and not self.mock:
            status, t = self.tray
            ok = status == "ok" and now - t < 2.0
            out.append((True if ok else None, "กล้อง: เห็นกล่อง" if ok else
                        f"กล้อง: ยังไม่เห็นกล่อง ({status or 'ไม่มีข้อมูล'}) — วางกล่องที่ A ให้กล้องเห็นป้าย"))
        return out

    @staticmethod
    def print_checks(checks):
        for ok, text in checks:
            print(f"  {'✓' if ok else ('⚠' if ok is None else '✗')} {text}", flush=True)
        return not any(ok is False for ok, _ in checks)

    # ---------- เล่น ----------
    def list(self):
        cfg = self.load()
        routes = show_routes.load(self.routes_path)
        print(f"โชว์ใน {self.file}:")
        for name, show in (cfg.get("shows") or {}).items():
            print(f"  {name:<12} {len(show.get('round') or [])} ขั้น  {show.get('title', '')}")
        print(f"\nเส้นทาง ({self.routes_path}):")
        for name, legs in routes["routes"].items():
            try:
                print(f"  {name:<12} {show_routes.describe(show_routes.legs_of(legs))}")
            except ValueError as exc:
                print(f"  {name:<12} ✗ {exc}")
        print("ระยะจอด: " + (", ".join(f"{k} {v:.3f} m" for k, v in routes["docks"].items()) or "(ยังไม่มี)"))
        print("ถอยออกจนห่าง (undock): " + (", ".join(f"{k} {v:.3f} m" for k, v in routes["approach"].items())
                                          or "(ยังไม่มี — route-record จดให้)"))
        print("\nเปิดโชว์: arm show <ชื่อ> · เช็กอย่างเดียว: arm show check <ชื่อ>")

    def check(self, name):
        cfg = self.load()
        show = (cfg.get("shows") or {}).get(name)
        if not show:
            print(f"✗ ไม่มีโชว์ '{name}' ใน {self.file}")
            return 1
        time.sleep(2.0)  # discovery + ข้อความแรกของแต่ละหัวข้อ
        print(f"เช็กโชว์ '{name}':")
        return 0 if self.print_checks(self.preflight(name, show, cfg, show_routes.load(self.routes_path))) else 1

    def run(self, name):
        cfg = self.load()
        show = (cfg.get("shows") or {}).get(name)
        if not show:
            print(f"✗ ไม่มีโชว์ '{name}' ใน {self.file}")
            return 1
        time.sleep(2.0)
        print(f"โชว์ '{name}' {show.get('title', '')}{' (จำลอง)' if self.mock else ''}\n"
              "  SWB / Enter = เริ่มรอบ · ยืนยัน · ทำต่อ   SWD / s = หยุด (โยกอีกครั้ง = จบรอบ)   q = ออก", flush=True)
        self.report("idle", show=name, round=0, step=0, of=len(show.get("round") or []), text="", message="รอเริ่ม")
        try:
            while True:
                print(f"\nพร้อม — โยก SWB (หรือ Enter) = เริ่มรอบที่ {self.round_no + 1} · q = ออก", flush=True)
                if self.wait_cmd(("go", "quit")) == "quit":
                    return 0
                cfg = self.load()  # แก้ไฟล์ระหว่างพักได้ — รอบใหม่ใช้ค่าล่าสุด
                show = (cfg.get("shows") or {}).get(name) or show
                routes = show_routes.load(self.routes_path)
                print("เช็กระบบ:", flush=True)
                if not self.print_checks(self.preflight(name, show, cfg, routes)):
                    print("✗ ยังเริ่มรอบไม่ได้ — แก้ตามรายการแล้วโยก SWB ใหม่", flush=True)
                    self.report("idle", message="เช็กระบบไม่ผ่าน")
                    continue
                for k, v, _ in map(parse_step, show["round"]):
                    if k == "pick":
                        self.picker(v)  # สร้างก่อน ให้เก็บตำแหน่งกล่องจากกล้องไว้
                self.run_round(name, show, cfg, routes)
        except Quit:
            return 0

    def run_round(self, name, show, cfg, routes):
        steps = [parse_step(s) for s in show["round"]]
        n = len(steps)
        self.round_no += 1
        t0 = time.time()
        rec = {"type": "round", "show": name, "round": self.round_no, "ts": round(t0, 3),
               "time": arm_log.local_time(t0), "mock": self.mock, "steps": []}
        print(f"\n=== รอบที่ {self.round_no} ===", flush=True)
        result = "done"
        for i, (kind, val, opts) in enumerate(steps, 1):
            text = step_text(kind, val)
            st = {}
            if kind == "move":
                st["legs"] = [show_routes.leg_of(val)]
            elif kind == "route":
                st["legs"] = show_routes.legs_of(routes["routes"][val])
            srec = {"i": i, "kind": kind, "text": text, "tries": 0, "stops": 0}
            t_step, auto = time.monotonic(), 0
            while True:
                srec["tries"] += 1
                self.clear_stop()
                self.report("running", round=self.round_no, step=i, of=n, kind=kind, text=text, message="")
                print(f"[{i}/{n}] {text}" + (f" (ครั้งที่ {srec['tries']})" if srec["tries"] > 1 else ""), flush=True)
                stopped = False
                try:
                    if "prep" in opts and kind not in DRIVE:
                        self.do_program(opts["prep"])
                    self.exec_step(cfg, routes, kind, val, st)
                    srec["result"] = "ok"
                    break
                except Abort as exc:
                    err, stopped = str(exc), True
                    srec["stops"] += 1
                except StepError as exc:
                    err = str(exc)
                    self.stop_all(f"ขั้น {i} ไม่สำเร็จ")
                    if auto < int(opts.get("retry", 0)) and not self.quitting:
                        auto += 1
                        print(f"  ✗ {err} — ลองใหม่อัตโนมัติ ({auto}/{opts['retry']})", flush=True)
                        time.sleep(1.0)
                        continue
                    if opts.get("optional") and not self.quitting:
                        print(f"  ✗ {err} — ข้ามขั้นนี้ (optional)", flush=True)
                        srec.update(result="skipped", error=err)
                        break
                if self.quitting:
                    srec.update(result="stopped", error=err)
                    result = "quit"
                    break
                self.report("paused" if stopped else "error", message=err)
                print(f"  ⏸ {'หยุดแล้ว' if stopped else 'ไม่สำเร็จ'}: {err}\n"
                      f"    SWB / Enter = {'ทำขั้นนี้ต่อ' if stopped else 'ลองขั้นนี้ใหม่'} · SWD / s = จบรอบนี้ · q = ออก",
                      flush=True)
                c = self.wait_cmd(("go", "stop", "quit"))
                if c == "go":
                    continue
                srec.update(result="stopped" if stopped else "error", error=err)
                result = "ended" if c == "stop" else "quit"
                break
            srec["dur"] = round(time.monotonic() - t_step, 1)
            rec["steps"].append(srec)
            if result != "done":
                break
        rec.update(result=result, dur=round(time.time() - t0, 1))
        arm_log.append(show_log_file(t0), rec)
        print(f"=== รอบที่ {self.round_no}: {RESULT_TH[result]} ({rec['dur']:.0f} วิ) ===", flush=True)
        if result == "ended":  # รอบใหม่เริ่มขั้นแรกเสมอ — ต้องคืนของให้อยู่ที่เริ่ม
            print("  ! จบกลางรอบ — ก่อนเริ่มรอบใหม่: ขับรีโมทให้หุ่นกลับมาจอดที่จุดเริ่ม (หน้าโต๊ะ A) "
                  "แขนว่าง / พับ และกล่องวางที่ A", flush=True)
        self.report("idle", round=self.round_no, step=0, text="", message=f"รอบที่ {self.round_no} {RESULT_TH[result]}",
                    last=result)
        if result == "quit":
            raise Quit()
        return result


def main():
    ap = argparse.ArgumentParser(description="ตัวคุมโชว์ — เล่นทีละรอบตาม config/shows.yaml")
    ap.add_argument("action", nargs="?", default="list", help="list | check | run")
    ap.add_argument("name", nargs="?")
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    if args.action in ("check", "run") and not args.name:
        ap.error("ใส่ชื่อโชว์")
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Show()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    node.ex = ex
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()

    def read_keys():
        for line in sys.stdin:
            node.command(line)
    threading.Thread(target=read_keys, daemon=True).start()
    rc = 0
    try:
        if args.action == "run":
            rc = node.run(args.name)
        elif args.action == "check":
            rc = node.check(args.name)
        else:
            node.list()
    except StepError as exc:
        print(f"✗ {exc}")
        rc = 1
    except KeyboardInterrupt:
        node.stop_all("Ctrl+C")
        print("\n✗ ออกจากโชว์ (Ctrl+C) — ฐานและแขนหยุดอยู่กับที่")
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
