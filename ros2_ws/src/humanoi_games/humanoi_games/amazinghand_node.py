#!/usr/bin/env python3
"""amazinghand_node — คุมมือ AmazingHand (8× Feetech SCS0009 ผ่าน Waveshare Bus Servo Adapter) จาก ROS

ตัวนี้ตัวเดียวเป็นเจ้าของพอร์ตมือ (เปิดพอร์ตได้ทีละโปรแกรม — ปิด tools\\*.py / robot_hand.py ก่อน)

  ros2 run humanoi_games amazinghand_node                                  # มือแขนซ้าย
  ros2 run humanoi_games amazinghand_node --ros-args -p arm:=right -r __node:=amazinghand_right   # มือแขนขวา
  ros2 run humanoi_games amazinghand_node --ros-args -p dry_run:=true      # ไม่ต่อมือ แค่ log ว่าจะทำท่าอะไร
  ros2 topic pub --once /hand/gesture std_msgs/msg/String "{data: paper}"

บอร์ดมือของแต่ละแขนผูกด้วยหมายเลขบอร์ดใน config/hand_boards.yaml (มือสองข้างเป็น servo ID 1-8 เหมือนกัน
และลำดับ ttyACM สลับได้) — ไม่มีไฟล์ = มือข้างเดียวแบบเดิม (หาบอร์ดเอง ชุดมือจาก arm hand-side)

หัวข้อ (แขนซ้าย /hand/... · แขนขวา /right_hand/...)
  /hand/gesture (sub, String)  ชื่อท่าใน hand_poses (rock paper scissors like point victory nonono ...)
                               หรือท่าที่สอนเอง (arm hand-teach → programs.yaml hand_poses — ชื่อซ้ำท่าในตัว = ใช้แทน)
                               หรือ relax = ปลดแรงบิด นิ้วอ่อน — คำสั่งใหม่ตัดท่าเคลื่อนไหวที่ค้างอยู่ทันที
  /hand/status  (pub, String)  ready | gesture:<ท่า> | relaxed | error:<เหตุผล>

baud หาเอง (ลอง baud ที่ตั้งไว้ก่อน แล้วไล่ค่าอื่น — มือนี้เจอทั้ง 1 Mbps และ 250000 ของร้าน)
แบสุด / กำสุด / เอียง ของแต่ละนิ้ว อ่านจากไฟล์ที่ hand_calibrate บันทึก (arm hand-calibrate) — ไม่มีไฟล์ = ค่าเดโม
ก่อนเปิดแรงบิดเช็กทุกครั้ง (เหมือน tools\\scan_servos.py): servo ครบทุก ID และไฟ 4.8-7.4 V
ไม่ผ่าน = ไม่ขยับ รายงาน error แล้วลองใหม่ตอนได้คำสั่งถัดไป · ปิดโหนด = ปลดแรงบิดทุกตัว
"""
import json
import math
import os
import queue
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from humanoi_games import hand_poses

try:  # ท่ามือที่สอนเองอยู่ใน programs.yaml ของ ar_program — ไม่มี ar_program = ใช้ท่าในตัวอย่างเดียว
    from ar_program import programs
except ImportError:
    programs = None

ADAPTER_USB = (0x1A86, 0x55D3)  # Waveshare Bus Servo Adapter (A) = CH343 — Nano ของแขนเป็น CH340 (55d3 ≠ 7523)
FACTORY_LIMIT = 144.0  # ลิมิตโรงงาน SCS0009 (raw 20-1003) — ใช้เฉพาะตอนอ่านจาก servo ไม่ได้/ค่าแปลก
LIMIT_MARGIN = 1.0     # เผื่อการปัดเศษตอนแปลงหน่วย
SPEED = 7.0            # ความเร็วทำท่า (เท่า MaxSpeed ในเดโม) — hand_teach ใช้ค่านี้ตอน load ให้เห็นเหมือนตอนเล่นจริง
STUCK_DEG = 8.0        # หยุดขยับแล้วแต่ห่างเป้าหมายเกินนี้ = นิ้วไปต่อไม่ได้ (ติดนิ้วอื่น / สุดทางกลไก)
SETTLE_TIMEOUT = 3.0   # รอท่านิ่งนานสุดกี่วินาที
FINGERS_TH = ("ชี้", "กลาง", "นาง", "โป้ง")  # servo ID ตามลำดับ (1 2) (3 4) (5 6) (7 8)


def config_dir():
    """ที่เก็บค่าของมือ — อยู่ใน src (ไม่ใช่ install) เหมือน programs.yaml: build ใหม่ไม่ทับ

    ลำดับ: Pi (Docker, /humanoi_ws/src) > PC (WSL) > ~/.config
    """
    for src in ("/humanoi_ws/src", "/mnt/c/Users/User/Desktop/Humanoi/ros2_ws/src"):
        if os.path.isdir(os.path.join(src, "humanoi_games")):
            return os.path.join(src, "humanoi_games", "config")
    return os.path.expanduser("~/.config/humanoi_games")


def hand_side():
    """ชุดมือที่ใส่อยู่ตอนนี้ (right = ชุดมือขวาตัวแรก, left = ชุดมือซ้าย) — ตั้งด้วย arm hand-side

    ทั้งสองชุดเป็น servo ID 1-8 เหมือนกัน แยกจากการสื่อสารไม่ได้ ต้องบอกเอง · ตัวแปร HAND_SIDE > ไฟล์ > right
    """
    side = os.environ.get("HAND_SIDE")
    if not side:
        try:
            with open(os.path.join(config_dir(), "hand_side"), encoding="utf-8") as fh:
                side = fh.read().strip()
        except FileNotFoundError:
            side = "right"
    if side not in ("left", "right"):
        raise ValueError(f"ชุดมือ '{side}' ไม่รู้จัก — ตั้งใหม่: arm hand-side left|right")
    return side


def calib_file(side="right"):
    """ไฟล์คาลิเบรตของชุดมือนั้น (ชุดมือขวา = middle_pos.json เดิม) · ตัวแปร HAND_CALIB_FILE แทนได้"""
    env = os.environ.get("HAND_CALIB_FILE")
    if env:
        return env
    return os.path.join(config_dir(), "middle_pos.json" if side == "right" else f"middle_pos_{side}.json")


def load_calib(path):
    """ค่าคาลิเบรตมือ {นิ้ว: {open, close, tilt}} จากไฟล์ — None ถ้ายังไม่มีไฟล์ (นิ้วที่ขาด = ค่าเดโม)

    ไฟล์รุ่นแรก (middle_pos 8 ค่า + close/open แบบเทียบท่าตรง) แปลงให้เอง: งอ0 = (a-b)/2, เอียง = (a+b)/2
    """
    try:
        with open(path, encoding="utf-8") as fh:
            data = json.load(fh)
    except FileNotFoundError:
        return None
    calib = hand_poses.demo_calib()
    if "fingers" in data:
        for f, values in data["fingers"].items():
            if f in calib:
                calib[f] = {k: float(values[k]) for k in ("open", "close", "tilt")}
    else:  # รุ่นแรก
        mid = [float(v) for v in data["middle_pos"]]
        for i, f in enumerate(hand_poses.FINGERS):
            bend0, tilt0 = (mid[2 * i] - mid[2 * i + 1]) / 2, (mid[2 * i] + mid[2 * i + 1]) / 2
            calib[f] = {"open": bend0 + float((data.get("open") or {}).get(f, hand_poses.DEMO_OPEN)),
                        "close": bend0 + float((data.get("close") or {}).get(f, hand_poses.DEMO_CLOSE)),
                        "tilt": tilt0}
    for f, c in calib.items():
        if not c["close"] > c["open"]:
            raise ValueError(f"{path}: ค่าของ{f} ผิด {c} (กำสุดต้องมากกว่าแบสุด)")
    return calib


def describe(calib):
    """ค่าคาลิเบรตแบบอ่านง่าย: นิ้ว แบ/กำ/เอียง"""
    th = {"index": "ชี้", "middle": "กลาง", "ring": "นาง", "thumb": "โป้ง"}
    return " · ".join(f"{th[f]} แบ {c['open']:+.0f} กำ {c['close']:+.0f} เอียง {c['tilt']:+.0f}"
                      for f, c in calib.items())


def hand_boards():
    """บอร์ดมือของแต่ละแขน {แขน: {board: หมายเลขบอร์ด, kit: ชุดมือ}} จาก config/hand_boards.yaml — {} ถ้าไม่มีไฟล์

    มีมือสองข้าง = ต้องผูกบอร์ดกับแขนด้วยหมายเลขบอร์ด (ลำดับ ttyACM สลับได้ทุกครั้งที่เสียบ)
    """
    try:
        import yaml
        with open(os.path.join(config_dir(), "hand_boards.yaml"), encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
    except FileNotFoundError:
        return {}
    out = {}
    for arm, v in data.items():
        if arm not in ("left", "right") or not isinstance(v, dict) or v.get("kit") not in ("left", "right"):
            raise ValueError(f"hand_boards.yaml: '{arm}' ต้องเป็น left/right และมี board + kit (left/right)")
        out[arm] = {"board": str(v["board"]), "kit": v["kit"]}
    return out


def arm_hand(arm):
    """(พอร์ต, ชุดมือ) ของมือบนแขนนั้น ตาม hand_boards.yaml — RuntimeError ถ้าไม่ได้ตั้ง / บอร์ดไม่ได้เสียบ"""
    from serial.tools import list_ports
    boards = hand_boards()
    if arm not in boards:
        raise RuntimeError(f"hand_boards.yaml ไม่มีมือของแขน {arm}")
    serial = boards[arm]["board"]
    for p in list_ports.comports():
        if (p.vid, p.pid) == ADAPTER_USB and p.serial_number == serial:
            return p.device, boards[arm]["kit"]
    raise RuntimeError(f"ไม่เจอบอร์ดมือแขน{'ซ้าย' if arm == 'left' else 'ขวา'} ({serial}) — เสียบ USB ของมือนั้น · "
                       "ดูบอร์ดที่เสียบอยู่: arm hand-scan")


def resolve_hand(arm="left", port="auto", side="auto"):
    """(พอร์ตที่ส่งให้ Hand, ชุดมือ) ของมือบนแขนนั้น

    มี hand_boards.yaml + port auto -> บอร์ดตามหมายเลข (arm:<แขน>) และชุดมือตามไฟล์
    ไม่มีไฟล์ (มือข้างเดียว) -> แบบเดิม: port ตามที่ให้ / หาเอง, ชุดมือจาก arm hand-side
    """
    boards = hand_boards()
    if boards:
        if arm not in boards:
            raise RuntimeError(f"hand_boards.yaml ไม่มีมือของแขน {arm} (มี: {', '.join(boards)})")
        return (f"arm:{arm}" if port == "auto" else port), (boards[arm]["kit"] if side == "auto" else side)
    return port, (hand_side() if side == "auto" else side)


def topic_prefix(arm):
    """หัวข้อของมือบนแขนนั้น: แขนซ้าย /hand (ชื่อเดิม) · แขนขวา /right_hand"""
    return "/hand" if arm == "left" else f"/{arm}_hand"


def find_ports():
    """บอร์ดมือ (CH343) ทุกตัวที่เสียบอยู่ เรียงตามชื่อพอร์ต"""
    from serial.tools import list_ports
    return sorted(p.device for p in list_ports.comports() if (p.vid, p.pid) == ADAPTER_USB)


def find_port():
    ports = find_ports()
    return ports[0] if ports else None


def scalar(value):
    return value[0] if isinstance(value, list) else value  # rustypot read_* คืน list หนึ่งค่า


class Hand:
    """ห่อ rustypot: เปิดพอร์ต → เช็กไฟ → ถือท่าปัจจุบัน → เปิดแรงบิด (นิ้วไม่กระตุก)"""

    def __init__(self, port, baud, ids, min_volts, max_volts):
        self.port, self.baud, self.ids = port, baud, ids
        self.min_volts, self.max_volts = min_volts, max_volts
        self.c = None
        self.found_baud = None
        self.port_used = None
        self.limits = {sid: (-FACTORY_LIMIT, FACTORY_LIMIT) for sid in ids}  # องศา ต่ำสุด/สูงสุดของแต่ละ servo
        self.limit_warnings = []

    def connect(self):
        from rustypot import Scs0009PyController
        if self.c is None:
            # arm:<แขน> = บอร์ดของแขนนั้นตาม hand_boards.yaml (หาใหม่ทุกครั้ง — ลำดับพอร์ตเปลี่ยนได้)
            # auto: ลองบอร์ดมือทุกตัว ใช้ตัวแรกที่มีเซอร์โวตอบ (ใช้ได้เฉพาะตอนมีมือข้างเดียว)
            if self.port.startswith("arm:"):
                ports = [arm_hand(self.port[4:])[0]]
            else:
                ports = find_ports() if self.port == "auto" else [self.port]
            if not ports:
                raise RuntimeError("ไม่เจอบอร์ดมือ (Waveshare CH343 1a86:55d3) — เสียบ USB ของมือ")
            for port in ports:
                self.c = Scs0009PyController(serial_port=port, baudrate=self.baud, timeout=0.05)
                found = self._find_baud()
                if found is not None:
                    self.port_used = port
                    break
                self.c = None  # ปิดพอร์ตนี้ ลองตัวถัดไป
            if self.c is None:
                raise RuntimeError(f"ไม่มีเซอร์โวตอบเลยทุก baud ({', '.join(ports)}) — เปิดไฟมือหรือยัง, jumper อยู่ที่ B "
                                   "(USB), สายเซอร์โวเสียบช่อง D V G · เช็ก: arm hand-scan")
        else:
            found = self._find_baud()
            if found is None:
                raise RuntimeError(f"ไม่มีเซอร์โวตอบเลยทุก baud ({self.port_used}) — เปิดไฟมือหรือยัง · เช็ก: arm hand-scan")
        port = self.port_used
        self.found_baud = found
        bad = []
        for sid in self.ids:
            try:
                volts = scalar(self.c.read_present_voltage(sid)) / 10  # หน่วยรีจิสเตอร์ 0.1 V
            except Exception:
                bad.append(f"ID {sid} ไม่ตอบ")
                continue
            if not self.min_volts <= volts <= self.max_volts:
                bad.append(f"ID {sid} ไฟ {volts:.1f} V")
        if bad:
            raise RuntimeError(f"ไม่เปิดแรงบิด ({port}): " + ", ".join(bad) +
                               f" — ต้องครบทุกตัวและไฟ {self.min_volts}-{self.max_volts} V (เปิดไฟมือหรือยัง)")
        self._read_limits()
        for sid in self.ids:
            self.c.write_goal_position(sid, scalar(self.c.read_present_position(sid)))
            self.c.write_torque_enable(sid, 1)
        return port

    def _read_limits(self):
        """มุมต่ำสุด/สูงสุดที่ servo แต่ละตัวยอมหมุน — อ่านจาก EEPROM ของ servo เอง (อ่านอย่างเดียว)

        ค่าแปลก (อ่านไม่ได้ / ต่ำสุดไม่ติดลบ / สูงสุดไม่เป็นบวก / เกิน ±180°) = ใช้ลิมิตโรงงาน ±144° แล้วเตือน
        """
        self.limit_warnings = []
        for sid in self.ids:
            try:
                lo = math.degrees(scalar(self.c.read_min_angle_limit(sid)))
                hi = math.degrees(scalar(self.c.read_max_angle_limit(sid)))
            except Exception as exc:
                lo = hi = None
                self.limit_warnings.append(f"ID {sid} อ่านขีดจำกัดไม่ได้ ({exc})")
            if lo is not None and not -180 < lo < 0 < hi < 180:
                self.limit_warnings.append(f"ID {sid} ขีดจำกัดที่อ่านได้แปลก ({lo:+.1f}..{hi:+.1f}°)")
                lo = hi = None
            self.limits[sid] = (lo, hi) if lo is not None else (-FACTORY_LIMIT, FACTORY_LIMIT)

    def limits_text(self):
        return ", ".join(f"ID{sid} {lo:+.0f}..{hi:+.0f}" for sid, (lo, hi) in self.limits.items())

    def within(self, sid, deg):
        lo, hi = self.limits.get(sid, (-FACTORY_LIMIT, FACTORY_LIMIT))
        return lo + LIMIT_MARGIN <= deg <= hi - LIMIT_MARGIN

    def _find_baud(self):
        """baud ที่เซอร์โวตอบ — ลองค่าที่ตั้งไว้ก่อน แล้วไล่ค่าอื่น (อ่านอย่างเดียว ไม่เขียนอะไรลงเซอร์โว)

        มือนี้ร้านตั้งมา 250000 เคยย้ายเป็น 1 Mbps แล้ว แต่ 20 ต.ค. เจอกลับเป็น 250000 — หาเองจะได้ไม่ติดเรื่องนี้
        """
        order = [self.baud] + [b for b in sorted(self.c.baudrates(), reverse=True) if b != self.baud]
        for baud in order:
            self.c.set_baudrate(baud)
            for sid in self.ids:  # ตัวไหนตอบก็ได้ — ไม่ให้เซอร์โวตัวแรกเสียตัวเดียวแล้วหา baud ไม่เจอ
                try:
                    self.c.read_present_voltage(sid)
                    return baud
                except Exception:
                    continue
        return None

    def clamp(self, degrees):
        """เป้าหมายที่ servo ทำได้จริง (เกินขีดจำกัดของตัวนั้น = ขอบ)"""
        return [max(self.limits[sid][0] + LIMIT_MARGIN, min(self.limits[sid][1] - LIMIT_MARGIN, d))
                for sid, d in zip(self.ids, degrees)]

    def move(self, degrees, speed, enable=()):
        """สั่งทุก servo — เป้าหมายที่เกินขีดจำกัดของ servo ตัวนั้นถูกจำกัดไว้ที่ขอบ คืน ID ที่ถูกจำกัด (ไว้เตือน)

        enable = ID ที่เปิดแรงบิดซ้ำหลังตั้งเป้าหมาย — servo ตัดแรงบิดเองตอนโหลดเกิน (unloading condition) แล้วไม่เปิดคืนเอง
        """
        for sid in self.ids:
            self.c.write_goal_speed(sid, speed)
        clipped = [sid for sid, d in zip(self.ids, degrees) if not self.within(sid, d)]
        rads = [math.radians(d) for d in self.clamp(degrees)]
        try:
            self.c.sync_write_goal_position(self.ids, rads)
        except Exception:
            for sid, r in zip(self.ids, rads):
                self.c.write_goal_position(sid, r)
        for sid in enable:
            self.c.write_torque_enable(sid, 1)
        return clipped

    def unloaded(self):
        """ID ที่แรงบิดหลุดอยู่ (servo ตัดเองตอนโหลดเกิน / รีเซ็ตเพราะไฟตก)"""
        return [sid for sid in self.ids if not scalar(self._retry(self.c.read_torque_enable, sid))]

    def settle(self, ids=None, wait=None):
        """รอจน servo หยุดขยับ (ตำแหน่งเปลี่ยนไม่ถึง 1° ใน 0.1 s) -> (ตำแหน่งสุดท้าย, หยุดแล้วไหม)

        wait(0.1) แทน sleep ได้ — คืนค่าจริง = เลิกรอทันที (โหนดใช้รับคำสั่งใหม่ระหว่างรอ)
        """
        ids = ids or self.ids
        try:
            last = self.read_degrees(ids)
        except Exception:  # อ่านไม่ได้ = ไม่รู้ว่าถึงไหม (ไม่เช็ก) — ไม่ให้ท่าล้มเพราะเรื่องนี้
            return [], False
        for _ in range(int(SETTLE_TIMEOUT / 0.1)):
            if wait is None:
                time.sleep(0.1)
            elif wait(0.1):
                return last, False
            try:
                now = self.read_degrees(ids)
            except Exception:
                return last, False
            if all(abs(a - b) < 1.0 for a, b in zip(now, last)):
                return now, True
            last = now
        return last, False

    def stuck(self, degrees, ids, present):
        """servo ที่หยุดแล้วแต่ห่างเป้าหมายเกิน STUCK_DEG -> {ID: ตำแหน่งที่ไปถึง}"""
        goal = dict(zip(self.ids, self.clamp(degrees)))
        return {sid: p for sid, p in zip(ids, present) if abs(p - goal[sid]) > STUCK_DEG}

    def stuck_text(self, stuck, degrees):
        goal = dict(zip(self.ids, self.clamp(degrees)))
        return ", ".join(f"{FINGERS_TH[self.ids.index(sid) // 2]} ID{sid} ค้างที่ {p:+.0f} (สั่ง {goal[sid]:+.0f})"
                         for sid, p in stuck.items())

    def torque(self, ids, on):
        """เปิด/ปิดแรงบิดทีละ servo — เปิด = ถือที่ตำแหน่งปัจจุบันก่อน (นิ้วไม่กระตุก)"""
        for sid in ids:
            if on:
                self.c.write_goal_position(sid, scalar(self.c.read_present_position(sid)))
            self.c.write_torque_enable(sid, 1 if on else 0)

    def read_degrees(self, ids=None):
        """ตำแหน่งจริงของ servo (องศา) — อ่านพลาดลองซ้ำ (servo ที่เพิ่งรีเซ็ตเพราะไฟตกไม่ตอบชั่วครู่)"""
        return [math.degrees(scalar(self._retry(self.c.read_present_position, sid))) for sid in (ids or self.ids)]

    def _retry(self, read, sid, tries=4):
        for n in range(tries):
            try:
                return read(sid)
            except Exception as exc:
                if n == tries - 1:
                    raise RuntimeError(f"servo ID {sid} ไม่ตอบ {tries} ครั้งติด ({exc})") from exc
                time.sleep(0.05)

    def relax(self):
        if self.c is None:
            return
        for sid in self.ids:
            try:
                self.c.write_torque_enable(sid, 0)
            except Exception:
                pass


class AmazingHandNode(Node):
    def __init__(self):
        super().__init__("amazinghand")
        self.declare_parameter("arm", "left")    # แขนที่มือนี้ติด — เลือกบอร์ด (hand_boards.yaml) และหัวข้อ
        self.declare_parameter("port", "auto")   # auto = บอร์ดของแขนนี้ตาม hand_boards.yaml (ไม่มีไฟล์ = หาเอง)
        self.declare_parameter("baud", 1_000_000)
        self.declare_parameter("ids", [1, 2, 3, 4, 5, 6, 7, 8])
        self.declare_parameter("side", "auto")  # ชุดมือ right/left — auto = ตาม hand_boards.yaml / arm hand-side
        self.arm = self.get_parameter("arm").value
        port, self.side = resolve_hand(self.arm, self.get_parameter("port").value, self.get_parameter("side").value)
        # แบสุด / กำสุด / เอียง ต่อนิ้ว จาก hand_calibrate (ไม่มีไฟล์ = ท่าเหมือนเดโมของ Pollen)
        self.declare_parameter("calib_file", calib_file(self.side))
        self.declare_parameter("programs_file", programs.DEFAULT_FILE if programs else "")  # ท่ามือที่สอนเอง
        self.declare_parameter("speed", SPEED)
        self.declare_parameter("min_volts", 4.8)
        self.declare_parameter("max_volts", 7.4)  # ห้ามจ่ายไฟเซอร์โวเกิน 7.4 V
        self.declare_parameter("dry_run", False)

        gp = self.get_parameter
        path = gp("calib_file").value
        self.calib = load_calib(path)
        if self.calib is None:
            self.calib = hand_poses.demo_calib()
            self.get_logger().warn(f"ยังไม่ได้คาลิเบรตนิ้ว ({path} ไม่มี) — ใช้ค่าเดโม · คาลิเบรต: arm hand-calibrate")
        else:
            self.get_logger().info(f"คาลิเบรตมือจาก {path}: {describe(self.calib)}")
        self.speed = float(gp("speed").value)
        self.dry_run = bool(gp("dry_run").value)
        self.hand = Hand(port, int(gp("baud").value), list(gp("ids").value),
                         float(gp("min_volts").value), float(gp("max_volts").value))
        self.ready = False
        self.cmds = queue.Queue()
        prefix = topic_prefix(self.arm)
        self.status_pub = self.create_publisher(String, f"{prefix}/status", 10)
        self.create_subscription(String, f"{prefix}/gesture", self._on_gesture, 10)
        threading.Thread(target=self._worker, daemon=True).start()
        if self.dry_run:
            self.get_logger().warn("dry_run — ไม่ต่อมือจริง แค่ log ท่าที่ได้รับ")
            self._status("ready")
        else:
            self._connect()

    def _status(self, text):
        self.status_pub.publish(String(data=text))

    def _connect(self):
        try:
            port = self.hand.connect()
        except Exception as exc:
            self.ready = False
            self.get_logger().error(f"มือยังไม่พร้อม: {exc}")
            self._status(f"error:{exc}")
            return False
        self.ready = True
        if self.hand.found_baud != self.hand.baud:
            self.get_logger().warn(f"เซอร์โวตอบที่ {self.hand.found_baud} baud ไม่ใช่ {self.hand.baud} ที่ตั้งไว้ — ใช้ค่าที่เจอ")
        self.get_logger().info(f"มือแขน{'ซ้าย' if self.arm == 'left' else 'ขวา'}พร้อม ({port} @ {self.hand.found_baud}, "
                               f"ID {self.hand.ids}, ชุดมือ{'ซ้าย' if self.side == 'left' else 'ขวา'}) · "
                               f"ขีดจำกัดจาก servo: {self.hand.limits_text()}")
        for w in self.hand.limit_warnings:
            self.get_logger().warn(w + " — ใช้ลิมิตโรงงาน ±144°")
        self._status("ready")
        return True

    def _taught(self):
        """ท่ามือที่สอนเอง {ชื่อ: องศา 8 ตัว} — อ่านไฟล์ใหม่ทุกครั้ง สอนเพิ่มแล้วไม่ต้องรีสตาร์ท"""
        path = self.get_parameter("programs_file").value
        if programs is None or not path:
            return {}
        try:
            return programs.load(path)[programs.HAND_POSE_KEYS[self.side]]
        except Exception as exc:
            self.get_logger().warn(f"อ่านท่ามือที่สอนเองไม่ได้ ({exc}) — ใช้ท่าในตัว", throttle_duration_sec=30.0)
            return {}

    def _degrees(self, pose, taught):
        """องศาเป้าหมายของท่า — ท่าที่สอนเองก่อน ไม่มีค่อยคำนวณจากค่าคาลิเบรต"""
        if pose in taught and len(taught[pose]) == len(self.hand.ids):
            return list(taught[pose])
        return hand_poses.servo_degrees(pose, self.calib, self.side)

    def _on_gesture(self, msg):
        name = msg.data.strip()
        known = set(hand_poses.names()) | set(self._taught())
        if name != "relax" and name not in known:
            self.get_logger().warn(f"ไม่รู้จักท่า '{name}' — มี: relax, {', '.join(sorted(known))}")
            return
        self.cmds.put(name)

    def _worker(self):
        """ทำทีละคำสั่ง — ระหว่างท่าเคลื่อนไหว ถ้ามีคำสั่งใหม่เข้ามาให้ตัดทิ้งแล้วทำอันใหม่"""
        name = None
        while rclpy.ok():
            if name is None:
                try:
                    name = self.cmds.get(timeout=0.5)
                except queue.Empty:
                    continue
            nxt = None
            try:
                nxt = self._run(name)
            except Exception as exc:  # สายหลุด/ไฟดับกลางทาง — ไม่ให้ thread ตาย
                self.ready = False
                self.get_logger().error(f"สั่งมือไม่สำเร็จ ({name}): {exc}")
                self._status(f"error:{exc}")
            name = nxt

    def _run(self, name):
        """ทำท่า name — คืนชื่อคำสั่งใหม่ถ้ามีเข้ามาแทรกกลางท่าเคลื่อนไหว"""
        if name == "relax":
            if not self.dry_run:
                self.hand.relax()
            self.ready = False  # เปิดแรงบิดใหม่ (พร้อมเช็กไฟ) ตอนได้ท่าถัดไป
            self._status("relaxed")
            return None
        if not self.dry_run and not self.ready and not self._connect():
            return None
        taught = self._taught()
        # ท่าที่สอนเองชื่อเดียวกับท่าเคลื่อนไหว (victory / nonono) = ท่านิ่งตามที่สอน
        steps = [(name, 0.0)] if name in taught else hand_poses.ANIMATIONS.get(name, [(name, 0.0)])
        self._status(f"gesture:{name}")
        if not self.dry_run:
            off = self.hand.unloaded()
            if off:
                self.get_logger().warn(f"servo ID {off} แรงบิดหลุดอยู่ (servo ตัดเองตอนดันนานเกิน — ท่าก่อนหน้าสั่งเลย"
                                       "ที่นิ้วไปได้) — เปิดคืนแล้ว")
        for pose, hold in steps:
            degrees = self._degrees(pose, taught)
            if self.dry_run:
                self.get_logger().info(f"[dry_run] ท่า {pose}: " + " ".join(f"{d:+.0f}" for d in degrees)
                                       + (" (ท่าที่สอนเอง)" if pose in taught else ""))
            else:
                clipped = self.hand.move(degrees, self.speed, enable=self.hand.ids)
                if clipped:
                    self.get_logger().warn(f"ท่า {pose}: servo ID {clipped} เป้าหมายเกินขีดจำกัดของ servo "
                                           f"({self.hand.limits_text()}) — จำกัดไว้ที่ขอบ", throttle_duration_sec=10.0)
            end = time.monotonic() + hold
            while time.monotonic() < end:
                try:
                    return self.cmds.get(timeout=max(0.0, end - time.monotonic()))
                except queue.Empty:
                    pass
        return None if self.dry_run else self._check_reached(pose, degrees)

    def _check_reached(self, pose, degrees):
        """รอท่านิ่ง แล้วดูว่านิ้วไปถึงไหม — ไม่ถึง (ติดนิ้วอื่น / สุดทางกลไก) = ถือไว้ตรงที่ไปถึง ไม่ให้ servo
        ดันค้างจนตัดแรงบิดตัวเอง (นิ้วอ่อนไปทุกท่าหลังจากนั้น) · มีคำสั่งใหม่ระหว่างรอ = ไปทำคำสั่งนั้นเลย"""
        got = []

        def wait(t):
            try:
                got.append(self.cmds.get(timeout=t))
                return True
            except queue.Empty:
                return False

        present, still = self.hand.settle(wait=wait)
        if got:
            return got[0]
        off = self.hand.unloaded()
        if off:  # servo รีเซ็ตกลางท่า (ไฟตกตอนหลายนิ้วขยับ/ดันพร้อมกัน) — แรงบิดหลุด speed กลับค่าเริ่ม
            self.hand.torque(off, True)
            self.get_logger().warn(f"ท่า {pose}: servo ID {off} แรงบิดหลุดระหว่างทำท่า — ไฟมือน่าจะตก "
                                   "(แหล่งจ่ายไม่พอตอนนิ้วดันพร้อมกัน) · เปิดคืนแล้ว")
        stuck = self.hand.stuck(degrees, self.hand.ids, present) if still and not off else {}
        if stuck:
            self.hand.torque(list(stuck), True)  # เป้าหมาย = ตำแหน่งจริง (หยุดดัน)
            self.get_logger().warn(f"ท่า {pose}: นิ้วไปไม่ถึงที่สั่ง {self.hand.stuck_text(stuck, degrees)} "
                                   "— ถือไว้ตรงที่ไปถึง · สอนท่านี้ใหม่ (arm hand-teach: load แล้ว save)")
        return None

    def shutdown(self):
        if not self.dry_run:
            self.hand.relax()


def main():
    rclpy.init()
    node = AmazingHandNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()  # นิ้วอ่อนลงเอง (ท่าพักของมือนี้คือกำ)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
