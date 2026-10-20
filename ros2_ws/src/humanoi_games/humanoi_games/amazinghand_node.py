#!/usr/bin/env python3
"""amazinghand_node — คุมมือ AmazingHand (8× Feetech SCS0009 ผ่าน Waveshare Bus Servo Adapter) จาก ROS

ตัวนี้ตัวเดียวเป็นเจ้าของพอร์ตมือ (เปิดพอร์ตได้ทีละโปรแกรม — ปิด tools\\*.py / robot_hand.py ก่อน)

  ros2 run humanoi_games amazinghand_node                                  # หาพอร์ตเอง (CH343 1a86:55d3)
  ros2 run humanoi_games amazinghand_node --ros-args -p dry_run:=true      # ไม่ต่อมือ แค่ log ว่าจะทำท่าอะไร
  ros2 topic pub --once /hand/gesture std_msgs/msg/String "{data: paper}"

หัวข้อ
  /hand/gesture (sub, String)  ชื่อท่าใน hand_poses (rock paper scissors like point victory nonono ...)
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

ADAPTER_USB = (0x1A86, 0x55D3)  # Waveshare Bus Servo Adapter (A) = CH343 — Nano ของแขนเป็น CH340 (55d3 ≠ 7523)
FACTORY_LIMIT = 144.0  # ลิมิตโรงงาน SCS0009 (raw 20-1003) — ใช้เฉพาะตอนอ่านจาก servo ไม่ได้/ค่าแปลก
LIMIT_MARGIN = 1.0     # เผื่อการปัดเศษตอนแปลงหน่วย


def calib_file():
    """ไฟล์คาลิเบรตมือ — อยู่ใน src (ไม่ใช่ install) เหมือน programs.yaml: build ใหม่ไม่ทับ

    ลำดับ: ตัวแปร HAND_CALIB_FILE > Pi (Docker, /humanoi_ws/src) > PC (WSL) > ~/.config
    """
    env = os.environ.get("HAND_CALIB_FILE")
    if env:
        return env
    for src in ("/humanoi_ws/src", "/mnt/c/Users/User/Desktop/Humanoi/ros2_ws/src"):
        if os.path.isdir(os.path.join(src, "humanoi_games")):
            return os.path.join(src, "humanoi_games", "config", "middle_pos.json")
    return os.path.expanduser("~/.config/humanoi_games/middle_pos.json")


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


def find_port():
    from serial.tools import list_ports
    for p in list_ports.comports():
        if (p.vid, p.pid) == ADAPTER_USB:
            return p.device
    return None


def scalar(value):
    return value[0] if isinstance(value, list) else value  # rustypot read_* คืน list หนึ่งค่า


class Hand:
    """ห่อ rustypot: เปิดพอร์ต → เช็กไฟ → ถือท่าปัจจุบัน → เปิดแรงบิด (นิ้วไม่กระตุก)"""

    def __init__(self, port, baud, ids, min_volts, max_volts):
        self.port, self.baud, self.ids = port, baud, ids
        self.min_volts, self.max_volts = min_volts, max_volts
        self.c = None
        self.found_baud = None
        self.limits = {sid: (-FACTORY_LIMIT, FACTORY_LIMIT) for sid in ids}  # องศา ต่ำสุด/สูงสุดของแต่ละ servo
        self.limit_warnings = []

    def connect(self):
        from rustypot import Scs0009PyController
        port = find_port() if self.port == "auto" else self.port
        if not port:
            raise RuntimeError("ไม่เจอบอร์ดมือ (Waveshare CH343 1a86:55d3) — เสียบ USB ของมือ")
        if self.c is None:
            self.c = Scs0009PyController(serial_port=port, baudrate=self.baud, timeout=0.05)
        found = self._find_baud()
        if found is None:
            raise RuntimeError(f"ไม่มีเซอร์โวตอบเลยทุก baud ({port}) — เปิดไฟมือหรือยัง, jumper อยู่ที่ B (USB), "
                               "สายเซอร์โวเสียบช่อง D V G")
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

    def move(self, degrees, speed):
        """สั่งทุก servo — เป้าหมายที่เกินขีดจำกัดของ servo ตัวนั้นถูกจำกัดไว้ที่ขอบ คืน ID ที่ถูกจำกัด (ไว้เตือน)"""
        for sid in self.ids:
            self.c.write_goal_speed(sid, speed)
        clipped = [sid for sid, d in zip(self.ids, degrees) if not self.within(sid, d)]
        degrees = [max(self.limits[sid][0] + LIMIT_MARGIN, min(self.limits[sid][1] - LIMIT_MARGIN, d))
                   for sid, d in zip(self.ids, degrees)]
        rads = [math.radians(d) for d in degrees]
        try:
            self.c.sync_write_goal_position(self.ids, rads)
        except Exception:
            for sid, r in zip(self.ids, rads):
                self.c.write_goal_position(sid, r)
        return clipped

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
        self.declare_parameter("port", "auto")
        self.declare_parameter("baud", 1_000_000)
        self.declare_parameter("ids", [1, 2, 3, 4, 5, 6, 7, 8])
        # แบสุด / กำสุด / เอียง ต่อนิ้ว จาก hand_calibrate (ไม่มีไฟล์ = ท่าเหมือนเดโมของ Pollen)
        self.declare_parameter("calib_file", calib_file())
        self.declare_parameter("side", "right")
        self.declare_parameter("speed", 7.0)  # เท่า MaxSpeed ในเดโม
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
        self.side = gp("side").value
        self.speed = float(gp("speed").value)
        self.dry_run = bool(gp("dry_run").value)
        self.hand = Hand(gp("port").value, int(gp("baud").value), list(gp("ids").value),
                         float(gp("min_volts").value), float(gp("max_volts").value))
        self.ready = False
        self.cmds = queue.Queue()
        self.status_pub = self.create_publisher(String, "/hand/status", 10)
        self.create_subscription(String, "/hand/gesture", self._on_gesture, 10)
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
        self.get_logger().info(f"มือพร้อม ({port} @ {self.hand.found_baud}, ID {self.hand.ids}, {self.side}) · "
                               f"ขีดจำกัดจาก servo: {self.hand.limits_text()}")
        for w in self.hand.limit_warnings:
            self.get_logger().warn(w + " — ใช้ลิมิตโรงงาน ±144°")
        self._status("ready")
        return True

    def _on_gesture(self, msg):
        name = msg.data.strip()
        if name != "relax" and name not in hand_poses.names():
            self.get_logger().warn(f"ไม่รู้จักท่า '{name}' — มี: relax, {', '.join(hand_poses.names())}")
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
        steps = hand_poses.ANIMATIONS.get(name, [(name, 0.0)])
        self._status(f"gesture:{name}")
        for pose, hold in steps:
            if self.dry_run:
                self.get_logger().info(f"[dry_run] ท่า {pose}: " + " ".join(
                    f"{d:+.0f}" for d in hand_poses.servo_degrees(pose, self.calib, self.side)))
            else:
                clipped = self.hand.move(hand_poses.servo_degrees(pose, self.calib, self.side), self.speed)
                if clipped:
                    self.get_logger().warn(f"ท่า {pose}: servo ID {clipped} เป้าหมายเกินขีดจำกัดของ servo "
                                           f"({self.hand.limits_text()}) — จำกัดไว้ที่ขอบ", throttle_duration_sec=10.0)
            end = time.monotonic() + hold
            while time.monotonic() < end:
                try:
                    return self.cmds.get(timeout=max(0.0, end - time.monotonic()))
                except queue.Empty:
                    pass
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
