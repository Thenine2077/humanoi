#!/usr/bin/env python3
"""rps_hud — จอบอกสถานะเกมเป่ายิ้งฉุบ ให้ผู้เล่น/คนรอบๆ ดูใน Foxglove (Image panel)

  ros2 run humanoi_games rps_hud          (rps.launch.py เปิดให้เอง)
  Foxglove → Image panel → /rps/hud/image/compressed

หัวข้อ
  /rps/state           (sub, String)  จังหวะเกมจาก rps_game (ready, pump:<ตา>:<เวลาจังหวะ>, beat:N, reveal:<ท่า>,
                                      result:<ผล>:<หุ่น>:<ผู้เล่น>, pose:, return:, waiting_replay:<วินาที>, idle)
  /rps/result          (sub, String)  ผลสุดท้าย "<หุ่น>:<ผู้เล่น>:<ผล>" — นับคะแนน
  /rps/player_gesture  (sub, String)  ท่าที่กล้องเห็นตอนนี้ (บอกตอนรอชูนิ้วชี้)
  /rps/hud/image/compressed (pub, CompressedImage)  ภาพจอ — วาดเฉพาะตอนมีคนดู

นับถอยหลัง 3-2-1 ตามจังหวะแขนจริง: rps_game ส่งเวลาจังหวะทั้งหมดตอนแขนเริ่มเป่า (pump:) แล้วจอนับเอง
ตัวหนังสือภาษาอังกฤษ (ฟอนต์ไทยไม่มีใน container) — ใช้ Pillow + DejaVu ถ้ามี ไม่มีใช้ฟอนต์ของ OpenCV
"""
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:  # ไม่มี Pillow = วาดด้วย OpenCV (ตัวหนังสือสวยน้อยกว่า)
    Image = None

W, H = 1280, 720
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
NAME = {"rock": "ROCK", "paper": "PAPER", "scissors": "SCISSORS", "point": "ONE FINGER", "none": "-", None: "-"}
# สี (R, G, B)
BG = (18, 20, 28)
WHITE, GREY, DIM = (240, 240, 240), (150, 155, 165), (70, 75, 90)
GREEN, RED, YELLOW, BLUE, ORANGE = (60, 200, 110), (230, 70, 70), (240, 200, 60), (80, 150, 240), (245, 140, 50)
RESULT = {  # ผล (มุมผู้เล่น) -> (ข้อความใหญ่, สี)
    "player_win": ("YOU WIN!", GREEN),
    "robot_win": ("YOU LOSE", RED),
    "draw": ("DRAW!", YELLOW),
    "no_player": ("NO HAND SEEN", GREY),
}


class Hud:
    """สถานะจอ — ป้อนข้อความจากหัวข้อเข้า on_*() แล้ว frame() วาดภาพ ณ เวลานั้น (แยกจาก ROS ไว้ทดสอบได้)"""

    def __init__(self):
        self.phase = "idle"
        self.game = 0
        self.attempt = 1
        self.beats, self.pump_t0 = [], 0.0
        self.robot = self.player = None
        self.result = None
        self.replay_t0, self.replay_s = 0.0, 0.0
        self.message = ""
        self.score = {"player_win": 0, "robot_win": 0, "draw": 0}
        self.seen, self.seen_t = None, 0.0
        self.after_replay = False

    def on_state(self, text, now):
        kind, _, rest = text.partition(":")
        if kind == "ready":
            self.phase, self.game, self.attempt = "ready", self.game + 1, 1
            self.robot = self.player = self.result = None
            self.after_replay = False
        elif kind == "pump":
            attempt, _, beats = rest.partition(":")
            self.phase, self.attempt, self.pump_t0 = "pump", int(attempt or 1), now
            self.beats = [float(b) for b in beats.split(",") if b]
            self.robot = None
        elif kind == "reveal":
            self.robot = rest or None  # จอยังนับถอยหลังต่อจนถึงจังหวะสุดท้าย (มือออกท่าก่อนนิดหน่อย)
        elif kind == "result":
            res, _, rest = rest.partition(":")
            robot, _, player = rest.partition(":")
            self.phase, self.result = "result", res
            self.robot = robot or self.robot
            self.player = player or None
        elif kind == "waiting_replay":
            self.phase, self.replay_t0 = "replay", now
            self.replay_s = float(rest or 0)
        elif kind == "idle":
            self.after_replay = self.phase == "replay"  # หมดเวลารอนิ้วชี้
            if self.phase not in ("stopped", "error"):
                self.phase = "idle"

    def on_result(self, text):
        parts = text.split(":", 2)
        res = parts[2] if len(parts) == 3 else text
        if res in self.score:
            self.score[res] += 1
        elif res == "stopped":
            self.phase, self.message = "stopped", ""
        elif res.startswith("error"):
            self.phase, self.message = "error", res[6:]

    def on_seen(self, gesture, now):
        self.seen, self.seen_t = gesture, now

    def view(self, now):
        """-> (หัวเรื่องเล็ก, ข้อความใหญ่, สี, คำอธิบาย, แถบเวลา 0..1 หรือ None)"""
        label = f"GAME {self.game}" + (f"  ·  TRY {self.attempt}" if self.attempt > 1 else "")
        if self.phase == "ready":
            return label, "ROUND BEGIN", BLUE, "Get your hand ready in front of the camera", None
        if self.phase == "pump" and self.beats:
            t = now - self.pump_t0
            n = len(self.beats)
            done = sum(1 for b in self.beats if t >= b)
            if done < n:
                start = self.beats[done - 1] if done else 0.0
                frac = (t - start) / max(1e-3, self.beats[done] - start)
                words = ["ROCK...", "PAPER...", "SCISSORS..."] if n == 3 else [""] * n
                return label, str(n - done), ORANGE, words[done], min(1.0, frac)
            return label, "SHOOT!", ORANGE, f"Robot shows {NAME.get(self.robot, '?')}", None
        if self.phase == "result" and self.result:
            big, color = RESULT.get(self.result, (self.result.upper(), GREY))
            sub = f"Robot {NAME.get(self.robot, '?')}   vs   You {NAME.get(self.player, '-')}"
            if self.result == "draw":
                sub += "   -   again!"
            return label, big, color, sub, None
        if self.phase == "replay":
            if self.replay_s <= 0:
                return "PLAY AGAIN?", "1", BLUE, "Hold up ONE finger to play again", None
            left = max(0.0, self.replay_s - (now - self.replay_t0))
            fresh = now - self.seen_t < 0.6
            sub = "Hold up ONE finger to play again"
            if fresh and self.seen == "point":
                sub = "ONE finger seen - hold it..."
            return "PLAY AGAIN?", f"{left:.0f}", BLUE, sub, left / self.replay_s
        if self.phase == "stopped":
            return "", "STOPPED", GREY, "Start again: arm rps play", None
        if self.phase == "error":
            return "", "ERROR", RED, self.message[:70], None
        if self.after_replay:
            return "", "TIME OUT", GREY, "Start again: arm rps play", None
        return "", "ROCK  PAPER  SCISSORS", WHITE, "Waiting for the game to start", None

    def frame(self, now):
        """ภาพจอ (BGR numpy) ณ เวลา now"""
        title, big, color, sub, bar = self.view(now)
        score = (f"YOU {self.score['player_win']}   ROBOT {self.score['robot_win']}   "
                 f"DRAW {self.score['draw']}")
        if Image is None:
            return _frame_cv(title, big, color, sub, bar, score)
        img = Image.new("RGB", (W, H), BG)
        d = ImageDraw.Draw(img)
        d.rectangle([0, 0, W, 70], fill=(30, 33, 45))
        d.text((32, 35), "ROCK PAPER SCISSORS", font=_font(30), fill=GREY, anchor="lm")
        d.text((W - 32, 35), score, font=_font(30), fill=WHITE, anchor="rm")
        if title:
            d.text((W // 2, 140), title, font=_font(40), fill=GREY, anchor="mm")
        size = 260 if len(big) <= 3 else (150 if len(big) <= 9 else 96)
        d.text((W // 2, 360), big, font=_font(size), fill=color, anchor="mm")
        if sub:
            d.text((W // 2, 560), sub, font=_font(44), fill=WHITE, anchor="mm")
        if bar is not None:
            x0, x1, y = 140, W - 140, 640
            d.rounded_rectangle([x0, y, x1, y + 22], radius=11, fill=DIM)
            d.rounded_rectangle([x0, y, x0 + max(22, int((x1 - x0) * bar)), y + 22], radius=11, fill=color)
        return cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR)


_FONTS = {}


def _font(size):
    if size not in _FONTS:
        try:
            _FONTS[size] = ImageFont.truetype(FONT, size)
        except OSError:
            _FONTS[size] = ImageFont.load_default(size)
    return _FONTS[size]


def _frame_cv(title, big, color, sub, bar, score):
    """สำรองตอนไม่มี Pillow — ฟอนต์ของ OpenCV (ไม่มีขอบตัวหนังสือ: ขอบทำให้ตัวอักษรเพี้ยน)"""
    img = np.full((H, W, 3), BG[::-1], np.uint8)
    bgr = color[::-1]

    def center(text, y, scale, col, thick):
        (w, h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_DUPLEX, scale, thick)
        cv2.putText(img, text, ((W - w) // 2, y + h // 2), cv2.FONT_HERSHEY_DUPLEX, scale, col, thick, cv2.LINE_AA)

    cv2.putText(img, score, (32, 48), cv2.FONT_HERSHEY_DUPLEX, 1.0, WHITE[::-1], 2, cv2.LINE_AA)
    if title:
        center(title, 140, 1.4, GREY[::-1], 2)
    center(big, 360, 8.0 if len(big) <= 3 else (4.0 if len(big) <= 9 else 2.6), bgr, 10 if len(big) <= 3 else 6)
    if sub:
        center(sub, 560, 1.3, WHITE[::-1], 2)
    if bar is not None:
        cv2.rectangle(img, (140, 640), (W - 140, 662), DIM[::-1], -1)
        cv2.rectangle(img, (140, 640), (140 + max(22, int((W - 280) * bar)), 662), bgr, -1)
    return img


class HudNode(Node):
    def __init__(self):
        super().__init__("rps_hud")
        self.declare_parameter("rate", 10.0)
        self.hud = Hud()
        self.create_subscription(String, "/rps/state", lambda m: self.hud.on_state(m.data, time.monotonic()), 20)
        self.create_subscription(String, "/rps/result", lambda m: self.hud.on_result(m.data), 10)
        self.create_subscription(String, "/rps/player_gesture",
                                 lambda m: self.hud.on_seen(m.data.strip(), time.monotonic()), 20)
        self.pub = self.create_publisher(CompressedImage, "/rps/hud/image/compressed", 2)
        self.create_timer(1.0 / float(self.get_parameter("rate").value), self._tick)
        self.get_logger().info("rps_hud พร้อม — Foxglove: Image panel → /rps/hud/image/compressed"
                               + ("" if Image else " (ไม่มี Pillow ใช้ฟอนต์ OpenCV)"))

    def _tick(self):
        if not self.pub.get_subscription_count():  # ไม่มีใครดู ไม่ต้องวาด
            return
        ok, jpg = cv2.imencode(".jpg", self.hud.frame(time.monotonic()), [cv2.IMWRITE_JPEG_QUALITY, 85])
        if ok:
            msg = CompressedImage(format="jpeg", data=jpg.tobytes())
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "rps_hud"
            self.pub.publish(msg)


def main():
    rclpy.init()
    node = HudNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
