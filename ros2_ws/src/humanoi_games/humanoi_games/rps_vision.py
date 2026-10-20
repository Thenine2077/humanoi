#!/usr/bin/env python3
"""rps_vision — ดูมือผู้เล่นจากกล้องแล้วบอกว่าออก ค้อน/กระดาษ/กรรไกร (MediaPipe HandLandmarker บน CPU)

  ros2 run humanoi_games rps_vision                 # ภาพจาก /camera/color/image_raw (กล้อง Astra บนเสา)
  ros2 topic echo /rps/player_gesture

หัวข้อ
  image_topic                (sub, Image)            ค่าเริ่ม /camera/color/image_raw (rgb8 หรือ bgr8)
  /rps/player_gesture        (pub, String)           ทุกเฟรมที่ประมวลผล: rock | paper | scissors | none
  /rps/vision/image/compressed (pub, CompressedImage) ภาพพร้อมจุดมือและท่าที่อ่านได้ (เฉพาะตอนมีคนดู เช่น Foxglove)

ต้องมี mediapipe (ติดตั้งแบบไม่ลาก numpy 2 มาทับของ ROS):
  pip3 install --break-system-packages --no-deps mediapipe==1.1.1 "absl-py~=2.3" "flatbuffers~=25.9"
แยกท่าแบบเดียวกับ Fun thing\\robot_hand.py: นับนิ้วที่เหยียด (ระยะข้อมือ→ปลายนิ้ว เทียบข้อมือ→ข้อกลางนิ้ว)
ไม่ดูนิ้วโป้ง — กำ = ค้อน, 4 นิ้วเหยียด = กระดาษ, ชี้+กลางเหยียด นาง+ก้อยพับ = กรรไกร, อย่างอื่น = none
"""
import math
import os
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

# จุด landmark ของ MediaPipe: (ข้อกลางนิ้ว PIP, ปลายนิ้ว)
FINGERS = {"index": (6, 8), "middle": (10, 12), "ring": (14, 16), "pinky": (18, 20)}
BONES = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11), (11, 12),
         (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (17, 18), (18, 19), (19, 20), (0, 17)]
LABEL = {"rock": "ROCK", "paper": "PAPER", "scissors": "SCISSORS", "none": "-"}


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def extended(pts):
    """นิ้วไหนเหยียด — เทียบระยะจากข้อมือ จึงใช้ได้แม้มือเอียงหรือหันข้าง (pts = 21 จุด (x, y) พิกเซล)"""
    wrist = pts[0]
    return {f: _dist(wrist, pts[tip]) > _dist(wrist, pts[pip]) * 1.1 for f, (pip, tip) in FINGERS.items()}


def classify(pts):
    up = extended(pts)
    fingers = [up["index"], up["middle"], up["ring"], up["pinky"]]
    if not any(fingers):
        return "rock"
    if all(fingers):
        return "paper"
    if fingers == [True, True, False, False]:
        return "scissors"
    return "none"


def default_model():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("humanoi_games"), "models", "hand_landmarker.task")


class RpsVision(Node):
    def __init__(self):
        super().__init__("rps_vision")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("model_path", "")
        self.declare_parameter("max_fps", 15.0)  # Pi 5 CPU ได้ราว 15-25 fps ที่ 640×480
        self.declare_parameter("min_confidence", 0.5)
        try:
            import mediapipe as mp
            from mediapipe.tasks import python as mp_python
            from mediapipe.tasks.python import vision
        except ImportError as exc:
            raise SystemExit(f'โหลด mediapipe ไม่ได้ ({exc}) — ติดตั้ง: pip3 install --break-system-packages '
                             '--no-deps mediapipe==1.1.1 "absl-py~=2.3" "flatbuffers~=25.9"')
        self.mp = mp
        model = self.get_parameter("model_path").value or default_model()
        conf = float(self.get_parameter("min_confidence").value)
        self.landmarker = vision.HandLandmarker.create_from_options(vision.HandLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=model),
            running_mode=vision.RunningMode.VIDEO, num_hands=1,
            min_hand_detection_confidence=conf, min_hand_presence_confidence=conf, min_tracking_confidence=conf))
        self.period = 1.0 / float(self.get_parameter("max_fps").value)
        self.latest = None
        self.cv = threading.Condition()
        self.pub = self.create_publisher(String, "/rps/player_gesture", 10)
        self.img_pub = self.create_publisher(CompressedImage, "/rps/vision/image/compressed", 2)
        topic = self.get_parameter("image_topic").value
        self.create_subscription(Image, topic, self._on_image, qos_profile_sensor_data)
        self.running = True
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()
        self.get_logger().info(f"rps_vision พร้อม — ภาพจาก {topic}, model {model}")

    def _on_image(self, msg):
        with self.cv:  # เก็บแค่ภาพล่าสุด ประมวลผลไม่ทันก็ข้ามเฟรมเก่า
            self.latest = msg
            self.cv.notify()

    def _worker(self):
        t0, last = time.monotonic(), 0.0
        while self.running and rclpy.ok():
            with self.cv:
                self.cv.wait_for(lambda: self.latest is not None, timeout=0.5)
                msg, self.latest = self.latest, None
            if msg is None:
                continue
            wait = self.period - (time.monotonic() - last)
            if wait > 0:
                time.sleep(wait)
            last = time.monotonic()
            try:
                self._process(msg, int((last - t0) * 1000))
            except Exception as exc:
                self.get_logger().error(f"ประมวลผลภาพไม่ได้: {exc}", throttle_duration_sec=5.0)

    def _process(self, msg, ts_ms):
        if msg.encoding not in ("rgb8", "bgr8"):
            raise ValueError(f"encoding {msg.encoding} — ต้องเป็น rgb8/bgr8")
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        img = img.reshape(msg.height, msg.width, 3)
        rgb = np.ascontiguousarray(img if msg.encoding == "rgb8" else img[:, :, ::-1])
        res = self.landmarker.detect_for_video(self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb), ts_ms)
        pts = None
        gesture = "none"
        if res.hand_landmarks:
            pts = [(lm.x * msg.width, lm.y * msg.height) for lm in res.hand_landmarks[0]]
            gesture = classify(pts)
        self.pub.publish(String(data=gesture))
        if self.img_pub.get_subscription_count():
            self._publish_image(rgb, pts, gesture, msg.header)

    def close(self):
        """หยุด thread แล้วปิด MediaPipe เอง — ถ้าปล่อยให้ปิดตอน Python กำลังออก จะ TypeError ใน ctypes"""
        self.running = False
        self.thread.join(timeout=2.0)
        try:
            self.landmarker.close()
        except Exception:
            pass

    def _publish_image(self, rgb, pts, gesture, header):
        import cv2
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if pts:
            p = [(int(x), int(y)) for x, y in pts]
            for a, b in BONES:
                cv2.line(bgr, p[a], p[b], (0, 220, 255), 2)
            for q in p:
                cv2.circle(bgr, q, 3, (255, 255, 255), -1)
        cv2.putText(bgr, LABEL[gesture], (12, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
        ok, jpg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            self.img_pub.publish(CompressedImage(header=header, format="jpeg", data=jpg.tobytes()))


def main():
    rclpy.init()
    node = RpsVision()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
