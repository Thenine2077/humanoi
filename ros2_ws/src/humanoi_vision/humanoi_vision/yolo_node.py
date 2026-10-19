#!/usr/bin/env python3
"""YOLO บนการ์ด Hailo-10H ตรวจจับวัตถุจากภาพสีของกล้อง Astra Pro + หาพิกัด 3D จาก depth (รันใน container)

  # ตัวคิดบนการ์ด Hailo เปิดเองตอนบูต (systemd: scripts/hailo-yolo-server.service) — ใน container สั่งแค่
  # กล้อง + node นี้ + Foxglove
  ros2 launch humanoi_vision yolo.launch.py
  ros2 launch humanoi_vision yolo.launch.py classes:=bottle,cup     # เอาเฉพาะบางคลาส

รับ:  /camera/color/image_raw    (sensor_msgs/Image rgb8 จากไดรเวอร์ astra_camera)
      /camera/depth/image_raw    (16UC1 หน่วย mm — ต้องเปิด depth_registration ให้ตรงกับภาพสีทีละพิกเซล)
      /camera/color/camera_info  (ค่าเลนส์ fx fy cx cy ของกล้องสี)
ส่ง:  /yolo/detections           (vision_msgs/Detection2DArray) กรอบเป็นพิกเซลของภาพกล้อง + ชื่อคลาส + ความมั่นใจ
      /yolo/detections_3d        (vision_msgs/Detection3DArray) จุดกลางวัตถุเป็นเมตร ในเฟรม camera_color_optical_frame
      /yolo/markers              (visualization_msgs/MarkerArray) ลูกบอล + ชื่อ ไว้ดูใน 3D panel ของ Foxglove
      /yolo/image/compressed     (sensor_msgs/CompressedImage JPEG) ภาพที่วาดกรอบ + ระยะ ไว้ดูใน Foxglove

พิกัด 3D อยู่ในเฟรมของกล้อง (optical): X = ขวา, Y = ลง, Z = พุ่งออกจากเลนส์ — ไม่ขึ้นกับว่าหุ่นติดตั้งท่าไหน
จะเอาไปสั่งแขนให้แปลงด้วย TF (camera_color_optical_frame -> base_link ของแขน) ซึ่งได้จากการ calibrate
กล้องกับแขนภายหลัง จุดที่ได้คือ "ผิวหน้าวัตถุที่กล้องเห็น" ไม่ใช่ใจกลางวัตถุ

ของที่ใกล้กว่าระยะต่ำสุดของ depth (~55-60 cm = พื้นที่ที่แขนเอื้อมถึง) depth วัดไม่ได้ -> ใช้ผิวโต๊ะแทน
(table_mode=auto): หาผิวโต๊ะจาก depth ส่วนที่ไกล แล้วคิดตำแหน่งของที่ตั้งบนโต๊ะจากกรอบ YOLO (ดู table.py)
จุดที่ได้แบบนี้คือ "ใจกลางของที่ครึ่งความสูง" และมีป้าย [table] ในรายการบนจอ

การ์ด Hailo ต่อกับ HailoRT ที่ลงบน Raspberry Pi OS — container (Ubuntu) ใช้ตรงๆ ไม่ได้ node นี้จึงส่งภาพไปให้
scripts/hailo_yolo_server.py ผ่าน TCP 127.0.0.1:5555 (container ใช้ network แบบ host) ดูโปรโตคอลในไฟล์นั้น

หยิบ "ภาพล่าสุด" เสมอ ภาพที่มาระหว่างกำลังคิดถูกทิ้ง กรอบที่ส่งออกจึงไม่ช้ากว่าของจริงแม้คิดไม่ทัน 30 fps
"""
import json
import socket
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import ColorRGBA
from vision_msgs.msg import (Detection2D, Detection2DArray, Detection3D, Detection3DArray,
                             ObjectHypothesisWithPose)
from visualization_msgs.msg import Marker, MarkerArray

from humanoi_vision import table
from humanoi_vision.coco import COCO_CLASSES


@dataclass
class Det:
    name: str
    score: float
    box: Tuple[float, float, float, float]          # (x1, y1, x2, y2) พิกเซลบนภาพสี
    xyz: Optional[Tuple[float, float, float]] = None  # เมตร ในเฟรมกล้อง — None = วัดระยะไม่ได้
    source: str = ""                                  # "depth" = ผิวหน้าที่กล้องเห็น, "table" = ใจกลางจากผิวโต๊ะ
    size: Optional[Tuple[float, float]] = None        # (กว้าง, สูง) เมตร — มีเฉพาะแบบ table


def to_rgb(msg):
    """sensor_msgs/Image -> numpy RGB (ไม่ใช้ cv_bridge — แปลงเองสั้นกว่า)"""
    if msg.encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"ไม่รองรับ encoding {msg.encoding} (รับแค่ rgb8 / bgr8)")
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    img = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB) if msg.encoding == "bgr8" else img


def to_depth_mm(msg):
    """sensor_msgs/Image 16UC1 -> numpy uint16 หน่วย mm (0 = วัดไม่ได้)"""
    if msg.encoding not in ("16UC1", "mono16"):
        raise ValueError(f"depth encoding {msg.encoding} ไม่รองรับ (รับแค่ 16UC1 หน่วย mm)")
    rows = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)
    return rows[:, :msg.width]


def depth_in_box(depth, box, core=0.4):
    """ระยะ (mm) ของวัตถุในกรอบ = ค่ากลาง (median) ของพิกเซลที่วัดได้ในแกนกลางกรอบ — None ถ้าวัดไม่ได้

    ใช้แค่แกนกลาง 40%: ขอบกรอบมักติดพื้นหลัง/โต๊ะที่อยู่ไกลกว่า ส่วน median ทนต่อจุด depth ที่หลุด
    """
    x1, y1, x2, y2 = box
    mx, my = (x2 - x1) * (1 - core) / 2, (y2 - y1) * (1 - core) / 2
    patch = depth[int(y1 + my):int(np.ceil(y2 - my)), int(x1 + mx):int(np.ceil(x2 - mx))]
    valid = patch[patch > 0]
    return float(np.median(valid)) if valid.size >= 10 else None


class Letterbox:
    """ย่อภาพให้พอดีกรอบโมเดลโดยคงสัดส่วน แล้วเติมขอบสีเทา (แบบที่ YOLO เทรนมา) + แปลงพิกัดกลับ"""

    def __init__(self, src_w, src_h, dst_w, dst_h):
        self.scale = min(dst_w / src_w, dst_h / src_h)
        self.new_w, self.new_h = round(src_w * self.scale), round(src_h * self.scale)
        self.pad_x, self.pad_y = (dst_w - self.new_w) // 2, (dst_h - self.new_h) // 2
        self.dst_w, self.dst_h = dst_w, dst_h
        self.src_w, self.src_h = src_w, src_h
        self.canvas = np.full((dst_h, dst_w, 3), 114, np.uint8)

    def apply(self, img):
        if (self.new_w, self.new_h) == (img.shape[1], img.shape[0]):
            resized = img
        else:
            resized = cv2.resize(img, (self.new_w, self.new_h), interpolation=cv2.INTER_LINEAR)
        self.canvas[self.pad_y:self.pad_y + self.new_h, self.pad_x:self.pad_x + self.new_w] = resized
        return self.canvas

    def to_pixels(self, ymin, xmin, ymax, xmax):
        """พิกัด 0-1 บนภาพโมเดล -> (x1, y1, x2, y2) พิกเซลบนภาพกล้อง"""
        x1 = (xmin * self.dst_w - self.pad_x) / self.scale
        x2 = (xmax * self.dst_w - self.pad_x) / self.scale
        y1 = (ymin * self.dst_h - self.pad_y) / self.scale
        y2 = (ymax * self.dst_h - self.pad_y) / self.scale
        return (min(max(x1, 0.0), self.src_w), min(max(y1, 0.0), self.src_h),
                min(max(x2, 0.0), self.src_w), min(max(y2, 0.0), self.src_h))


class HailoClient:
    """ต่อ scripts/hailo_yolo_server.py — ส่งภาพที่ letterbox แล้ว รับรายการกรอบกลับ"""

    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=5.0)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.reader = self.sock.makefile("rb")
        # server เปิดการ์ด + โหลดโมเดลก่อนตอบ (ไม่กี่วินาที) — รอนานกว่ารอบปกติ
        self.sock.settimeout(20.0)
        line = self.reader.readline()
        self.sock.settimeout(5.0)
        if not line:
            self.close()
            raise ConnectionError("server ตัดการเชื่อมต่อก่อนตอบ")
        hello = json.loads(line)
        if "error" in hello:  # เช่น โปรแกรม Hailo อื่นถือการ์ดอยู่
            self.close()
            raise ConnectionError(hello["error"])
        self.h, self.w, _ = hello["input"]
        self.n_classes = hello["classes"]
        self.hef = hello["hef"]

    def infer(self, rgb):
        self.sock.sendall(memoryview(np.ascontiguousarray(rgb)).cast("B"))
        line = self.reader.readline()
        if not line:
            raise ConnectionError("hailo_yolo_server ปิดการเชื่อมต่อ")
        return json.loads(line)

    def close(self):
        try:
            self.reader.close()
            self.sock.close()
        except OSError:
            pass


class YoloNode(Node):
    def __init__(self):
        super().__init__("yolo_node")
        self.declare_parameter("server_host", "127.0.0.1")
        self.declare_parameter("server_port", 5555)
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("conf", 0.4)
        # คั่นด้วยจุลภาค เช่น "bottle,cup" — ว่าง = ทุกคลาส
        self.declare_parameter("classes", "")
        self.declare_parameter("jpeg_quality", 80)
        # true = เปิดหน้าต่างภาพ + รายการที่เจอบนจอ Pi (ต้องมี DISPLAY + XAUTHORITY — ใช้ scripts/yolo_view.sh)
        self.declare_parameter("show", False)
        # เมตร บวกเข้ากับระยะจาก depth: Astra Pro อ่านสั้นกว่าระยะจากจุดศูนย์เลนส์สี ~3 cm คงที่
        # (วัด 15 ต.ค.: PnP ตารางหมากรุก vs depth ต่าง 3.1-4.2 cm, ตลับเมตรถึงขวด 3 จุดตรงภายใน 1 cm เมื่อบวก 3.2 cm)
        self.declare_parameter("depth_offset", 0.0)
        # ของที่ depth วัดไม่ได้ (ใกล้เกิน) คิดจากผิวโต๊ะแทน: auto = เฉพาะตอน depth วัดไม่ได้, always = ทุกชิ้น
        # (ไว้เทียบกับ depth), off = ไม่ใช้ · หาผิวโต๊ะใหม่ทุก table_update วินาที (หุ่นย้ายไปโต๊ะอื่นได้)
        self.declare_parameter("table_mode", "auto")
        self.declare_parameter("table_update", 2.0)

        self.host = str(self._param("server_host"))
        self.port = int(self._param("server_port"))
        self.conf = float(self._param("conf"))
        self.jpeg_quality = int(self._param("jpeg_quality"))
        self.show = bool(self._param("show"))
        self.depth_offset = float(self._param("depth_offset"))
        self.table_mode = str(self._param("table_mode"))
        if self.table_mode not in ("auto", "always", "off"):
            raise ValueError(f"table_mode ต้องเป็น auto / always / off (ได้ {self.table_mode})")
        self.table_update = float(self._param("table_update"))
        self._table = None       # (n, d, สัดส่วน) ผิวโต๊ะในเฟรมกล้อง: n·X = d
        self._table_t = 0.0
        self._grid = None        # (u, v, รังสี) ของพิกเซลที่สุ่มมาหาผิวโต๊ะ — คิดครั้งเดียวต่อขนาดภาพ
        self.class_ids = self._class_ids(str(self._param("classes")))
        image_topic = str(self._param("image_topic"))

        self.det_pub = self.create_publisher(Detection2DArray, "/yolo/detections", 10)
        self.det3d_pub = self.create_publisher(Detection3DArray, "/yolo/detections_3d", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/yolo/markers", 2)
        self.img_pub = self.create_publisher(CompressedImage, "/yolo/image/compressed", 2)

        self._lock = threading.Lock()
        self._latest = None
        self._depth = None       # ภาพ depth ล่าสุด (sensor_msgs/Image)
        self._K = None           # (fx, fy, cx, cy) จาก camera_info
        self._Kmat = None        # เมทริกซ์ 3x3 + ค่าความโค้งเลนส์ (None = ไม่มี) ไว้แก้ความโค้งก่อนคำนวณ X/Y
        self._D = None
        self._warned = set()     # เตือนเรื่องเดิมครั้งเดียว
        self._new = threading.Event()
        self._running = True
        self.create_subscription(Image, image_topic, self._on_image, qos_profile_sensor_data)
        self.create_subscription(Image, str(self._param("depth_topic")), self._on_depth,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, str(self._param("camera_info_topic")), self._on_info, 10)
        self._worker = threading.Thread(target=self._loop, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f"รอภาพจาก {image_topic} · Hailo ที่ {self.host}:{self.port} · conf={self.conf} "
            f"depth_offset={100 * self.depth_offset:+.1f} cm "
            f"classes={self._param('classes') or 'ทั้งหมด'}")

    def _param(self, name):
        return self.get_parameter(name).value

    def _warn_once(self, key, text):
        if key not in self._warned:
            self._warned.add(key)
            self.get_logger().warning(text)

    def _class_ids(self, classes):
        wanted = [c.strip() for c in classes.split(",") if c.strip()]
        if not wanted:
            return None
        unknown = [c for c in wanted if c not in COCO_CLASSES]
        if unknown:
            raise ValueError(f"ไม่รู้จักคลาส {unknown} — ที่มี: {', '.join(COCO_CLASSES)}")
        return {COCO_CLASSES.index(c) for c in wanted}

    def _on_image(self, msg):
        with self._lock:
            self._latest = msg
        self._new.set()

    def _on_depth(self, msg):
        with self._lock:
            self._depth = msg

    def _on_info(self, msg):
        fx, fy, cx, cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
        if not all(np.isfinite([fx, fy, cx, cy])) or fx <= 0 or fy <= 0:
            self._warn_once("bad_info", f"camera_info ใช้ไม่ได้ (fx={fx}) — ยังหาพิกัด 3D ไม่ได้")
            return
        dist = np.array(msg.d, np.float64)
        if self._K is None:
            self.get_logger().info(
                f"ค่าเลนส์กล้องสี fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f} "
                f"distortion={'มี' if dist.size and np.any(dist) else 'ไม่มี'}"
                + (" (ค่ากลางของไดรเวอร์ — ยังไม่ได้ calibrate)" if abs(fx - 570.3422) < 0.01 else ""))
        self._K = (fx, fy, cx, cy)
        self._Kmat = np.array(msg.k, np.float64).reshape(3, 3)
        self._D = dist if dist.size and np.any(dist) else None

    def _connect(self):
        """ต่อ server จนกว่าจะได้ (หรือ node ถูกปิด) — เตือนเมื่อสาเหตุเปลี่ยน ไม่เตือนซ้ำทุกรอบ

        ลองใหม่ห่างขึ้นเรื่อยๆ 2 -> 10 วินาที: ถ้าโปรแกรม Hailo อื่นถือการ์ดอยู่นานๆ จะได้ไม่ไปกวนถี่เกิน
        """
        last_reason, delay = None, 2.0
        while self._running:
            try:
                client = HailoClient(self.host, self.port)
                self.get_logger().info(f"ต่อ Hailo แล้ว: {client.hef} (input {client.w}x{client.h})")
                return client
            except (OSError, ValueError) as e:
                if str(e) != last_reason:
                    self.get_logger().warning(
                        f"ใช้ Hailo ไม่ได้: {e} — เช็กบน Pi (นอก container): systemctl status hailo-yolo-server "
                        "· จะลองใหม่เรื่อยๆ")
                    last_reason = str(e)
                time.sleep(delay)
                delay = min(delay * 2, 10.0)
        return None

    def _loop(self):
        client, box = None, None
        n, hailo_ms, t0 = 0, 0.0, time.monotonic()
        fps, t_prev = 0.0, time.monotonic()
        while self._running:
            if client is None:
                client = self._connect()
                box = None
                if client is None:
                    break
            if not self._new.wait(timeout=1.0):
                continue
            with self._lock:
                msg, self._latest = self._latest, None
                depth_msg = self._depth
                self._new.clear()
            if msg is None:
                continue
            try:
                rgb = to_rgb(msg)
            except ValueError as e:
                self.get_logger().error(str(e), throttle_duration_sec=5.0)
                continue
            if box is None or (box.src_w, box.src_h) != (rgb.shape[1], rgb.shape[0]):
                box = Letterbox(rgb.shape[1], rgb.shape[0], client.w, client.h)

            try:
                reply = client.infer(box.apply(rgb))
            except (OSError, ConnectionError, ValueError) as e:
                self.get_logger().warning(f"Hailo หลุด ({e}) — ต่อใหม่")
                client.close()
                client = None
                continue

            dets = self._detections(reply["dets"], box)
            self._add_3d(dets, msg, depth_msg)
            now = time.monotonic()
            fps, t_prev = 0.9 * fps + 0.1 / max(now - t_prev, 1e-3), now
            hailo_ms += reply["ms"]
            n += 1
            try:
                self.det_pub.publish(self._to_msg(dets, msg.header))
                self.det3d_pub.publish(self._to_msg_3d(dets, msg.header))
                if self.marker_pub.get_subscription_count() > 0:
                    self.marker_pub.publish(self._markers(dets, msg.header))
                publish_img = self.img_pub.get_subscription_count() > 0
                if publish_img or self.show:
                    img = self._draw(rgb, dets, fps)
                    if publish_img:
                        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
                        if ok:
                            self.img_pub.publish(CompressedImage(header=msg.header, format="jpeg", data=jpg.tobytes()))
                    if self.show:  # คำสั่งหน้าต่างทุกตัวอยู่ใน thread นี้ (Qt)
                        cv2.imshow("YOLO (Ctrl+C in terminal to quit)", cv2.resize(img, (960, 720)))
                        cv2.waitKey(1)
            except Exception:  # Ctrl+C ปิด context ก่อน stop() ถูกเรียก — publish ระหว่างนั้นจะ error
                if self._running:
                    raise
                break

            now = time.monotonic()
            if now - t0 >= 5.0:
                found = ", ".join(self._describe(d) for d in dets) or "-"
                self.get_logger().info(
                    f"YOLO {n / (now - t0):.1f} fps (Hailo {hailo_ms / n:.0f} ms/ภาพ) เจอ: {found}")
                n, hailo_ms, t0 = 0, 0.0, now
        if client is not None:
            client.close()
        if self.show:
            cv2.destroyAllWindows()

    def _detections(self, raw, box):
        """[[class_id, score, ymin, xmin, ymax, xmax], ...] -> [Det] เรียงจากมั่นใจมากไปน้อย"""
        out = []
        for cls, score, ymin, xmin, ymax, xmax in raw:
            if score < self.conf or (self.class_ids is not None and cls not in self.class_ids):
                continue
            name = COCO_CLASSES[cls] if cls < len(COCO_CLASSES) else f"class_{cls}"
            out.append(Det(name, score, box.to_pixels(ymin, xmin, ymax, xmax)))
        out.sort(key=lambda d: -d.score)
        return out

    def _add_3d(self, dets, color_msg, depth_msg):
        """เติม Det.xyz จาก depth ที่ตรงกับภาพสี: X = (u-cx)Z/fx, Y = (v-cy)Z/fy, Z = ระยะ"""
        if not dets or self._K is None or depth_msg is None:
            return
        # depth ต้องอยู่เฟรมเดียวกับภาพสี (depth_registration) ไม่งั้นพิกเซล (u,v) ของ YOLO ชี้ผิดจุด
        if (depth_msg.header.frame_id != color_msg.header.frame_id
                or (depth_msg.width, depth_msg.height) != (color_msg.width, color_msg.height)):
            self._warn_once(
                "not_registered",
                f"depth ({depth_msg.header.frame_id} {depth_msg.width}x{depth_msg.height}) ไม่ตรงกับภาพสี "
                f"({color_msg.header.frame_id} {color_msg.width}x{color_msg.height}) — เปิดกล้องด้วย "
                "depth_registration:=true (ค่าเริ่มต้นของ camera.launch.py) · ยังไม่ส่งพิกัด 3D")
            return
        try:
            depth = to_depth_mm(depth_msg)
        except ValueError as e:
            self._warn_once("depth_encoding", str(e))
            return
        fx, fy, cx, cy = self._K
        if self.table_mode != "off" and time.monotonic() - self._table_t >= self.table_update:
            self._update_table(depth, dets)
        for d in dets:
            mm = depth_in_box(depth, d.box)
            if self._table is not None and (self.table_mode == "always" or (self.table_mode == "auto" and mm is None)):
                res = table.object_on_table(d.box, self._Kmat, self._dist(), self._table,
                                            depth.shape[1], depth.shape[0])
                if res is not None:
                    d.xyz, d.source, d.size = tuple(float(v) for v in res[0]), "table", (res[1], res[2])
                    continue
            if mm is None:
                continue
            d.source = "depth"
            z = mm / 1000.0 + self.depth_offset
            u, v = (d.box[0] + d.box[2]) / 2, (d.box[1] + d.box[3]) / 2
            if self._D is not None:
                # depth อ่านที่พิกเซลเดิม (จัดตรงกับภาพสีที่ยังโค้งอยู่) แต่ทิศของรังสีต้องคิดจากจุดที่แก้ความโค้งแล้ว
                u, v = cv2.undistortPoints(np.array([[[u, v]]], np.float64), self._Kmat, self._D,
                                           P=self._Kmat)[0, 0]
            d.xyz = ((u - cx) * z / fx, (v - cy) * z / fy, z)

    def _dist(self):
        return self._D if self._D is not None else np.zeros(5)

    def _update_table(self, depth, dets, step=8):
        """หาผิวโต๊ะจาก depth (ตัดพิกเซลในกรอบของที่เจอออก) — บอกใน log ตอนเจอครั้งแรกหรือเปลี่ยนเกิน 1 cm"""
        self._table_t = time.monotonic()
        h, w = depth.shape
        if self._grid is None or self._grid[0] != (w, h):
            vv, uu = np.mgrid[step // 2:h:step, step // 2:w:step]
            uv = np.column_stack([uu.ravel(), vv.ravel()])
            self._grid = ((w, h), uv, table.rays(uv, self._Kmat, self._dist()))
        _, uv, r = self._grid
        z = depth[uv[:, 1], uv[:, 0]].astype(np.float64) / 1000.0
        ok = z > 0
        for d in dets:
            x1, y1, x2, y2 = d.box
            ok &= ~((uv[:, 0] >= x1) & (uv[:, 0] <= x2) & (uv[:, 1] >= y1) & (uv[:, 1] <= y2))
        plane = table.fit_table(r[ok] * (z[ok] + self.depth_offset)[:, None])
        if plane is None:
            if self._table is None:
                self._warn_once("no_table", "ยังหาผิวโต๊ะไม่เจอ (ต้องเห็นโต๊ะส่วนที่ห่างเกิน ~60 cm) — "
                                "ของที่ใกล้กว่านั้นจะยังไม่มีพิกัด 3D")
            return
        if self._table is None or abs(plane[1] - self._table[1]) > 0.01:
            self.get_logger().info(
                f"ผิวโต๊ะ: กล้องสูงจากโต๊ะ {100 * -plane[1]:.1f} cm ก้ม {np.degrees(np.arcsin(abs(plane[0][2]))):.0f}° "
                f"(จุดบนโต๊ะ {100 * plane[2]:.0f}%)")
        self._table = plane

    @staticmethod
    def _describe(d):
        if d.xyz is None:
            return f"{d.name} {d.score:.2f}"
        x, y, z = d.xyz
        return f"{d.name} {d.score:.2f} @ ({x:+.3f}, {y:+.3f}, {z:.3f}) m" + (" [table]" if d.source == "table" else "")

    def _to_msg(self, dets, header):
        out = Detection2DArray(header=header)
        for d in dets:
            x1, y1, x2, y2 = d.box
            det = Detection2D(header=header)
            det.bbox.center.position.x = (x1 + x2) / 2
            det.bbox.center.position.y = (y1 + y2) / 2
            det.bbox.size_x = x2 - x1
            det.bbox.size_y = y2 - y1
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = d.name
            hyp.hypothesis.score = float(d.score)
            det.results.append(hyp)
            out.detections.append(det)
        return out

    def _to_msg_3d(self, dets, header):
        out = Detection3DArray(header=header)
        if self._K is None:
            return out
        fx, fy = self._K[0], self._K[1]
        for d in dets:
            if d.xyz is None:
                continue
            x1, y1, x2, y2 = d.box
            det = Detection3D(header=header)
            det.bbox.center.position.x, det.bbox.center.position.y, det.bbox.center.position.z = d.xyz
            det.bbox.center.orientation.w = 1.0
            if d.size is not None:  # แบบ table: กว้าง (เส้นผ่านศูนย์กลาง) x สูง จากทรงกระบอกที่ฟิตได้
                det.bbox.size.x = det.bbox.size.z = d.size[0]
                det.bbox.size.y = d.size[1]
            else:  # กว้าง/สูงโดยประมาณจากกรอบในภาพ ณ ระยะนั้น — ความลึกของวัตถุกล้องมองไม่เห็น ใส่ 0
                det.bbox.size.x = (x2 - x1) * d.xyz[2] / fx
                det.bbox.size.y = (y2 - y1) * d.xyz[2] / fy
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = d.name
            hyp.hypothesis.score = float(d.score)
            hyp.pose.pose = det.bbox.center
            det.results.append(hyp)
            out.detections.append(det)
        return out

    def _markers(self, dets, header):
        out = MarkerArray(markers=[Marker(header=header, action=Marker.DELETEALL)])
        for i, d in enumerate(d for d in dets if d.xyz is not None):
            x, y, z = d.xyz
            ball = Marker(header=header, ns="yolo", id=2 * i, type=Marker.SPHERE, action=Marker.ADD)
            ball.pose.position.x, ball.pose.position.y, ball.pose.position.z = x, y, z
            ball.pose.orientation.w = 1.0
            ball.scale.x = ball.scale.y = ball.scale.z = 0.04
            ball.color = ColorRGBA(r=0.0, g=0.85, b=0.0, a=1.0)
            label = Marker(header=header, ns="yolo", id=2 * i + 1, type=Marker.TEXT_VIEW_FACING,
                           action=Marker.ADD)
            label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y - 0.06, z  # -Y = ขึ้น
            label.pose.orientation.w = 1.0
            label.scale.z = 0.05
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label.text = f"{d.name} {z:.2f} m"
            out.markers += [ball, label]
        return out

    def _draw(self, rgb, dets, fps):
        """ภาพพร้อมกรอบ + รายการที่เจอ (มุมซ้ายบน) — ใช้ทั้ง /yolo/image/compressed และหน้าต่างบนจอ"""
        img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        for d in dets:
            x1, y1, x2, y2 = d.box
            p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
            cv2.rectangle(img, p1, p2, (0, 220, 0), 2)
            # ระยะเส้นตรงจากกล้อง (เทียบกับตลับเมตรได้ตรงๆ) — Z คือระยะตามแนวที่กล้องชี้ ซึ่งสั้นกว่าเมื่อของไม่อยู่กลางภาพ
            label = f"{d.name} {d.score:.2f}" + (f" {np.linalg.norm(d.xyz):.2f}m" if d.xyz else "")
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            ty = max(p1[1], th + 6)
            cv2.rectangle(img, (p1[0], ty - th - 6), (p1[0] + tw + 4, ty), (0, 220, 0), -1)
            cv2.putText(img, label, (p1[0] + 2, ty - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 0), 1, cv2.LINE_AA)
        # รายการ: ชื่อ ความมั่นใจ และพิกัดในเฟรมกล้อง (X ขวา, Y ลง, Z ไปข้างหน้า) — พื้นทึบให้อ่านออก (OpenCV 5)
        lines = [f"{len(dets)} found  {fps:.0f} fps  conf>={self.conf:.2f}"]
        for d in dets[:8]:
            lines.append(f"{d.name} {d.score:.2f}" + (
                "  X{:+.2f} Y{:+.2f} Z{:.2f}  dist {:.2f} m".format(*d.xyz, np.linalg.norm(d.xyz))
                + ("  [table]" if d.source == "table" else "") if d.xyz else "  (no depth)"))
        w = max(cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)[0][0] for t in lines) + 10
        cv2.rectangle(img, (0, 0), (w, 18 * len(lines) + 6), (0, 0, 0), -1)
        for i, t in enumerate(lines):
            cv2.putText(img, t, (5, 17 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (0, 255, 255) if i == 0 else (255, 255, 255), 1, cv2.LINE_AA)
        return img

    def stop(self):
        self._running = False
        self._new.set()
        self._worker.join(timeout=5.0)


def main():
    rclpy.init()
    try:
        node = YoloNode()
    except ValueError as e:  # ชื่อคลาสผิด
        rclpy.logging.get_logger("yolo_node").fatal(str(e))
        rclpy.shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
