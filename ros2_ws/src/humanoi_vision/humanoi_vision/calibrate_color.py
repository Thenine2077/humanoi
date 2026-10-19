#!/usr/bin/env python3
"""calibrate ค่าเลนส์กล้องสีของ Astra Pro (fx fy cx cy + distortion) ด้วยตารางหมากรุก — เก็บภาพเองอัตโนมัติ

  ros2 launch humanoi_vision calibrate_color.launch.py     # กล้อง + ตัวนี้ + Foxglove (ใน container)
  ดูใน Foxglove: Image panel ที่ /calibration/image/compressed (เห็นมุมที่จับได้ + แถบความครอบคลุม)

ทำไมต้องทำ: กล้องตัวนี้ไม่มีค่าเลนส์จากโรงงาน (/camera/get_camera_params ได้ NaN) ไดรเวอร์จึงใช้ค่ากลาง
fx=fy=570.34 — พิกัด X/Y ของ yolo_node จะเพี้ยนตามสัดส่วนที่ fx ผิด

กระดาษ: docs/calibration/checkerboard_9x6_25mm.pdf (มุมด้านใน 9x6 ช่องละ 25 mm) ติดบนแผ่นแข็งที่เรียบ
วิธีถือ: ห่างกล้อง ~0.4-1.2 m ค่อยๆ ขยับให้ทั่วภาพ (ซ้าย/ขวา/บน/ล่าง/มุม) ทั้งใกล้และไกล และเอียงกระดาษ
หลายทิศ (ก้ม/เงย/หมุนซ้ายขวา ~30°) หยุดนิ่งครู่หนึ่งทุกท่า — เก็บภาพเองเมื่อ: เห็นครบทุกมุม + นิ่ง + ท่าต่างจากเดิมพอ

ครบ (30-40 ภาพ) แล้วคำนวณ จากนั้นตรวจกับของจริง: ระยะตารางที่คิดจากค่าเลนส์ใหม่ต้องตรงกับระยะที่ depth วัดได้
(ต่างเกิน 4% = ไม่ใช้ผล เขียนเป็น .rejected แทน) ผ่านแล้วเขียนไฟล์แบบ camera_info_manager ที่ out
(ค่าเริ่ม = humanoi_description/config/astra_color.yaml)
camera.launch.py โหลดไฟล์นี้ให้ไดรเวอร์เอง (หลัง colcon build humanoi_description หนึ่งครั้งตามที่ log บอก)
ขนาดช่องไม่กระทบค่าเลนส์ (fx fy cx cy ไม่ขึ้นกับขนาดตาราง) แต่การตรวจกับ depth ขึ้นกับมันตรงๆ — square ต้องเป็น
ขนาดที่วัดจริงบนกระดาษ (เครื่องพิมพ์มักย่อ: ตาราง 30 mm ออกมา 28.2 mm) และกระดาษต้องเรียบ
ผลยังค้านกับ depth อยู่ -> ตรวจด้วยตลับเมตรว่าใครคลาด: ros2 run humanoi_vision check_depth_scale
"""
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

DEFAULT_FX = 570.3422047415297   # ค่ากลางของไดรเวอร์ — ไว้เทียบว่าคลาดไปเท่าไร
# ช่วงที่อยากให้ครอบคลุมในแต่ละแกน (ตาม camera_calibration ของ ROS): x, y = ตำแหน่งกลางกระดาษ (0-1),
# size = ขนาดกระดาษในภาพ, skew = ความเอียง (0-1)
COVER_SPAN = {"x": 0.6, "y": 0.6, "size": 0.25, "skew": 0.5}
MAX_SAMPLES = 40   # ครบเท่านี้คำนวณเลยแม้บางแกนยังไม่เต็ม (ROS ก็ใช้ 40)
SAMPLES_FILE = "/tmp/calibrate_color_samples.npz"   # มุมตาราง + depth ของทุกภาพที่ใช้คำนวณรอบล่าสุด
HINTS = {  # สิ่งที่ให้ทำเมื่อแกนนั้นยังครอบคลุมน้อยสุด (ไทยไว้ใน log, อังกฤษไว้บนภาพ — OpenCV เขียนไทยไม่ได้)
    "x": ("ขยับกระดาษไปทางซ้าย/ขวาของภาพให้สุด", "move LEFT / RIGHT"),
    "y": ("ขยับกระดาษขึ้น/ลงให้สุดขอบภาพ", "move UP / DOWN"),
    "size": ("ถือใกล้กล้องขึ้น แล้วถอยออกไกลๆ", "move CLOSER / FARTHER"),
    "skew": ("เอียงกระดาษ ก้ม/เงย/หมุนซ้ายขวา ~30°", "TILT the board"),
}


def board_features(corners, cols, rows, w, h):
    """จุดเด่นของท่ากระดาษ: ตำแหน่งกลาง (x, y) ขนาด และความเอียง — ไว้ตัดสินว่าท่าใหม่ต่างจากเดิมพอไหม"""
    pts = corners.reshape(-1, 2)
    quad = np.array([pts[0], pts[cols - 1], pts[-1], pts[-cols]], np.float32)  # มุมนอก 4 มุม
    cx, cy = pts.mean(axis=0)
    area = cv2.contourArea(quad)
    # ความเอียง = มุมที่มุมบนซ้ายของกระดาษต่างจาก 90° คูณ 2 (สูตรของ ROS) — กระดาษตรงหน้า = 0
    # (เอียงกระดาษ 30° มุมในภาพเปลี่ยนแค่ ~5° เลยต้องขยาย ไม่งั้นแถบนี้แทบไม่ขยับ)
    a, b = quad[1] - quad[0], quad[3] - quad[0]
    angle = np.arccos(np.clip(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-9), -1, 1))
    return {"x": cx / w, "y": cy / h, "size": float(np.sqrt(area / (w * h))),
            "skew": min(1.0, 2.0 * abs(np.pi / 2 - angle))}


def depth_at_corners(depth_msg, color_msg, corners):
    """ระยะ depth (m) ที่แต่ละมุมของตาราง — None ถ้า depth ไม่ได้จัดตรงกับภาพสีหรือวัดไม่ได้เกินครึ่ง"""
    if (depth_msg is None or depth_msg.header.frame_id != color_msg.header.frame_id
            or (depth_msg.width, depth_msg.height) != (color_msg.width, color_msg.height)
            or depth_msg.encoding not in ("16UC1", "mono16")):
        return None
    depth = np.frombuffer(depth_msg.data, np.uint16).reshape(depth_msg.height, depth_msg.step // 2)
    zs = []
    for u, v in corners.reshape(-1, 2):
        patch = depth[max(int(v) - 2, 0):int(v) + 3, max(int(u) - 2, 0):int(u) + 3]
        valid = patch[patch > 0]
        zs.append(np.median(valid) / 1000.0 if valid.size >= 5 else np.nan)
    zs = np.array(zs)
    return zs if np.isnan(zs).mean() < 0.5 else None


class CalibrateColor(Node):
    def __init__(self):
        super().__init__("calibrate_color")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        # depth ที่จัดตรงกับภาพสีแล้ว (depth_registration) — ใช้ตรวจผลกับระยะจริง ไม่มีก็แค่ข้ามการตรวจ
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("cols", 9)          # มุมด้านใน (ไม่ใช่จำนวนช่อง)
        self.declare_parameter("rows", 6)
        self.declare_parameter("square", 0.025)    # เมตร
        self.declare_parameter("samples", 30)      # ขั้นต่ำก่อนคำนวณ (ถ้าครอบคลุมครบ) — ไม่ครบก็คำนวณที่ 40
        self.declare_parameter("out", "/humanoi_ws/src/humanoi_description/config/astra_color.yaml")
        # true = เปิดหน้าต่างภาพบนจอ Pi ด้วย (ต้องมี DISPLAY + XAUTHORITY — ใช้ scripts/calibrate_camera.sh)
        self.declare_parameter("show", False)

        self.cols = int(self._param("cols"))
        self.show = bool(self._param("show"))
        self.rms = None
        self.rows = int(self._param("rows"))
        self.square = float(self._param("square"))
        self.min_samples = int(self._param("samples"))
        self.out = str(self._param("out"))
        self.objp = np.zeros((self.rows * self.cols, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2) * self.square

        self.samples = []        # [(corners, features, ระยะ depth ที่แต่ละมุม (m) หรือ None)]
        self.size = None         # (w, h)
        self.done = False
        self.img_pub = self.create_publisher(CompressedImage, "/calibration/image/compressed", 2)
        self._lock = threading.Lock()
        self._latest = None
        self._depth = None
        self._new = threading.Event()
        self._running = True
        self.create_subscription(Image, str(self._param("image_topic")), self._on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, str(self._param("depth_topic")), self._on_depth,
                                 qos_profile_sensor_data)
        self._worker = threading.Thread(target=self._loop, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f"รอภาพ — ถือตารางหมากรุก {self.cols}x{self.rows} มุม (ช่อง {self.square * 1000:.0f} mm) หน้ากล้อง "
            f"ห่าง ~0.4-1.2 m แล้วค่อยๆ ขยับ/เอียงไปทั่วภาพ · ดูใน Foxglove: /calibration/image/compressed")

    def _param(self, name):
        return self.get_parameter(name).value

    def _on_image(self, msg):
        with self._lock:
            self._latest = msg
        self._new.set()

    def _on_depth(self, msg):
        with self._lock:
            self._depth = msg

    def coverage(self):
        """แต่ละแกนครอบคลุมไปแล้วกี่ส่วนของช่วงที่ต้องการ (0-1)"""
        if not self.samples:
            return {k: 0.0 for k in COVER_SPAN}
        out = {}
        for k, span in COVER_SPAN.items():
            vals = [s[1][k] for s in self.samples]
            lo = 0.0 if k == "skew" else min(vals)   # skew นับจาก 0 (กระดาษตรงหน้า)
            out[k] = min(1.0, (max(vals) - lo) / span)
        return out

    def _is_new_pose(self, feat):
        # ระยะรวมของ 4 แกนจากทุกภาพที่เก็บแล้ว ต้องเกิน 0.2 (ค่าเดียวกับ camera_calibration ของ ROS)
        return all(sum(abs(feat[k] - s[1][k]) for k in feat) > 0.2 for s in self.samples)

    def _loop(self):
        prev = None
        still_frames = 0
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
        while self._running:
            if not self._new.wait(timeout=1.0):
                continue
            with self._lock:
                msg, self._latest = self._latest, None
                depth_msg = self._depth
                self._new.clear()
            if msg is None or msg.encoding not in ("rgb8", "bgr8"):
                continue
            rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
            rgb = rgb.reshape(msg.height, msg.width, 3)
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else rgb.copy()
            if self.done:  # คำนวณเสร็จแล้ว — แค่โชว์ภาพสดพร้อมผล จนกว่าจะกด Ctrl+C
                self._output(bgr, False, None, "REJECTED - redo (see terminal)" if np.isnan(self.rms)
                             else f"DONE rms {self.rms:.2f}px - Ctrl+C")
                continue
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            h, w = gray.shape
            self.size = (w, h)

            found, corners = cv2.findChessboardCorners(gray, (self.cols, self.rows), flags)
            status = "NO BOARD"
            if found:
                corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
                # กระดาษต้องนิ่ง: มุมขยับจากภาพก่อนหน้าเฉลี่ยไม่เกิน 0.7 พิกเซล ติดกัน 3 ภาพ
                # (ถือด้วยมือแล้วขยับนิดเดียวก็เบลอ/เบี้ยวจาก rolling shutter -> rms สูง fx เพี้ยน)
                moving = prev is None or float(np.abs(corners - prev).mean()) >= 0.7
                still_frames = 0 if moving else still_frames + 1
                feat = board_features(corners, self.cols, self.rows, w, h)
                if still_frames < 3:
                    status = "HOLD STILL"
                elif self._is_new_pose(feat):
                    self.samples.append((corners.copy(), feat, depth_at_corners(depth_msg, msg, corners)))
                    status = "CAPTURED"
                    still_frames = 0
                    self._log_progress()
                else:
                    status = "SAME POSE - move"
                prev = corners
            else:
                prev = None

            self._output(bgr, found, corners if found else None, status)

            cov = self.coverage()
            enough = len(self.samples) >= self.min_samples and min(cov.values()) >= 0.7
            if enough or len(self.samples) >= MAX_SAMPLES:
                self._calibrate()
        if self.show:
            cv2.destroyAllWindows()  # คำสั่งหน้าต่างทุกตัวต้องอยู่ thread เดียวกัน (Qt)

    def _output(self, bgr, found, corners, status):
        """ส่งภาพที่วาดผลแล้วไป Foxglove (ถ้ามีคนดู) และหน้าต่างบนจอ Pi (ถ้า show:=true)"""
        try:
            to_foxglove = self.img_pub.get_subscription_count() > 0
        except Exception:  # Ctrl+C ปิด context ไปก่อน thread นี้จะหยุด
            if self._running:
                raise
            return
        if not (to_foxglove or self.show):
            return
        img = self._annotate(bgr, found, corners, status)
        if to_foxglove:
            ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                self.img_pub.publish(CompressedImage(format="jpeg", data=jpg.tobytes()))
        if self.show:
            if not getattr(self, "_window", False):
                cv2.namedWindow("calibrate_color", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("calibrate_color", 960, 720)  # ใหญ่ขึ้น มองจากระยะถือกระดาษได้
                self._window = True
            cv2.imshow("calibrate_color", img)
            cv2.waitKey(1)

    def _log_progress(self):
        cov = self.coverage()
        weakest = min(cov, key=cov.get)
        bars = " ".join(f"{k}:{int(100 * v):3d}%" for k, v in cov.items())
        self.get_logger().info(
            f"เก็บภาพ {len(self.samples)}/{self.min_samples} · {bars} · ต่อไป: {HINTS[weakest][0]}")

    def _annotate(self, bgr, found, corners, status):
        img = bgr.copy()
        if found:
            cv2.drawChessboardCorners(img, (self.cols, self.rows), corners, True)
        cov = self.coverage()
        weakest = min(cov, key=cov.get)
        lines = [f"{status}   samples {len(self.samples)}/{self.min_samples}",
                 *(f"{k:<5}{'#' * int(20 * v):<20} {int(100 * v):3d}%" for k, v in cov.items()),
                 f"next: {HINTS[weakest][1]}"]
        cv2.rectangle(img, (0, 0), (330, 14 + 22 * len(lines)), (0, 0, 0), -1)
        color = {"CAPTURED": (0, 255, 0), "NO BOARD": (0, 0, 255)}.get(
            status, (0, 255, 0) if status.startswith("DONE")
            else (0, 0, 255) if status.startswith("REJECTED") else (0, 220, 255))
        for i, text in enumerate(lines):
            cv2.putText(img, text, (8, 24 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        color if i == 0 else (255, 255, 255), 1, cv2.LINE_AA)
        return img

    def _calibrate(self):
        self.done = True
        w, h = self.size
        samples = list(self.samples)
        # ไม่หาค่าความโค้งแบบเอียง (p1 p2) และ k3: เลนส์เว็บแคมแทบไม่มี แต่ถ้าปล่อยให้หา มันจะเลื่อนจุดกลางภาพ
        # (cx) ไปชดเชยกันจนได้ค่าผิดที่ "ดูพอดี" — รอบแรก 15 ต.ค. ได้ fx 647 cx 373 ซึ่งระยะผิดจาก depth 12.5%
        flags = cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3

        def run(subset):
            return cv2.calibrateCamera([self.objp] * len(subset), [s[0] for s in subset], (w, h),
                                       None, None, flags=flags)

        self.get_logger().info(f"คำนวณจาก {len(samples)} ภาพ ...")
        try:  # เก็บภาพที่ใช้ไว้ตรวจซ้ำภายหลังได้โดยไม่ต้องถือกระดาษใหม่
            np.savez(SAMPLES_FILE, size=np.array([w, h]), objp=self.objp,
                     corners=np.array([s[0] for s in samples]),
                     depths=np.array([s[2] if s[2] is not None else np.full(len(self.objp), np.nan) for s in samples]))
        except OSError as e:
            self.get_logger().warning(f"บันทึก {SAMPLES_FILE} ไม่ได้: {e}")
        rms, K, D, rvecs, tvecs = run(samples)
        # ตัดภาพที่คลาดมากผิดปกติ (เช่น ตอนกระดาษงอ) แล้วคำนวณใหม่หนึ่งรอบ
        errs = [cv2.norm(s[0], cv2.projectPoints(self.objp, r, t, K, D)[0], cv2.NORM_L2) / np.sqrt(len(s[0]))
                for s, r, t in zip(samples, rvecs, tvecs)]
        keep = [s for s, e in zip(samples, errs) if e <= max(3 * np.median(errs), 0.3)]
        if len(keep) < len(samples) and len(keep) >= 10:
            self.get_logger().info(f"ตัดภาพที่คลาดผิดปกติ {len(samples) - len(keep)} ภาพ แล้วคำนวณใหม่")
            samples = keep
            rms, K, D, rvecs, tvecs = run(samples)

        self.rms = rms
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        d = D.ravel()[:5]
        quality = "ดีมาก" if rms < 0.3 else "ดี" if rms < 0.6 else "พอใช้" if rms < 1.0 else "แย่ — ควรทำใหม่"
        self.get_logger().info(
            f"คำนวณได้: rms {rms:.3f} px ({quality}) · fx {fx:.1f} fy {fy:.1f} cx {cx:.1f} cy {cy:.1f} · "
            f"dist {np.round(d, 4).tolist()} · fx ต่างจากค่ากลาง {100 * (fx / DEFAULT_FX - 1):+.1f}%")

        # ตรวจกับของจริง: ระยะตารางที่คิดจากค่าเลนส์ใหม่ (รู้ขนาดช่อง) เทียบกับระยะที่เซนเซอร์ depth วัดได้
        # ค่าเลนส์ผิด -> ระยะผิดตามสัดส่วน fx — rms ต่ำอย่างเดียวพิสูจน์ไม่ได้ (รอบแรก rms ดูดีแต่ fx ผิด 13%)
        # ดู "ความชัน" ข้ามหลายระยะ ไม่ใช่อัตราส่วนตรงๆ: จุดศูนย์ของ depth ไม่ได้อยู่ที่จุดศูนย์เลนส์สี (Astra Pro
        # เหลื่อมกัน ~3 cm) อัตราส่วนเลยฟ้องผิด ~5% ทั้งที่ fx ถูก (ตรวจด้วย check_depth_scale 15 ต.ค.)
        pairs = []  # (depth, ระยะจากค่าเลนส์) ค่ากลางต่อภาพ ที่มุมเดียวกัน
        for s, r, t in zip(samples, rvecs, tvecs):
            if s[2] is not None:
                z_lens = (self.objp @ cv2.Rodrigues(r)[0].T + t.reshape(1, 3))[:, 2]
                ok = ~np.isnan(s[2])
                pairs.append((float(np.median(s[2][ok])), float(np.median(z_lens[ok]))))
        odd = abs(fx / fy - 1) > 0.03 or abs(cx / w - 0.5) > 0.1 or abs(cy / h - 0.5) > 0.1
        zd, zl = np.array(pairs).T if pairs else (np.zeros(0), np.zeros(0))
        if len(pairs) >= 8 and np.ptp(zd) >= 0.2:
            slope, offset = np.polyfit(zd, zl, 1)
            fit = np.abs(zl - (slope * zd + offset)) <= max(3 * np.median(np.abs(zl - (slope * zd + offset))), 0.005)
            slope, offset = np.polyfit(zd[fit], zl[fit], 1)  # ตัดภาพที่ depth วัดพลาดแล้ว fit ใหม่
            diff = 100 * (slope - 1)
            self.get_logger().info(
                f"ตรวจกับ depth ({fit.sum()} ภาพ ระยะ {zd.min():.2f}-{zd.max():.2f} m): สเกลต่างจาก depth {diff:+.1f}% "
                f"· ระยะจากเลนส์สีมากกว่า depth {100 * (offset + (slope - 1) * zd.mean()):+.1f} cm (จุดศูนย์เหลื่อมกัน)")
            bad = abs(diff) > 4.0
            note = f"rms {rms:.3f} px · สเกลต่างจาก depth {diff:+.1f}% ({fit.sum()} ภาพ {zd.min():.2f}-{zd.max():.2f} m)"
        elif pairs:
            # ระยะที่ถือแคบเกินจะแยกสเกลออกจากระยะเหลื่อม — ใช้อัตราส่วน (ฟ้องเกินจริงได้ ~5% จากระยะเหลื่อม)
            diff = 100 * (float(np.median(zl / zd)) - 1)
            self.get_logger().warning(
                f"ระยะที่ถือแคบเกิน ({zd.min():.2f}-{zd.max():.2f} m) ตรวจสเกลไม่ได้ — ใช้อัตราส่วนแทน: {diff:+.1f}% "
                "· ครั้งหน้าถือทั้งใกล้ ~0.5 m และไกล ~1.1 m")
            bad = abs(diff) > 8.0
            note = f"rms {rms:.3f} px · ระยะต่างจาก depth {diff:+.1f}% ({len(pairs)} ภาพ ช่วงระยะแคบ)"
        else:
            self.get_logger().warning("ไม่มีข้อมูล depth ให้ตรวจ (depth_registration ปิดอยู่?) — ข้ามการตรวจ")
            bad = odd
            note = f"rms {rms:.3f} px · ไม่ได้ตรวจกับ depth"
        if bad:
            self._write_yaml(w, h, K, d, self.out + ".rejected", note)
            self.get_logger().error(
                f"ไม่ใช้ผลนี้ — ค่าไม่สอดคล้องกับของจริง (เก็บไว้ดูที่ {self.out}.rejected) · ทำใหม่: ถือกระดาษ"
                "ให้ทั่วภาพทั้ง 9 ตำแหน่ง (โดยเฉพาะมุมภาพ) ทั้งใกล้/ไกล และเอียงหลายทิศ")
            self.rms = float("nan")
            return
        if odd:
            self.get_logger().warning("fx/fy หรือจุดกลางภาพแปลกกว่าปกติเล็กน้อย แต่ระยะตรงกับ depth จึงยังใช้ได้")
        self._write_yaml(w, h, K, d, self.out, note)
        self.get_logger().info(
            f"บันทึกแล้ว: {self.out} · ใช้งาน: colcon build --symlink-install --packages-select "
            "humanoi_description แล้วเปิดกล้องใหม่ (camera.launch.py โหลดไฟล์นี้เอง) · กด Ctrl+C ออกได้")

    def _write_yaml(self, w, h, K, d, path, note):
        def row(vals):
            return "[" + ", ".join(f"{v:.6f}" for v in vals) + "]"
        P = [K[0, 0], 0.0, K[0, 2], 0.0, 0.0, K[1, 1], K[1, 2], 0.0, 0.0, 0.0, 1.0, 0.0]
        text = (
            f"# ค่าเลนส์กล้องสี Astra Pro — สร้างโดย humanoi_vision calibrate_color "
            f"{time.strftime('%Y-%m-%d %H:%M %Z')} จาก {len(self.samples)} ภาพ\n"
            f"# {note}\n"
            f"image_width: {w}\nimage_height: {h}\ncamera_name: rgb_camera\n"
            f"camera_matrix:\n  rows: 3\n  cols: 3\n  data: {row(K.ravel())}\n"
            f"distortion_model: plumb_bob\n"
            f"distortion_coefficients:\n  rows: 1\n  cols: 5\n  data: {row(d)}\n"
            f"rectification_matrix:\n  rows: 3\n  cols: 3\n  data: {row(np.eye(3).ravel())}\n"
            f"projection_matrix:\n  rows: 3\n  cols: 4\n  data: {row(P)}\n")
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)

    def stop(self):
        self._running = False
        self._new.set()
        self._worker.join(timeout=5.0)


def main():
    rclpy.init()
    node = CalibrateColor()
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
