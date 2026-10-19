#!/usr/bin/env python3
"""ตรวจระยะด้วยตลับเมตร — depth กับค่าเลนส์กล้องสี ใครคลาด

  bash ~/Humanoi/scripts/check_depth_scale.sh      # บน Pi นอก container (มีหน้าต่างภาพ + เปิดกล้องให้เองถ้ายังไม่เปิด)

ใช้เมื่อ calibrate_color ได้ค่าเลนส์ที่ระยะค้านกับ depth: สองอย่างนี้ตรวจกันเองไม่ได้ว่าใครผิด ต้องมีของจริงมาตัดสิน
ตั้งตารางหมากรุก (แปะแผ่นแข็ง ตั้งตรง หันหน้าหากล้อง) 2 ระยะ วัดจากหน้ากล้องถึงกระดาษด้วยตลับเมตรแล้วพิมพ์ใส่
แล้วเทียบ "ระยะที่เลื่อนไป" จาก 3 แหล่ง (ใช้ผลต่าง จึงไม่ต้องรู้ว่าจุดศูนย์ของกล้องอยู่ลึกจากหน้ากล้องเท่าไร):
  ตลับเมตร = ความจริง · depth -> สเกล depth ถูกไหม · PnP ภาพสี (ขึ้นกับ fx x ขนาดช่อง) -> fx ที่ถูกควรเป็นเท่าไร
วัดเป็นระยะตั้งฉากจากกล้องถึงระนาบกระดาษ จึงไม่ขึ้นกับมุมก้มเงยของกล้อง ขอแค่เลื่อนกระดาษถอยตรงๆ ไม่หมุน
"""
import os
import re
import signal
import subprocess
import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from humanoi_vision.calibrate_color import DEFAULT_FX, depth_at_corners

CONFIG = "/humanoi_ws/src/humanoi_description/config"
K_DEFAULT = np.array([[DEFAULT_FX, 0, 319.5], [0, DEFAULT_FX, 239.5], [0, 0, 1]])
FRAMES = 30                                  # ภาพต่อ 1 ระยะ (กระดาษวางนิ่ง ใช้ค่ากลาง)
SCALES = np.arange(0.80, 1.2001, 0.0025)     # ลองคูณ fx fy ช่วงนี้ หาค่าที่ระยะเลื่อนตรงกับตลับเมตร
POSITIONS = (("ใกล้ ~60 cm", "NEAR ~60cm"),
             ("ไกล ~100 cm — เลื่อนกระดาษถอยตรงๆ ไม่หมุน", "FAR ~100cm"))


def to_bgr(msg):
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
    img = img.reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else img.copy()


def load_calib(path):
    with open(path) as f:
        c = yaml.safe_load(f)
    K = np.array(c["camera_matrix"]["data"], float).reshape(3, 3)
    return K, np.array(c["distortion_coefficients"]["data"], float)


def scaled(K, k):
    K = K.copy()
    K[0, 0] *= k
    K[1, 1] *= k
    return K


def plane_distance(objp, corners, K, D):
    """ระยะตั้งฉากจากกล้องถึงระนาบกระดาษ (m) จาก PnP + แกนตั้งฉากกระดาษ (ในเฟรมกล้อง)"""
    _, rvec, tvec = cv2.solvePnP(objp, corners, K, D)
    n = cv2.Rodrigues(rvec)[0][:, 2]
    return abs(float(n @ tvec.ravel())), n


def depth_plane_distance(corners, zs, n, K, D):
    """ระยะตั้งฉากถึงระนาบกระดาษ (m) จาก depth ที่มุมตาราง (กระดาษอยู่กลางภาพ -> แทบไม่ขึ้นกับ fx)"""
    xy = cv2.undistortPoints(corners, K, D).reshape(-1, 2)
    pts = np.column_stack([xy * zs[:, None], zs])
    return float(np.nanmedian(np.abs(pts @ n)))


def ask_cm(prompt):
    while True:
        m = re.search(r"\d+(?:\.\d+)?", input(prompt))
        if m and 20 <= float(m.group()) <= 400:
            return float(m.group()) / 100.0
        print("  พิมพ์เป็นเซนติเมตร เช่น 60.5")


class DepthScaleCheck(Node):
    def __init__(self):
        super().__init__("check_depth_scale")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")   # ต้อง depth_registration (ค่าเริ่ม)
        self.declare_parameter("cols", 8)          # มุมด้านใน
        self.declare_parameter("rows", 5)
        self.declare_parameter("square", 0.0282)   # เมตร — ขนาดที่วัดจริงบนกระดาษ
        self.declare_parameter("calib", "")        # ไฟล์ค่าเลนส์ที่จะตรวจ ("" = astra_color.yaml หรือ .rejected)
        self.declare_parameter("show", False)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.image_topic = str(p("image_topic"))
        self.cols, self.rows, self.show = int(p("cols")), int(p("rows")), bool(p("show"))
        self.objp = np.zeros((self.rows * self.cols, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2) * float(p("square"))
        calib = str(p("calib")) or next((f for f in (f"{CONFIG}/astra_color.yaml", f"{CONFIG}/astra_color.yaml.rejected")
                                         if os.path.exists(f)), "")
        if not calib:
            raise SystemExit(f"ไม่มีไฟล์ค่าเลนส์ใน {CONFIG} — ระบุเอง: -p calib:=<ไฟล์>")
        self.calib = calib
        self.K, self.D = load_calib(calib)

        self._lock = threading.Lock()
        self._color = self._depth = None
        self.status = "waiting for camera"
        self.create_subscription(Image, self.image_topic, self._on_color, qos_profile_sensor_data)
        self.create_subscription(Image, str(p("depth_topic")), self._on_depth, qos_profile_sensor_data)

    def _on_color(self, msg):
        with self._lock:
            self._color = msg

    def _on_depth(self, msg):
        with self._lock:
            self._depth = msg

    def latest(self):
        with self._lock:
            return self._color, self._depth

    def find_board(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray, (self.cols, self.rows),
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
        if not found:
            return None
        return cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))

    def grab(self, n, timeout):
        """เก็บ n ภาพที่เจอตาราง + มี depth ที่มุม -> [(corners, z ที่แต่ละมุม)]"""
        out, last = [], None
        t_end = time.monotonic() + timeout
        while len(out) < n and time.monotonic() < t_end:
            color, depth = self.latest()
            if color is None or color is last:
                time.sleep(0.01)
                continue
            last = color
            corners = self.find_board(to_bgr(color))
            zs = None if corners is None else depth_at_corners(depth, color, corners)
            if zs is not None:
                out.append((corners, zs))
        return out

    def camera_running(self):
        return (None not in self.latest() or self.count_publishers(self.image_topic) > 0
                or ("camera", "/camera") in self.get_node_names_and_namespaces())

    def ensure_camera(self):
        """ไม่มีใครเปิดกล้องอยู่ -> เปิดเอง (คืน process ไว้ปิดตอนจบ)
        เปิดซ้อนตัวที่มีอยู่ไม่ได้ (แย่งอุปกรณ์กัน) จึงรอ discovery ให้นานพอก่อนตัดสินว่าไม่มี"""
        t_end = time.monotonic() + 6
        while time.monotonic() < t_end:
            if self.camera_running():
                return None
            time.sleep(0.2)
        print("ยังไม่มีกล้อง — เปิดกล้องให้ (log: /tmp/check_depth_camera.log) ...")
        cam = subprocess.Popen(
            ["ros2", "launch", "humanoi_description", "camera.launch.py",
             "foxglove:=false", "enable_ir:=false", "enable_point_cloud:=false"],
            stdout=open("/tmp/check_depth_camera.log", "w"), stderr=subprocess.STDOUT, start_new_session=True)
        t_end = time.monotonic() + 30
        while None in self.latest() and time.monotonic() < t_end:
            time.sleep(0.2)
        if None in self.latest():
            stop_camera(cam)
            raise SystemExit("เปิดกล้องไม่ขึ้น — ดู /tmp/check_depth_camera.log")
        return cam

    def display_loop(self, running):
        """หน้าต่างภาพสด: เห็นตาราง + ระยะจาก depth คร่าวๆ (คำสั่งหน้าต่างทุกตัวอยู่ thread นี้ — Qt)"""
        last = None
        while running.is_set():
            color, depth = self.latest()
            if color is None or color is last:
                time.sleep(0.01)
                continue
            last = color
            bgr = to_bgr(color)
            corners = self.find_board(bgr)
            text, ok = "NO BOARD - put it in the middle", False
            if corners is not None:
                cv2.drawChessboardCorners(bgr, (self.cols, self.rows), corners, True)
                zs = depth_at_corners(depth, color, corners)
                text, ok = ("BOARD OK (no depth - too close?)", False) if zs is None else \
                    (f"BOARD OK  depth {100 * np.nanmedian(zs):.1f} cm", True)
            cv2.rectangle(bgr, (0, 0), (640, 64), (0, 0, 0), -1)
            cv2.putText(bgr, self.status, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(bgr, text, (8, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if ok else (0, 0, 255), 2)
            cv2.imshow("check_depth_scale", cv2.resize(bgr, (960, 720)))
            cv2.waitKey(1)
        cv2.destroyAllWindows()

    def measure(self, samples, K, D):
        """(ระยะจาก PnP, ระยะจาก depth) ค่ากลางของทุกภาพ"""
        d_pnp, d_depth = [], []
        for corners, zs in samples:
            d, n = plane_distance(self.objp, corners, K, D)
            d_pnp.append(d)
            d_depth.append(depth_plane_distance(corners, zs, n, K, D))
        return float(np.median(d_pnp)), float(np.median(d_depth))

    def report(self, tapes, grabs):
        (t1, t2), (s1, s2) = tapes, grabs
        dt = t2 - t1
        # fx ที่ทำให้ระยะเลื่อนจากภาพสีตรงกับตลับเมตร (ลองคูณ fx fy ทีละนิด) — ไม่ใช้ depth เลย
        errs = [abs(np.median([plane_distance(self.objp, c, scaled(self.K, k), self.D)[0] for c, _ in s2])
                    - np.median([plane_distance(self.objp, c, scaled(self.K, k), self.D)[0] for c, _ in s1]) - dt)
                for k in SCALES]
        k = float(SCALES[int(np.argmin(errs))])
        p1, _ = self.measure(s1, self.K, self.D)
        p2, _ = self.measure(s2, self.K, self.D)
        q1, _ = self.measure(s1, K_DEFAULT, np.zeros(5))
        q2, _ = self.measure(s2, K_DEFAULT, np.zeros(5))
        # depth -> ระยะตั้งฉาก ต้องแปลงพิกเซลเป็นพิกัดด้านข้าง ใช้ fx ที่ถูก (จากตลับเมตร) จะได้ไม่พาความคลาดของ fx มาปน
        _, z1 = self.measure(s1, scaled(self.K, k), self.D)
        _, z2 = self.measure(s2, scaled(self.K, k), self.D)
        pct = lambda d: f"{100 * (d / dt - 1):+.1f}%"  # noqa: E731
        fx = self.K[0, 0]

        print("\n================ ผล ================")
        print(f"ไฟล์ค่าเลนส์: {self.calib} (fx {fx:.1f})")
        print("ระยะตั้งฉากถึงกระดาษ (cm)   ตลับเมตร   depth   ภาพสี(ไฟล์)   ภาพสี(ค่ากลาง 570)")
        for i, (t, z, p, q) in enumerate(((t1, z1, p1, q1), (t2, z2, p2, q2)), 1):
            print(f"  ระยะ {i}                     {100 * t:6.1f}  {100 * z:6.1f}     {100 * p:6.1f}       {100 * q:6.1f}")
        print(f"  เลื่อนไป                    {100 * dt:6.1f}  {100 * (z2 - z1):6.1f}     {100 * (p2 - p1):6.1f}       "
              f"{100 * (q2 - q1):6.1f}")
        print(f"  คลาดจากตลับเมตร                    {pct(z2 - z1):>6}     {pct(p2 - p1):>6}       {pct(q2 - q1):>6}")
        print(f"  (depth - ตลับเมตร = {100 * (z1 - t1):+.1f} / {100 * (z2 - t2):+.1f} cm — ถ้า depth แม่น สองค่านี้เท่ากัน "
              "= ระยะจากหน้ากล้องถึงจุดศูนย์ของ depth)")

        depth_err = (z2 - z1) / dt - 1
        print("\nสรุป:")
        # ไม่ใช้ตลับเมตรเลย: ภาพสีกับ depth เห็นกระดาษเลื่อนไปเท่ากันไหม (ตัดระยะเหลื่อมของจุดศูนย์ทิ้ง)
        _, zf1 = self.measure(s1, self.K, self.D)
        _, zf2 = self.measure(s2, self.K, self.D)
        print(f"- ภาพสี (ไฟล์) เทียบ depth กันเอง ไม่ใช้ตลับเมตร: เลื่อนไป {100 * (p2 - p1):.1f} vs {100 * (zf2 - zf1):.1f} cm "
              f"({100 * ((p2 - p1) / (zf2 - zf1) - 1):+.1f}%) · ระยะเหลื่อม ภาพสี-depth "
              f"{100 * (p1 - zf1):+.1f} / {100 * (p2 - zf2):+.1f} cm")
        if abs(depth_err) <= 0.02:
            print(f"- depth แม่น (คลาด {100 * depth_err:+.1f}%) -> ใช้ depth เป็นตัวตัดสินค่าเลนส์ได้")
        else:
            print(f"- depth วัดระยะ{'ยาว' if depth_err > 0 else 'สั้น'}กว่าจริง {100 * abs(depth_err):.1f}% "
                  f"-> ต้องแก้สเกล depth (คูณ {1 / (1 + depth_err):.3f})")
        if k in (SCALES[0], SCALES[-1]):
            print("- หา fx ที่เข้ากับตลับเมตรไม่เจอในช่วง ±20% — ตรวจขนาดช่อง (square) กับตัวเลขที่พิมพ์")
        else:
            print(f"- fx ที่ถูก ≈ {fx * k:.0f} (ไฟล์นี้ {fx:.0f} คลาด {100 * (1 / k - 1):+.1f}% · "
                  f"ค่ากลาง 570 คลาด {100 * (DEFAULT_FX / (fx * k) - 1):+.1f}%)")
        if dt < 0.3:
            print("- (ระยะเลื่อนน้อยกว่า 30 cm ผลหยาบ — เลื่อนให้ห่างกันมากกว่านี้จะแม่นกว่า)")


def stop_camera(cam):
    if cam is None or cam.poll() is not None:
        return
    os.killpg(cam.pid, signal.SIGINT)
    try:
        cam.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(cam.pid, signal.SIGKILL)


def main():
    rclpy.init()
    node = DepthScaleCheck()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin)
    spin.start()
    running = threading.Event()
    running.set()
    cam, display = None, None
    try:
        cam = node.ensure_camera()
        if node.show:
            display = threading.Thread(target=node.display_loop, args=(running,), daemon=True)
            display.start()
        print(f"ไฟล์ค่าเลนส์ที่ตรวจ: {node.calib}")
        print("ตั้งกระดาษ (แปะแผ่นแข็ง) ให้ตั้งตรง หันหน้าหากล้อง อยู่กลางภาพ แล้ววัดด้วยตลับเมตร"
              "จากหน้ากล้องถึงผิวกระดาษ ในแนวตั้งฉากกับกระดาษ")
        tapes, grabs = [], []
        for i, (hint, label) in enumerate(POSITIONS, 1):
            while True:
                node.status = f"{i}/2: {label} - type distance in terminal"
                tape = ask_cm(f"\n[{i}/2] ตั้งกระดาษ{hint} พิมพ์ระยะที่วัดได้ (cm) แล้วกด Enter: ")
                node.status = f"{i}/2: measuring - don't move"
                samples = node.grab(FRAMES, timeout=20)
                if len(samples) < 5:
                    print(f"  ได้แค่ {len(samples)} ภาพ — ไม่เจอตารางครบทุกมุม หรือ depth วัดไม่ได้ (ใกล้เกิน ~60 cm?) "
                          "จัดใหม่แล้ววัดอีกที")
                    continue
                # กันพิมพ์ก่อนวางกระดาษเสร็จ / วัดผิดจุด: depth กับตลับเมตรต่างกันได้แค่ไม่กี่ cm (จุดศูนย์เหลื่อม)
                seen = node.measure(samples, node.K, node.D)[1]
                if abs(seen - tape) <= 0.10 or input(
                        f"  กล้องเห็นกระดาษห่าง ~{100 * seen:.0f} cm แต่พิมพ์ {100 * tape:.1f} cm — ต่างกันเกิน 10 cm "
                        "(วางกระดาษก่อนกด Enter หรือยัง?) พิมพ์ y = ใช้ค่านี้, Enter = วัดใหม่: ").strip().lower() == "y":
                    break
            print(f"  เก็บ {len(samples)} ภาพแล้ว")
            tapes.append(tape)
            grabs.append(samples)
        node.status = "done - see terminal"
        node.report(tapes, grabs)
    except KeyboardInterrupt:
        pass
    finally:
        running.clear()
        if display is not None:
            display.join(timeout=2)
        stop_camera(cam)
        executor.shutdown()
        spin.join(timeout=5)  # ต้องให้ thread ของ ROS จบก่อน shutdown ไม่งั้น C++ abort ตอนออก
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
