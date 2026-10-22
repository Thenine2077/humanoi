#!/usr/bin/env python3
"""hand-eye calibration: หาว่ากล้อง Astra Pro (ติดอยู่กับที่บนเสา) อยู่ตรงไหนเทียบกับฐานแขน (eye-to-hand)

  bash ~/Humanoi/scripts/hand_eye_calibrate.sh     # บน Pi นอก container — เปิด arm start + arm moveit ไว้ก่อน

แปะป้าย ArUco (4x4_50 id 0 ขนาด 40 mm มีขอบขาว) บนมือจับ แล้วพาแขนไป ~15 ท่าทั่วพื้นที่ทำงานด้วย MoveIt
หยุดค้างแต่ละท่า ~2 วินาที โปรแกรมเก็บเองเมื่อ: แขนนิ่ง + เห็นป้าย + ท่าต่างจากเดิมพอ
แต่ละท่าได้คู่ (ป้ายในเฟรมกล้อง, ท่าของ link_6 ในเฟรมฐานแขน จาก TF)
ตำแหน่งป้าย (measure): "pnp" (ค่าเริ่ม) = จากขนาดป้ายในภาพ ใช้ได้ทุกระยะ — แขนซ้ายเอื้อมห่างกล้องได้แค่ ~52 cm
ซึ่งใกล้กว่าระยะต่ำสุดของ depth (~55-60 cm) · "depth" = รังสีผ่านกลางป้ายตัดระนาบ depth (เหมือน yolo_node)
-> คำนวณตำแหน่ง/มุมกล้อง พร้อมตำแหน่งป้ายบนมือ (ไม่ต้องรู้ว่าแปะตรงไหน) -> เขียน yaml ที่ yolo.launch.py
ส่งเป็น TF base_link -> camera_link ทำให้แปลง /yolo/detections_3d เป็นพิกัดแขนได้
ผลใช้ได้ตราบที่กล้องกับฐานแขนไม่ขยับเทียบกัน — กล้องโดนกระแทกต้องทำใหม่
"""
import collections
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformException, TransformListener

from humanoi_vision import hand_eye

MIN_ANGLE_DEG = 30.0   # ข้อมือต้องหมุนต่างกันอย่างน้อยเท่านี้ ไม่งั้นหาตำแหน่งป้ายบนมือไม่ออก
MAX_RMS_MM = 10.0      # คลาดเฉลี่ยเกินนี้ไม่ใช้ผล


def tf_to_mat(tf):
    q, p = tf.transform.rotation, tf.transform.translation
    return hand_eye.quat_to_mat(q.x, q.y, q.z, q.w), np.array([p.x, p.y, p.z])


def fit_plane(X, tol=0.004, iters=80, rng=np.random.default_rng(0)):
    """ระนาบจากจุด 3D แบบทนจุดเพี้ยน -> (n, d) ที่ n·X = d หรือ None"""
    best = None
    for _ in range(iters):
        a, b, c = X[rng.choice(len(X), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        inl = np.abs((X - a) @ n) < tol
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None or best.sum() < max(30, 0.5 * len(X)):
        return None
    c = X[best].mean(0)
    n = np.linalg.svd(X[best] - c)[2][2]
    return n, float(n @ c)


class HandEyeCalibrate(Node):
    def __init__(self):
        super().__init__("hand_eye_calibrate")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("base_frame", "left_base_link")  # ฐานแขนซ้ายในโมเดลสองแขน ar_dual (แขนเดี่ยวเก่า = base_link)
        self.declare_parameter("tool_frame", "left_link_6")     # ข้อสุดท้ายของแขน ป้ายแปะอยู่บนมือที่ติดกับข้อนี้
        self.declare_parameter("mount_frame", "camera_link")    # รากของ TF ที่ไดรเวอร์กล้องส่ง -> ต่อเข้ากับฐานแขน
        self.declare_parameter("marker_id", 0)                  # -1 = id ไหนก็ได้
        self.declare_parameter("marker_size", 0.040)            # เมตร ด้านของสี่เหลี่ยมดำ
        self.declare_parameter("depth_offset", 0.03)            # ต้องเท่ากับของ yolo_node (ดู yolo.launch.py)
        self.declare_parameter("measure", "pnp")                # pnp | depth (ดูหัวไฟล์)
        self.declare_parameter("samples", 15)
        self.declare_parameter("out", "/humanoi_ws/src/humanoi_description/config/camera_to_left_arm.yaml")
        self.declare_parameter("show", False)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.base, self.tool, self.mount = str(p("base_frame")), str(p("tool_frame")), str(p("mount_frame"))
        self.marker_id, self.size = int(p("marker_id")), float(p("marker_size"))
        self.depth_offset, self.target = float(p("depth_offset")), int(p("samples"))
        self.out, self.show = str(p("out")), bool(p("show"))
        self.mode = str(p("measure"))
        if self.mode not in ("pnp", "depth"):
            raise ValueError(f"measure ต้องเป็น pnp หรือ depth (ได้ {self.mode})")

        if hasattr(cv2.aruco, "ArucoDetector"):  # OpenCV >= 4.7
            self._detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
            self._detect = lambda g: self._detector.detectMarkers(g)
        else:                                    # OpenCV 4.6 ใน container
            dic = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            prm = cv2.aruco.DetectorParameters_create()
            prm.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self._detect = lambda g: cv2.aruco.detectMarkers(g, dic, parameters=prm)
        s = self.size / 2  # ลำดับมุมตามที่ SOLVEPNP_IPPE_SQUARE ต้องการ (ตรงกับลำดับมุมของ ArUco)
        self._objp = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float32)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._lock = threading.Lock()
        self._color = self._depth = None
        self._K = self._D = None
        self._joints = collections.deque(maxlen=400)   # (เวลา, ชื่อข้อต่อ, มุม)
        self._new = threading.Event()
        self._running = True
        self.samples = []      # dict: p (ป้ายในเฟรมกล้อง), R t (link_6 ในเฟรมฐาน), joints, pnp
        self.result = None     # ข้อความบนจอหลังคำนวณ
        self.create_subscription(Image, str(p("image_topic")), self._on_color, qos_profile_sensor_data)
        self.create_subscription(Image, str(p("depth_topic")), self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, str(p("camera_info_topic")), self._on_info, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 50)
        self._worker = threading.Thread(target=self._loop, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f"แปะป้าย ArUco id {self.marker_id} ({self.size * 1000:.0f} mm) บนมือ แล้วพาแขนไป {self.target} ท่า "
            f"ทั่วพื้นที่ทำงาน (หมุนข้อมือหลายมุม) หยุดค้างท่าละ ~2 วินาที · วัดตำแหน่งป้ายแบบ {self.mode}"
            + (" (ป้ายต้องห่างกล้อง 0.6-1.0 m)" if self.mode == "depth" else ""))

    # ---------- รับข้อมูล ----------
    def _on_color(self, msg):
        with self._lock:
            self._color = msg
        self._new.set()

    def _on_depth(self, msg):
        with self._lock:
            self._depth = msg

    def _on_info(self, msg):
        self._K = np.array(msg.k, np.float64).reshape(3, 3)
        self._D = np.array(msg.d, np.float64)

    def _on_joints(self, msg):
        with self._lock:
            self._joints.append((time.monotonic(), list(msg.name), np.array(msg.position, float)))

    def arm_still(self, window=1.0, tol=0.002):
        """แขนนิ่งมาอย่างน้อย window วินาที (ทุกข้อขยับไม่เกิน tol rad) -> (ชื่อข้อต่อ, มุม) หรือ None"""
        with self._lock:
            js = list(self._joints)
        now = time.monotonic()
        recent = [j for j in js if now - j[0] <= window]
        if not recent or now - js[-1][0] > 0.5 or js[0][0] > now - window or len(recent) < 3:
            return None
        pos = np.array([j[2] for j in recent if len(j[2]) == len(recent[-1][2])])
        if np.ptp(pos, axis=0).max() > tol:
            return None
        return recent[-1][1], recent[-1][2]

    # ---------- วัดตำแหน่งป้าย ----------
    def measure(self, corners, color_msg, depth_msg):
        """ป้ายในเฟรมกล้อง: รังสีผ่านจุดกลางป้าย ตัดกับระนาบที่ depth วัดได้บนแผ่นป้าย (ส่วนสีดำ depth มองไม่เห็น)
        -> (p_depth หรือ None, p_pnp, ข้อความสาเหตุถ้าวัดไม่ได้)"""
        c = corners.reshape(4, 2).astype(np.float64)
        # จุดกลางป้าย = จุดตัดเส้นทแยง (ถูกกว่าค่าเฉลี่ยมุมเมื่อป้ายเอียง)
        d1, d2 = c[2] - c[0], c[3] - c[1]
        s = np.linalg.solve(np.column_stack([d1, -d2]), c[1] - c[0])[0]
        center = c[0] + s * d1
        _, rvec, tvec = cv2.solvePnP(self._objp, c.astype(np.float32), self._K, self._D,
                                     flags=cv2.SOLVEPNP_IPPE_SQUARE)
        p_pnp = tvec.ravel()
        if (depth_msg is None or depth_msg.header.frame_id != color_msg.header.frame_id
                or (depth_msg.width, depth_msg.height) != (color_msg.width, color_msg.height)):
            return None, p_pnp, "depth not registered"
        depth = np.frombuffer(depth_msg.data, np.uint16).reshape(depth_msg.height, depth_msg.step // 2)
        mask = np.zeros(depth.shape, np.uint8)  # ป้าย + ขอบขาวรอบๆ (ขยาย 1.1 เท่าจากจุดกลาง ให้ยังอยู่บนแผ่นป้าย)
        cv2.fillConvexPoly(mask, np.round(center + 1.1 * (c - center)).astype(np.int32), 1)
        vs, us = np.nonzero((mask > 0) & (depth[:, :depth_msg.width] > 0))
        if len(us) < 40:
            return None, p_pnp, "no depth (too close < 0.6 m?)"
        xy = cv2.undistortPoints(np.column_stack([us, vs]).astype(np.float64).reshape(-1, 1, 2),
                                 self._K, self._D).reshape(-1, 2)
        z = depth[vs, us] / 1000.0 + self.depth_offset
        plane = fit_plane(np.column_stack([xy * z[:, None], z]))
        if plane is None:
            return None, p_pnp, "depth not flat"
        n, dist = plane
        ray = np.append(cv2.undistortPoints(center.reshape(1, 1, 2), self._K, self._D).ravel(), 1.0)
        return ray * (dist / (n @ ray)), p_pnp, ""

    def _is_new_pose(self, R, t):
        for s in self.samples:
            ang = np.degrees(np.arccos(np.clip((np.trace(s["R"].T @ R) - 1) / 2, -1, 1)))
            if np.linalg.norm(s["t"] - t) < 0.03 and ang < 15:
                return False
        return True

    # ---------- วนหลัก ----------
    def _loop(self):
        prev = None
        last_status, last_log = None, 0.0
        while self._running:
            if not self._new.wait(timeout=1.0):
                continue
            with self._lock:
                msg, depth_msg = self._color, self._depth
                self._new.clear()
            if msg is None or self._K is None:
                continue
            img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
            img = img.reshape(msg.height, msg.width, 3)
            bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else img.copy()
            corners, ids, _ = self._detect(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
            found = None
            if ids is not None:
                for cr, i in zip(corners, ids.ravel()):
                    if self.marker_id < 0 or i == self.marker_id:
                        found = cr
            status, color, info = "NO MARKER", (0, 0, 255), ""
            if self.result is not None:
                status, color = self.result
            elif found is not None:
                cv2.polylines(bgr, [np.round(found).astype(np.int32)], True, (0, 255, 0), 2)
                p_depth, p_pnp, why = self.measure(found, msg, depth_msg)
                p = p_pnp if self.mode == "pnp" else p_depth
                info = f"dist {np.linalg.norm(p_pnp):.2f} m"
                moving = prev is None or np.abs(found - prev).mean() > 0.5
                still = self.arm_still()
                if p is None:
                    status = why.upper()
                elif still is None or moving:
                    status, color = "ARM MOVING - hold still", (0, 200, 255)
                else:
                    try:
                        R, t = tf_to_mat(self.tf_buffer.lookup_transform(self.base, self.tool, Time()))
                    except TransformException as e:
                        status = "NO ARM TF (arm start?)"
                        self.get_logger().warning(f"ไม่มี TF {self.base} -> {self.tool}: {e}",
                                                  throttle_duration_sec=5.0)
                    else:
                        if not self._is_new_pose(R, t):
                            status, color = "SAME POSE - move the arm", (0, 200, 255)
                        else:
                            self.samples.append({"p": p, "p_pnp": p_pnp, "p_depth": p_depth, "R": R, "t": t,
                                                 "names": still[0], "joints": still[1]})
                            status, color = "CAPTURED", (0, 255, 0)
                            cmp = ("ไม่มี depth (ใกล้เกิน)" if p_depth is None else
                                   f"depth ต่างจาก PnP {1000 * np.linalg.norm(p_depth - p_pnp):.0f} mm")
                            self.get_logger().info(
                                f"เก็บท่า {len(self.samples)}/{self.target}: ป้ายห่าง {np.linalg.norm(p):.3f} m "
                                f"({cmp}) · link_6 ({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f}) m")
                            if len(self.samples) >= self.target:
                                self._calibrate()
                prev = found
            else:
                prev = None
            # บอกสถานะใน terminal ด้วยเมื่อเปลี่ยน (ไม่ถี่เกิน 2 วินาที) — ดูได้แม้ไม่มีหน้าต่าง
            if status != last_status and time.monotonic() - last_log > 2.0 and status != "CAPTURED":
                self.get_logger().info(f"สถานะ: {status} {info}")
                last_status, last_log = status, time.monotonic()
            if self.show:
                self._draw(bgr, status, color, info)
        if self.show:
            cv2.destroyAllWindows()

    def _draw(self, bgr, status, color, info):
        lines = [f"{status}   {len(self.samples)}/{self.target}  {info}"]
        if self.samples and self.result is None:
            sp, ang = hand_eye.spread([s["R"] for s in self.samples], [s["t"] for s in self.samples])
            lines.append(f"spread {100 * sp:.0f} cm (want >15)  wrist {ang:.0f} deg (want >{MIN_ANGLE_DEG:.0f})")
        w = max(cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)[0][0] for t in lines) + 12
        cv2.rectangle(bgr, (0, 0), (w, 24 * len(lines) + 8), (0, 0, 0), -1)
        for i, t in enumerate(lines):
            cv2.putText(bgr, t, (6, 22 + 24 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        color if i == 0 else (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow("Hand-eye calibration (Ctrl+C in terminal to quit)", cv2.resize(bgr, (960, 720)))
        cv2.waitKey(1)

    # ---------- คำนวณ + บันทึก ----------
    def _calibrate(self):
        P = np.array([s["p"] for s in self.samples])
        Rs = np.array([s["R"] for s in self.samples])
        ts = np.array([s["t"] for s in self.samples])
        R, t, m, res = hand_eye.solve(P, Rs, ts)
        keep = res <= max(3 * np.median(res), 0.01)   # ตัดท่าที่คลาดผิดปกติ (เช่น depth พลาด) แล้วคำนวณใหม่
        if not keep.all() and keep.sum() >= 8:
            self.get_logger().info(f"ตัดท่าที่คลาดผิดปกติ {int((~keep).sum())} ท่า แล้วคำนวณใหม่")
            R, t, m, res = hand_eye.solve(P[keep], Rs[keep], ts[keep])
            self.samples = [s for s, k in zip(self.samples, keep) if k]
        rms = 1000 * float(np.sqrt(np.mean(res ** 2)))
        sp, ang = hand_eye.spread(Rs, ts)
        diffs = [np.linalg.norm(s["p_depth"] - s["p_pnp"]) for s in self.samples if s["p_depth"] is not None]
        pnp_mm = 1000 * float(np.median(diffs)) if diffs else float("nan")
        self.get_logger().info(
            f"คำนวณได้: คลาดเฉลี่ย {rms:.1f} mm (มากสุด {1000 * res.max():.1f}) จาก {len(self.samples)} ท่า · "
            f"ป้ายอยู่ห่าง link_6 {100 * np.linalg.norm(m):.1f} cm · ท่ากระจาย {100 * sp:.0f} cm ข้อมือหมุน {ang:.0f}° · "
            f"depth vs PnP ต่างกลางๆ {pnp_mm:.0f} mm ({len(diffs)} ท่าที่ depth วัดได้)")
        try:  # ไดรเวอร์กล้องส่ง camera_link -> camera_color_optical_frame เอง -> หาท่าของ camera_link ในฐานแขน
            R_lo, t_lo = tf_to_mat(self.tf_buffer.lookup_transform(self.mount, self._optical_frame(), Time()))
        except TransformException as e:
            self.get_logger().error(f"ไม่มี TF {self.mount} -> optical frame ({e}) — บันทึกไม่ได้")
            self.result = ("NO CAMERA TF - see terminal", (0, 0, 255))
            return
        R_bl = R @ R_lo.T
        t_bl = t - R_bl @ t_lo
        bad = []
        if rms > MAX_RMS_MM:
            bad.append(f"คลาดเฉลี่ย {rms:.1f} mm เกิน {MAX_RMS_MM:.0f}")
        if ang < MIN_ANGLE_DEG:
            bad.append(f"ข้อมือหมุนต่างกันแค่ {ang:.0f}° (ต้อง > {MIN_ANGLE_DEG:.0f}°)")
        if np.linalg.norm(m) > 0.3:
            bad.append(f"ป้ายห่าง link_6 ถึง {100 * np.linalg.norm(m):.0f} cm — ผิดปกติ")
        path = self.out + (".rejected" if bad else "")
        self._write_yaml(path, R_bl, t_bl, m, rms)
        # ข้อมูลดิบไว้วิเคราะห์ทีหลัง (ผลไม่ผ่านจะได้หาสาเหตุได้ ไม่ต้องเดา) + ท่าที่คลาดมากสุด
        np.savez(path + ".samples.npz", P=np.array([s["p"] for s in self.samples]),
                 Rs=np.array([s["R"] for s in self.samples]), ts=np.array([s["t"] for s in self.samples]),
                 joints=np.array([s["joints"] for s in self.samples]), names=np.array(self.samples[0]["names"]),
                 res=res, size=self.size)
        worst = np.argsort(res)[::-1][:5]
        self.get_logger().info("ท่าที่คลาดมากสุด: " + ", ".join(f"#{i + 1} {1000 * res[i]:.0f} mm" for i in worst)
                               + f" · ข้อมูลดิบ {path}.samples.npz")
        if bad:
            self.get_logger().error(f"ไม่ใช้ผลนี้ ({' · '.join(bad)}) — เก็บไว้ดูที่ {path} · ทำใหม่: ท่าให้กระจาย"
                                    "ทั้งซ้าย/ขวา/ใกล้/ไกล/สูง/ต่ำ และหมุนข้อมือหลายมุม")
            self.result = ("REJECTED - see terminal", (0, 0, 255))
        else:
            self.get_logger().info(f"บันทึกแล้ว: {path} · ใช้งาน: colcon build --symlink-install --packages-select "
                                   "humanoi_description แล้วเปิด yolo.launch.py ใหม่ · กด Ctrl+C ออกได้")
            self.result = (f"DONE rms {rms:.1f} mm - Ctrl+C", (0, 255, 0))

    def _optical_frame(self):
        with self._lock:
            return self._color.header.frame_id

    def _write_yaml(self, path, R, t, m, rms):
        q = hand_eye.mat_to_quat(R)
        names = self.samples[0]["names"]
        poses = "\n".join("  - [" + ", ".join(f"{v:.5f}" for v in s["joints"]) + "]" for s in self.samples)
        text = (
            f"# ตำแหน่งกล้อง Astra Pro เทียบกับฐานแขนซ้าย — humanoi_vision hand_eye_calibrate "
            f"{time.strftime('%Y-%m-%d %H:%M %Z')} จาก {len(self.samples)} ท่า\n"
            f"# คลาดเฉลี่ย {rms:.1f} mm · ป้าย ArUco {self.size * 1000:.0f} mm · วัดป้ายแบบ {self.mode}"
            f" · depth_offset {self.depth_offset:.3f} m\n"
            f"# yolo.launch.py ส่งเป็น TF {self.base} -> {self.mount} (ไดรเวอร์กล้องต่อจาก {self.mount} ไปเอง)\n"
            f"parent_frame: {self.base}\nchild_frame: {self.mount}\n"
            f"x: {t[0]:.5f}\ny: {t[1]:.5f}\nz: {t[2]:.5f}\n"
            f"qx: {q[0]:.6f}\nqy: {q[1]:.6f}\nqz: {q[2]:.6f}\nqw: {q[3]:.6f}\n"
            f"rms_mm: {rms:.2f}\n"
            f"marker_in_{self.tool}: [{m[0]:.4f}, {m[1]:.4f}, {m[2]:.4f}]\n"
            f"# ท่าที่ใช้ (มุมข้อต่อ rad) — ไว้ให้แขนเดินซ้ำเองตอน calibrate ใหม่\n"
            f"joint_names: [{', '.join(names)}]\nposes:\n{poses}\n")
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)

    def stop(self):
        self._running = False
        self._new.set()
        try:
            self._worker.join(timeout=5.0)
        except KeyboardInterrupt:  # Ctrl+C ซ้ำระหว่างรอปิด — ไม่ต้องโชว์ traceback
            pass


def main():
    rclpy.init()
    node = HandEyeCalibrate()
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
