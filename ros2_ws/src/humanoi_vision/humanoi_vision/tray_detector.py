#!/usr/bin/env python3
"""tray_detector — หาตำแหน่งกล่อง (ถาด) จากป้าย ArUco ที่แปะกลางพื้นกล่อง ด้วยภาพสีอย่างเดียว (PnP)

  ros2 launch humanoi_vision tray.launch.py            # กล้อง + TF ฐานแขน -> กล้อง (ผล hand-eye) + โหนดนี้
  Foxglove: Image panel -> /tray/image/compressed (กรอบป้าย + แกน + ระยะ) · 3D panel -> TF tray_marker

ป้าย: docs/calibration/aruco_tray_id1_120mm.pdf (4x4_50 id 1 ด้านดำ 120 mm) — พิมพ์ขนาดจริงแล้ววัด
ไม่ใช้ depth: กล่องอยู่ใกล้กล้องกว่าระยะต่ำสุดของ Astra (~0.6 m) แต่ PnP ใช้ได้ทุกระยะ

ต่อเฟรม: หาป้าย -> solvePnP -> เก็บไว้ช่วง window วินาที -> ตัดเฟรมกระโดด -> เฉลี่ย
นิ่ง (เฟรมพอ + แกว่งไม่เกิน max_spread_mm + ป้ายไม่เอียงหนีกล้องเกิน) = ส่งตำแหน่ง

หัวข้อ
  /tray/pose      (pub, PoseStamped)  ป้ายในเฟรมกล้อง (camera_color_optical_frame) — เฉพาะตอนนิ่ง
  /tray/pose_arm  (pub, PoseStamped)  ป้ายในเฟรมฐานแขน (target_frame) — ต้องมี TF จากผล hand-eye
  /tray/status    (pub, String)       ok | no_marker | unstable | tilted | no_camera_info
  /tray/image/compressed (pub, CompressedImage)  ภาพไว้ตรวจ — วาดเฉพาะตอนมีคนดู
  TF: <เฟรมกล้อง> -> tray_marker (ตอนนิ่ง)
"""
import collections
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from humanoi_vision import hand_eye, tray


def pose_msg(frame, stamp, R, t):
    msg = PoseStamped()
    msg.header.frame_id, msg.header.stamp = frame, stamp
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in t)
    q = hand_eye.mat_to_quat(R)
    o = msg.pose.orientation
    o.x, o.y, o.z, o.w = (float(v) for v in q)
    return msg


class TrayDetector(Node):
    def __init__(self):
        super().__init__("tray_detector")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("marker_id", 1)
        self.declare_parameter("marker_size", 0.120)       # เมตร ด้านสี่เหลี่ยมดำที่วัดจริงหลังพิมพ์
        self.declare_parameter("target_frame", "left_base_link")
        self.declare_parameter("window", 0.6)              # วินาที เฉลี่ยเฟรมช่วงนี้
        self.declare_parameter("min_frames", 5)
        self.declare_parameter("max_spread_mm", 4.0)       # แกว่งเกินนี้ = ยังไม่นิ่ง (กล่องขยับ / มองเฉียง)
        self.declare_parameter("max_tilt_deg", 65.0)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.marker_id, self.size = int(p("marker_id")), float(p("marker_size"))
        self.target = str(p("target_frame"))
        self.window, self.min_frames = float(p("window")), int(p("min_frames"))
        self.max_spread, self.max_tilt = float(p("max_spread_mm")), float(p("max_tilt_deg"))

        if hasattr(cv2.aruco, "ArucoDetector"):  # OpenCV >= 4.7
            det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
            self._detect = det.detectMarkers
        else:                                    # OpenCV 4.6 ใน container
            dic = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            prm = cv2.aruco.DetectorParameters_create()
            prm.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self._detect = lambda g: cv2.aruco.detectMarkers(g, dic, parameters=prm)

        self._K = self._D = None
        self._lock = threading.Lock()
        self._frame = None
        self._new = threading.Event()
        self._hist = collections.deque(maxlen=120)  # (เวลา, R, t)
        self._last_status = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, "/tray/pose", 10)
        self.arm_pub = self.create_publisher(PoseStamped, "/tray/pose_arm", 10)
        self.status_pub = self.create_publisher(String, "/tray/status", 10)
        self.img_pub = self.create_publisher(CompressedImage, "/tray/image/compressed", 2)
        self.create_subscription(Image, str(p("image_topic")), self._on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, str(p("camera_info_topic")), self._on_info, qos_profile_sensor_data)
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self.get_logger().info(f"หาป้าย ArUco id {self.marker_id} ({self.size * 1000:.0f} mm) บนพื้นกล่อง · "
                               f"ตำแหน่งเทียบ {self.target} ที่ /tray/pose_arm · ภาพ /tray/image/compressed")

    def _on_info(self, msg):
        self._K = np.array(msg.k, np.float64).reshape(3, 3)
        self._D = np.array(msg.d, np.float64)

    def _on_image(self, msg):
        with self._lock:
            self._frame = msg
        self._new.set()

    def _status(self, text):
        self.status_pub.publish(String(data=text))
        if text != self._last_status:
            self.get_logger().info(f"กล่อง: {text}")
            self._last_status = text

    def _to_target(self, frame, R, t):
        """ป้ายในเฟรมกล้อง -> เฟรมฐานแขน ผ่าน TF (ผล hand-eye) หรือ None ถ้ายังไม่มี TF"""
        try:
            tf = self.tf_buffer.lookup_transform(self.target, frame, Time())
        except TransformException:
            return None
        q, p = tf.transform.rotation, tf.transform.translation
        Rb = hand_eye.quat_to_mat(q.x, q.y, q.z, q.w)
        return Rb @ R, Rb @ t + np.array([p.x, p.y, p.z])

    def _loop(self):
        while self._running:
            if not self._new.wait(timeout=1.0):
                continue
            with self._lock:
                msg = self._frame
                self._new.clear()
            if msg is None or not self._running:
                continue
            if self._K is None:
                self._status("no_camera_info")
                continue
            img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
            img = img.reshape(msg.height, msg.width, 3)
            bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else img.copy()
            now = time.monotonic()
            corners, ids, _ = self._detect(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
            found = None
            if ids is not None:
                for cr, i in zip(corners, ids.ravel()):
                    if i == self.marker_id:
                        found = cr
            lines, color = [], (0, 0, 255)
            if found is None:
                self._hist.clear()
                status = "no_marker"
                lines.append(f"NO MARKER id {self.marker_id}")
            else:
                res = tray.marker_pose(found, self._K, self._D, self.size)
                cv2.polylines(bgr, [np.round(found).astype(np.int32)], True, (0, 255, 0), 2)
                if res is None:
                    status = "unstable"
                else:
                    R, t, err = res
                    self._hist.append((now, R, t))
                    while self._hist and now - self._hist[0][0] > self.window:
                        self._hist.popleft()
                    cv2.drawFrameAxes(bgr, self._K, self._D, cv2.Rodrigues(R)[0], t, self.size * 0.6, 2)
                    avg = tray.average_pose([h[1] for h in self._hist], [h[2] for h in self._hist])
                    status, color = self._publish(msg, avg, R, t, lines)
                    lines.append(f"dist {np.linalg.norm(t):.3f} m  reproj {err:.1f} px")
            self._status(status)
            if self.img_pub.get_subscription_count():
                self._draw(bgr, msg, [status.upper()] + lines, color)

    def _publish(self, msg, avg, R_now, t_now, lines):
        """ส่งตำแหน่งเมื่อนิ่ง -> (สถานะ, สีข้อความ)"""
        if not tray.facing_camera(R_now, t_now, self.max_tilt):
            return "tilted", (0, 0, 255)
        if avg is None:
            return "unstable", (0, 165, 255)
        R, t, spread, n = avg
        lines.append(f"frames {n}  spread {spread:.1f} mm")
        if n < self.min_frames or spread > self.max_spread:
            return "unstable", (0, 165, 255)
        frame = msg.header.frame_id
        self.pose_pub.publish(pose_msg(frame, msg.header.stamp, R, t))
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = "tray_marker"
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = (float(v) for v in t)
        q = hand_eye.mat_to_quat(R)
        r = tf.transform.rotation
        r.x, r.y, r.z, r.w = (float(v) for v in q)
        self.tf_pub.sendTransform(tf)
        arm = self._to_target(frame, R, t)
        if arm is None:
            lines.append(f"no TF {self.target} -> camera (hand-eye?)")
        else:
            self.arm_pub.publish(pose_msg(self.target, msg.header.stamp, *arm))
            x, y, z = arm[1]
            lines.append(f"arm {x:+.3f} {y:+.3f} {z:+.3f} m")
        return "ok", (0, 200, 0)

    def _draw(self, bgr, msg, lines, color):
        h = 26 * len(lines) + 12
        cv2.rectangle(bgr, (0, 0), (bgr.shape[1], h), (0, 0, 0), -1)  # พื้นทึบ (ขอบตัวหนังสือทำให้เพี้ยน)
        for i, text in enumerate(lines):
            cv2.putText(bgr, text, (10, 28 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        color if i == 0 else (255, 255, 255), 2, cv2.LINE_AA)
        ok, jpg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            self.img_pub.publish(CompressedImage(header=msg.header, format="jpeg", data=jpg.tobytes()))

    def destroy_node(self):
        self._running = False  # หยุด thread ก่อน ไม่ให้ publish ตอนโหนดถูกทำลาย
        self._new.set()
        self._thread.join(timeout=2.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = TrayDetector()
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
