#!/usr/bin/env python3
"""tray_teach — สอนจับกล่องหลายตำแหน่ง = คาลิเบรตกล้องกับแขนไปในตัว (ไม่ต้องติดป้ายที่มือ ไม่ต้องพาแขนไปทั่วภาพ)

  หน้าต่าง 1: arm start left (+ arm moveit ไว้จ็อก)
  หน้าต่าง 2: docker exec -it humanoi_ros /entrypoint.sh ros2 launch humanoi_vision tray.launch.py
  หน้าต่าง 3: docker exec -it humanoi_ros /entrypoint.sh ros2 run humanoi_vision tray_teach

ทำซ้ำ 6-8 ครั้ง: วางกล่อง (ป้าย id 1 บนพื้นกล่อง) -> จ็อกแขนให้มือจับหูกล่องจริง (จุดเดิมบนหูทุกครั้ง) -> กด Enter
ระหว่างครั้ง เลื่อนกล่องซ้าย/ขวา/หน้า/หลัง ~5 cm และหมุนเบี้ยว ±15° (ต้องเลื่อนอย่างน้อย 2 ทิศ)
แต่ละครั้งได้ (ข้อมือ link_6 ในฐานแขน, ป้ายในกล้อง) -> ความสัมพันธ์ ข้อมือ = กล้อง · ป้าย · จุดจับ
-> หาตำแหน่งกล้องเทียบแขน และจุดจับเทียบป้าย พร้อมกัน (tray.solve_grasp_camera)
แม่นที่สุดรอบๆ บริเวณที่สอน (ทดสอบจำลอง: ~3-5 mm) — ตำแหน่งกล้องที่ได้ ความสูงอาจเพี้ยน (บนโต๊ะเดียวกันไม่มีผล)

คำสั่ง: Enter = เก็บครั้งนี้ · u = ลบครั้งล่าสุด · d <n> = ลบครั้งที่ n · l = ดูที่เก็บ · w = บันทึกผล · q = ออก
ผล: humanoi_description/config/camera_to_<arm>_arm.yaml (TF ฐานแขน -> camera_link) +
    humanoi_description/config/tray_grasp_<arm>.yaml (จุดจับเทียบป้าย + มุมข้อต่อตอนสอนทุกครั้ง)
"""
import collections
import os
import shutil
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from humanoi_vision import hand_eye, tray

CONFIG = "/humanoi_ws/src/humanoi_description/config"
MIN_CAPTURES = 4
GOOD_MM = 10.0      # คลาดต่อครั้งเกินนี้ = ครั้งนั้นน่าสงสัย (จับคนละจุด / ข้อ 1 รูด / ป้ายอ่านพลาด)


def tf_mat(tf):
    q, p = tf.transform.rotation, tf.transform.translation
    return tray.mat(hand_eye.quat_to_mat(q.x, q.y, q.z, q.w), [p.x, p.y, p.z])


def pose_mat(msg):
    q, p = msg.pose.orientation, msg.pose.position
    return tray.mat(hand_eye.quat_to_mat(q.x, q.y, q.z, q.w), [p.x, p.y, p.z])


class TrayTeach(Node):
    def __init__(self):
        super().__init__("tray_teach")
        self.declare_parameter("arm", "left")
        self.declare_parameter("camera_link", "camera_link")
        self.arm = str(self.get_parameter("arm").value)
        self.base, self.tool = f"{self.arm}_base_link", f"{self.arm}_link_6"
        self.cam_link = str(self.get_parameter("camera_link").value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lock = threading.Lock()
        self.tray_msg, self.tray_t, self.status = None, 0.0, "-"
        self.joints = collections.deque(maxlen=300)  # (เวลา, ชื่อ, มุม)
        self.create_subscription(PoseStamped, "/tray/pose", self._on_tray, 10)
        self.create_subscription(String, "/tray/status", lambda m: setattr(self, "status", m.data), 10)
        self.create_subscription(JointState, "/joint_states", self._on_js, 50)
        self.caps = []  # dict: A, C, joints, names

    def _on_tray(self, msg):
        with self.lock:
            self.tray_msg, self.tray_t = msg, time.monotonic()

    def _on_js(self, msg):
        with self.lock:
            self.joints.append((time.monotonic(), list(msg.name), np.array(msg.position, float)))

    def arm_still(self, window=0.8, tol=0.002):
        with self.lock:
            now = time.monotonic()
            js = [j for j in self.joints if now - j[0] <= window]
        if len(js) < 3 or js[0][0] > now - window + 0.2:
            return None
        names = js[-1][1]
        pos = np.array([j[2] for j in js if j[1] == names])
        return (names, pos[-1]) if np.ptp(pos, axis=0).max() <= tol else None

    def capture(self):
        """-> ข้อความผล"""
        with self.lock:
            msg, age = self.tray_msg, time.monotonic() - self.tray_t
        if msg is None or age > 1.0 or self.status != "ok":
            return f"✗ ยังไม่เห็นกล่องนิ่ง (สถานะ {self.status}) — เปิด tray.launch.py และดูว่ากล้องเห็นป้ายทั้งแผ่น"
        still = self.arm_still()
        if still is None:
            return "✗ แขนยังขยับอยู่ (หรือไม่มี /joint_states) — รอแขนนิ่งแล้วกด Enter ใหม่"
        try:
            A = tf_mat(self.tf_buffer.lookup_transform(self.base, self.tool, Time()))
        except TransformException as exc:
            return f"✗ ไม่มี TF {self.base} -> {self.tool} ({exc}) — เปิด arm start ก่อน"
        C = pose_mat(msg)
        self.caps.append({"A": A, "C": C, "names": still[0], "joints": still[1], "frame": msg.header.frame_id})
        return (f"✓ ครั้งที่ {len(self.caps)}: ป้ายห่างกล้อง {np.linalg.norm(C[:3, 3]):.3f} m · "
                f"ข้อมือ ({A[0, 3]:+.3f}, {A[1, 3]:+.3f}, {A[2, 3]:+.3f}) m")

    def solve(self):
        As, Cs = [c["A"] for c in self.caps], [c["C"] for c in self.caps]
        return tray.solve_grasp_camera(As, Cs)

    def report(self):
        n = len(self.caps)
        if n < 3:
            return f"  เก็บแล้ว {n} ครั้ง — ต้องอย่างน้อย {MIN_CAPTURES} (แนะนำ 6-8)"
        X, Y, pos, ang, spread = self.solve()
        rms = 1000 * np.sqrt(np.mean(pos ** 2))
        lines = [f"  {n} ครั้ง: คลาดเฉลี่ย {rms:.1f} mm · มุม {np.degrees(np.sqrt(np.mean(ang ** 2))):.1f}° · "
                 f"การกระจาย {spread:.2f}" + (" (ต่ำ: เลื่อนกล่องอีกทิศหนึ่ง)" if spread < 0.3 else "")]
        lines += [f"    #{i + 1}: {1000 * p:.1f} mm {np.degrees(a):.1f}°" + ("  ← น่าสงสัย (d <n> ลบแล้วเก็บใหม่)"
                  if 1000 * p > GOOD_MM else "") for i, (p, a) in enumerate(zip(pos, ang))]
        return "\n".join(lines)

    def write(self):
        n = len(self.caps)
        if n < MIN_CAPTURES:
            return f"✗ ต้องเก็บอย่างน้อย {MIN_CAPTURES} ครั้ง (ตอนนี้ {n})"
        X, Y, pos, ang, spread = self.solve()
        if spread < 0.3:
            return "✗ กล่องเลื่อนทิศเดียว — เลื่อนอีกทิศ (หน้า-หลัง หรือ ซ้าย-ขวา) แล้วเก็บเพิ่มก่อน"
        try:  # ไดรเวอร์กล้องส่ง camera_link -> optical เอง -> ท่าของ camera_link ในฐานแขน
            T_lo = tf_mat(self.tf_buffer.lookup_transform(self.cam_link, self.caps[0]["frame"], Time()))
        except TransformException as exc:
            return f"✗ ไม่มี TF {self.cam_link} -> {self.caps[0]['frame']} ({exc})"
        T_bl = X @ tray.inv(T_lo)
        rms = 1000 * float(np.sqrt(np.mean(pos ** 2)))
        stamp = time.strftime("%Y-%m-%d %H:%M %Z")
        cam_path = os.path.join(CONFIG, f"camera_to_{self.arm}_arm.yaml")
        grasp_path = os.path.join(CONFIG, f"tray_grasp_{self.arm}.yaml")
        for path in (cam_path, grasp_path):
            if os.path.exists(path):
                shutil.copy2(path, path + time.strftime(".bak-%Y%m%d-%H%M%S"))
        q = hand_eye.mat_to_quat(T_bl[:3, :3])
        t = T_bl[:3, 3]
        with open(cam_path, "w", encoding="utf-8") as f:
            f.write(
                f"# ตำแหน่งกล้อง Astra Pro เทียบกับฐานแขน{self.arm} — humanoi_vision tray_teach {stamp} จากการสอนจับกล่อง "
                f"{n} ครั้ง\n# คลาดเฉลี่ย {rms:.1f} mm (ท่าจับที่ทำนาย vs ที่สอน) · แม่นรอบบริเวณที่สอน · "
                "ความสูงกล้องอาจเพี้ยน (ไม่มีผลกับกล่องบนโต๊ะเดิม)\n"
                f"parent_frame: {self.base}\nchild_frame: {self.cam_link}\n"
                f"x: {t[0]:.5f}\ny: {t[1]:.5f}\nz: {t[2]:.5f}\n"
                f"qx: {q[0]:.6f}\nqy: {q[1]:.6f}\nqz: {q[2]:.6f}\nqw: {q[3]:.6f}\nrms_mm: {rms:.2f}\nmethod: tray_teach\n")
        qy = hand_eye.mat_to_quat(Y[:3, :3])
        names = self.caps[0]["names"]
        joints = "\n".join("  - [" + ", ".join(f"{v:.5f}" for v in c["joints"]) + "]" for c in self.caps)
        with open(grasp_path, "w", encoding="utf-8") as f:
            f.write(
                f"# จุดจับหูกล่องของแขน{self.arm} เทียบป้ายบนพื้นกล่อง (ป้าย <- {self.tool}) — tray_teach {stamp}\n"
                f"# ท่าจับรอบใหม่ = TF(ฐานแขน <- กล้อง) · ป้ายที่กล้องเห็น · ค่านี้\n"
                f"tool_frame: {self.tool}\n"
                f"x: {Y[0, 3]:.5f}\ny: {Y[1, 3]:.5f}\nz: {Y[2, 3]:.5f}\n"
                f"qx: {qy[0]:.6f}\nqy: {qy[1]:.6f}\nqz: {qy[2]:.6f}\nqw: {qy[3]:.6f}\nrms_mm: {rms:.2f}\n"
                f"# มุมข้อต่อตอนสอนแต่ละครั้ง (rad)\njoint_names: [{', '.join(names)}]\nposes:\n{joints}\n")
        return (f"✓ บันทึกแล้ว ({n} ครั้ง คลาดเฉลี่ย {rms:.1f} mm)\n  {cam_path}\n  {grasp_path}\n"
                "  เปิด tray.launch.py ใหม่ให้ใช้ตำแหน่งกล้องใหม่")


def main():
    rclpy.init()
    node = TrayTeach()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    print(f"สอนจับกล่อง แขน{node.arm} — Enter = เก็บ · u = ลบล่าสุด · d <n> = ลบครั้งที่ n · l = ดูที่เก็บ · w = บันทึก · q = ออก\n"
          "แต่ละครั้ง: วางกล่อง -> จ็อกแขนให้มือจับหูกล่อง (จุดเดิมบนหู) -> รอแขนนิ่ง -> Enter\n"
          "ระหว่างครั้ง เลื่อนกล่อง ~5 cm ซ้าย/ขวา/หน้า/หลัง และหมุนเบี้ยว ±15° · แนะนำ 6-8 ครั้ง")
    try:
        while True:
            cmd = input(f"[{len(node.caps)} ครั้ง] > ").strip().lower()
            if cmd == "q":
                break
            if cmd == "":
                print(node.capture())
                if len(node.caps) >= 3:
                    print(node.report())
            elif cmd == "u" or cmd.startswith("d "):
                k = len(node.caps) if cmd == "u" else int(cmd[2:]) if cmd[2:].strip().isdigit() else -1
                if 1 <= k <= len(node.caps):
                    node.caps.pop(k - 1)
                    print(f"  ลบครั้งที่ {k} แล้ว เหลือ {len(node.caps)} ครั้ง")
                    if len(node.caps) >= 3:
                        print(node.report())
                else:
                    print("  ไม่มีครั้งนั้น")
            elif cmd == "l":
                print(node.report())
            elif cmd == "w":
                print(node.write())
            else:
                print("  Enter = เก็บ · u = ลบล่าสุด · d <n> = ลบครั้งที่ n · l = ดู · w = บันทึก · q = ออก")
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        ex.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
