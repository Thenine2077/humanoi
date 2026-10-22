#!/usr/bin/env python3
"""tray_teach — สอนจับกล่องหลายตำแหน่ง = คาลิเบรตกล้องกับแขนไปในตัว (ไม่ต้องติดป้ายที่มือ ไม่ต้องพาแขนไปทั่วภาพ)

  arm start left · arm moveit (จ็อก) · arm tray (กล้อง + หากล่อง) · arm tray-teach

ทำซ้ำ 6-8 ครั้ง: วางกล่อง (ป้าย id 1 บนพื้นกล่อง) -> จ็อกแขนให้ "จุดเดิมบนนิ้ว" แตะ "จุดเดิมบนหูกล่อง" -> กด Enter
ข้อมือเอียงต่างกันแต่ละครั้งได้ (แขนไปทุกตำแหน่งด้วยท่ามือเดิมไม่ได้) — ขอแค่จุดแตะเป็นจุดเดิม
ระหว่างครั้ง เลื่อนกล่องซ้าย/ขวา/หน้า/หลัง ~5 cm และหมุนเบี้ยว ±15° (ต้องเลื่อนอย่างน้อย 2 ทิศ + หมุน)
คำนวณ (tray.solve_contact_point): จุดบนนิ้ว p (ข้อมือ) = จุดบนหู q (ป้าย) ทุกครั้ง -> ตำแหน่งกล้องเทียบแขน + p + q
ตอนหยิบจริง: ใช้ท่าข้อมือของครั้งที่สอนซึ่งกล่องวางใกล้ที่สุด ย้ายตามป้ายที่เห็น (tray.nearest_grasp)
ทดสอบจำลอง (สอน 7 ครั้ง ข้อมือเอียงต่าง ±12°): ท่าข้อมือคลาดกลางๆ ~5 mm · แม่นรอบบริเวณที่สอนเท่านั้น

คำสั่ง: Enter = เก็บครั้งนี้ · u = ลบครั้งล่าสุด · d <n> = ลบครั้งที่ n · l = ดูที่เก็บ · w = บันทึกผล · q = ออก
ทุกครั้งที่เก็บ/ลบ บันทึกข้อมูลดิบไว้ที่ config/tray_teach_<arm>.captures.npz (โปรแกรมปิดไปก็ไม่หาย — เปิดใหม่โหลดคืน)
ผล (w): humanoi_description/config/camera_to_<arm>_arm.yaml (TF ฐานแขน -> camera_link) +
        humanoi_description/config/tray_grasp_<arm>.yaml (ท่าจับเทียบป้ายทุกครั้ง + จุดแตะ + มุมข้อต่อ)
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
MIN_CAPTURES = 5    # ไม่รู้ 12 ค่า แต่ละครั้งให้ 3 สมการ
GOOD_MM = 10.0      # คลาดต่อครั้งเกินนี้ = ครั้งนั้นน่าสงสัย (แตะคนละจุด / ข้อ 1 รูด / ป้ายอ่านพลาด)
SAME_MM = 2.0       # ข้อมือห่างครั้งก่อนไม่ถึงนี้ = น่าจะกด Enter ซ้ำโดยแขนไม่ได้ขยับ


def tf_mat(tf):
    q, p = tf.transform.rotation, tf.transform.translation
    return tray.mat(hand_eye.quat_to_mat(q.x, q.y, q.z, q.w), [p.x, p.y, p.z])


def pose_mat(msg):
    q, p = msg.pose.orientation, msg.pose.position
    return tray.mat(hand_eye.quat_to_mat(q.x, q.y, q.z, q.w), [p.x, p.y, p.z])


def yaml_pose(T, indent="  "):
    q = hand_eye.mat_to_quat(T[:3, :3])
    return (f"{indent}xyz: [{T[0, 3]:.5f}, {T[1, 3]:.5f}, {T[2, 3]:.5f}]\n"
            f"{indent}quat: [{q[0]:.6f}, {q[1]:.6f}, {q[2]:.6f}, {q[3]:.6f}]   # x y z w\n")


class TrayTeach(Node):
    def __init__(self):
        super().__init__("tray_teach")
        self.declare_parameter("arm", "left")
        self.declare_parameter("camera_link", "camera_link")
        self.arm = str(self.get_parameter("arm").value)
        self.base, self.tool = f"{self.arm}_base_link", f"{self.arm}_link_6"
        self.cam_link = str(self.get_parameter("camera_link").value)
        self.save_path = os.path.join(CONFIG, f"tray_teach_{self.arm}.captures.npz")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lock = threading.Lock()
        self.tray_msg, self.tray_t, self.status = None, 0.0, "-"
        self.joints = collections.deque(maxlen=300)  # (เวลา, ชื่อ, มุม)
        self.create_subscription(PoseStamped, "/tray/pose", self._on_tray, 10)
        self.create_subscription(String, "/tray/status", lambda m: setattr(self, "status", m.data), 10)
        self.create_subscription(JointState, "/joint_states", self._on_js, 50)
        self.caps = []  # dict: A, C, names, joints, frame

    def _on_tray(self, msg):
        with self.lock:
            self.tray_msg, self.tray_t = msg, time.monotonic()

    def _on_js(self, msg):
        with self.lock:
            self.joints.append((time.monotonic(), list(msg.name), np.array(msg.position, float)))

    # ---------- เก็บ / โหลดข้อมูลดิบ ----------
    def autosave(self):
        if not self.caps:
            if os.path.exists(self.save_path):
                os.remove(self.save_path)
            return
        np.savez(self.save_path, A=np.array([c["A"] for c in self.caps]), C=np.array([c["C"] for c in self.caps]),
                 joints=np.array([c["joints"] for c in self.caps]), names=np.array(self.caps[0]["names"]),
                 frame=np.array(self.caps[0]["frame"]))

    def load_saved(self):
        if not os.path.exists(self.save_path):
            return 0
        d = np.load(self.save_path)
        names, frame = [str(n) for n in d["names"]], str(d["frame"])
        self.caps = [{"A": A, "C": C, "names": names, "joints": j, "frame": frame}
                     for A, C, j in zip(d["A"], d["C"], d["joints"])]
        return len(self.caps)

    # ---------- เก็บครั้งใหม่ ----------
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
            return f"✗ ยังไม่เห็นกล่องนิ่ง (สถานะ {self.status}) — เปิด arm tray และดูว่ากล้องเห็นป้ายทั้งแผ่น"
        still = self.arm_still()
        if still is None:
            return "✗ แขนยังขยับอยู่ (หรือไม่มี /joint_states) — รอแขนนิ่งแล้วกด Enter ใหม่"
        try:
            A = tf_mat(self.tf_buffer.lookup_transform(self.base, self.tool, Time()))
        except TransformException as exc:
            return f"✗ ไม่มี TF {self.base} -> {self.tool} ({exc}) — เปิด arm start ก่อน"
        C = pose_mat(msg)
        warn = ""
        if self.caps:
            dA = np.linalg.norm(A[:3, 3] - self.caps[-1]["A"][:3, 3]) * 1000
            dC = np.linalg.norm(C[:3, 3] - self.caps[-1]["C"][:3, 3]) * 1000
            if dA < SAME_MM and dC > 5:
                warn = (f"\n  ⚠ ข้อมือแทบไม่ขยับ ({dA:.1f} mm) แต่กล่องเลื่อนไป {dC:.0f} mm — มือแตะหูจริงไหม? "
                        "ถ้าไม่ใช่ กด u ลบ")
            elif dA < SAME_MM and dC < 2:
                warn = "\n  ⚠ ซ้ำกับครั้งก่อน (กล่องกับแขนไม่ได้ขยับ) — กด u ลบ"
        self.caps.append({"A": A, "C": C, "names": still[0], "joints": still[1], "frame": msg.header.frame_id})
        self.autosave()
        return (f"✓ ครั้งที่ {len(self.caps)}: ป้ายห่างกล้อง {np.linalg.norm(C[:3, 3]):.3f} m · "
                f"ข้อมือ ({A[0, 3]:+.3f}, {A[1, 3]:+.3f}, {A[2, 3]:+.3f}) m" + warn)

    # ---------- คำนวณ ----------
    def solve(self):
        return tray.solve_contact_point([c["A"] for c in self.caps], [c["C"] for c in self.caps])

    def report(self):
        n = len(self.caps)
        if n < MIN_CAPTURES:
            return f"  เก็บแล้ว {n} ครั้ง — ต้องอย่างน้อย {MIN_CAPTURES} (แนะนำ 6-8: เลื่อน 2 ทิศ + หมุน ±15°)"
        X, p, q, err, spread = self.solve()
        turns = [abs(tray.turn_deg(self.caps[0]["C"][:3, :3], c["C"][:3, :3])) for c in self.caps]
        lines = [f"  {n} ครั้ง: คลาดเฉลี่ย {1000 * np.sqrt(np.mean(err ** 2)):.1f} mm · การกระจาย {spread:.2f}"
                 + (" (ต่ำ: เลื่อนกล่องอีกทิศ)" if spread < 0.3 else "")
                 + (" · ยังไม่ได้หมุนกล่อง (หมุน ±15° อย่างละครั้ง)" if max(turns) < 8 else "")
                 + f" · จุดแตะบนนิ้วห่างข้อมือ {100 * np.linalg.norm(p):.0f} cm"]
        lines += [f"    #{i + 1}: {1000 * e:.1f} mm" + ("  ← น่าสงสัย (d <n> ลบแล้วเก็บใหม่)" if 1000 * e > GOOD_MM else "")
                  for i, e in enumerate(err)]
        return "\n".join(lines)

    def write(self):
        n = len(self.caps)
        if n < MIN_CAPTURES:
            return f"✗ ต้องเก็บอย่างน้อย {MIN_CAPTURES} ครั้ง (ตอนนี้ {n})"
        X, p, q, err, spread = self.solve()
        if spread < 0.3:
            return "✗ กล่องเลื่อนทิศเดียว — เลื่อนอีกทิศ (หน้า-หลัง หรือ ซ้าย-ขวา) แล้วเก็บเพิ่มก่อน"
        try:  # ไดรเวอร์กล้องส่ง camera_link -> optical เอง -> ท่าของ camera_link ในฐานแขน
            T_lo = tf_mat(self.tf_buffer.lookup_transform(self.cam_link, self.caps[0]["frame"], Time()))
        except TransformException as exc:
            return f"✗ ไม่มี TF {self.cam_link} -> {self.caps[0]['frame']} ({exc}) — arm tray เปิดอยู่ไหม"
        T_bl = X @ tray.inv(T_lo)
        rms = 1000 * float(np.sqrt(np.mean(err ** 2)))
        stamp = time.strftime("%Y-%m-%d %H:%M %Z")
        cam_path = os.path.join(CONFIG, f"camera_to_{self.arm}_arm.yaml")
        grasp_path = os.path.join(CONFIG, f"tray_grasp_{self.arm}.yaml")
        for path in (cam_path, grasp_path):
            if os.path.exists(path):
                shutil.copy2(path, path + time.strftime(".bak-%Y%m%d-%H%M%S"))
        qc, t = hand_eye.mat_to_quat(T_bl[:3, :3]), T_bl[:3, 3]
        with open(cam_path, "w", encoding="utf-8") as f:
            f.write(
                f"# ตำแหน่งกล้อง Astra Pro เทียบกับฐานแขน{self.arm} — humanoi_vision tray_teach {stamp} จากการสอนจับกล่อง "
                f"{n} ครั้ง\n# คลาดเฉลี่ย {rms:.1f} mm (จุดแตะ) · แม่นรอบบริเวณที่สอนบนโต๊ะนี้\n"
                f"parent_frame: {self.base}\nchild_frame: {self.cam_link}\n"
                f"x: {t[0]:.5f}\ny: {t[1]:.5f}\nz: {t[2]:.5f}\n"
                f"qx: {qc[0]:.6f}\nqy: {qc[1]:.6f}\nqz: {qc[2]:.6f}\nqw: {qc[3]:.6f}\nrms_mm: {rms:.2f}\nmethod: tray_teach\n")
        Xinv = tray.inv(X)
        grasps = "".join(f"  - marker_in_camera:\n{yaml_pose(c['C'], '      ')}"
                         f"    wrist_in_marker:\n{yaml_pose(tray.inv(c['C']) @ Xinv @ c['A'], '      ')}"
                         f"    joints: [" + ", ".join(f"{v:.5f}" for v in c["joints"]) + "]\n"
                         for c in self.caps)
        with open(grasp_path, "w", encoding="utf-8") as f:
            f.write(
                f"# ท่าจับหูกล่องของแขน{self.arm} — tray_teach {stamp} ({n} ครั้ง คลาดเฉลี่ย {rms:.1f} mm)\n"
                "# ตอนหยิบ: เลือกครั้งที่กล่อง (marker_in_camera) ใกล้ตำแหน่งที่เห็นที่สุด แล้วข้อมือ = TF(ฐานแขน <- กล้อง)"
                " · ป้ายที่เห็น · wrist_in_marker\n"
                f"tool_frame: {self.tool}\n"
                f"contact_on_tool: [{p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}]     # จุดแตะบนนิ้ว (เฟรมข้อมือ)\n"
                f"contact_on_marker: [{q[0]:.4f}, {q[1]:.4f}, {q[2]:.4f}]   # จุดบนหูกล่อง (เฟรมป้าย)\n"
                f"joint_names: [{', '.join(self.caps[0]['names'])}]\ngrasps:\n{grasps}")
        return (f"✓ บันทึกแล้ว ({n} ครั้ง คลาดเฉลี่ย {rms:.1f} mm)\n  {cam_path}\n  {grasp_path}\n"
                "  ปิด arm tray แล้วเปิดใหม่ ให้ใช้ตำแหน่งกล้องใหม่")


def main():
    rclpy.init()
    node = TrayTeach()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    print(f"สอนจับกล่อง แขน{node.arm} — Enter = เก็บ · u = ลบล่าสุด · d <n> = ลบครั้งที่ n · l = ดู · w = บันทึก · q = ออก\n"
          "แต่ละครั้ง: วางกล่อง -> จ็อกแขนให้จุดเดิมบนนิ้วแตะจุดเดิมบนหู (ข้อมือเอียงต่างกันได้) -> รอแขนนิ่ง -> Enter\n"
          "ระหว่างครั้ง เลื่อนกล่อง ~5 cm 2 ทิศ และหมุนเบี้ยว ±15° · แนะนำ 6-8 ครั้ง")
    k = node.load_saved()
    if k:
        print(f"  โหลดที่เก็บไว้ครั้งก่อน {k} ครั้ง ({node.save_path}) — เก็บต่อได้เลย หรือ d/u ลบ")
        print(node.report())
    try:
        while True:
            cmd = input(f"[{len(node.caps)} ครั้ง] > ").strip().lower()
            if cmd == "q":
                break
            if cmd == "":
                print(node.capture())
                if len(node.caps) >= MIN_CAPTURES:
                    print(node.report())
            elif cmd == "u" or cmd.startswith("d "):
                k = len(node.caps) if cmd == "u" else int(cmd[2:]) if cmd[2:].strip().isdigit() else -1
                if 1 <= k <= len(node.caps):
                    node.caps.pop(k - 1)
                    node.autosave()
                    print(f"  ลบครั้งที่ {k} แล้ว เหลือ {len(node.caps)} ครั้ง")
                    if len(node.caps) >= MIN_CAPTURES:
                        print(node.report())
                else:
                    print("  ไม่มีครั้งนั้น (d ตามด้วยเลขครั้ง เช่น d 2)")
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
