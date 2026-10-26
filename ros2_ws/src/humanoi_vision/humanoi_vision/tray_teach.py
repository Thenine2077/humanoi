#!/usr/bin/env python3
"""tray_teach — สอนหยิบกล่องแบบทางลัด: สอนท่าจับจริง + ชดเชยระยะที่กล่องเลื่อนไปจากตอนสอน (ไม่ต้องติดป้ายที่มือ)

  arm start left · arm moveit (จ็อก) · arm tray (กล้อง + หากล่อง) · arm tray-teach

ขั้น 1 ทิศกล้อง (4-6 ครั้ง): จ็อกให้ "จุดเดิมบนนิ้ว" แตะ "จุดเดิมบนกล่อง" -> แขนนิ่ง -> Enter
  ข้อมือต้องมุมเดิมทุกครั้ง (ใน RViz ลากแค่ลูกศรเลื่อน ไม่ลากวงหมุน) · กล่องเลื่อนอย่างเดียว ไม่หมุน
  ระหว่างครั้ง เลื่อนกล่อง ~3-7 cm อย่างน้อย 2 ทิศ (ซ้าย/ขวา + หน้า/หลัง)
  ท่าแตะที่ข้อมือหรือกล่องหมุนไปจากกลุ่ม (เกิน 3°) ไม่ถูกใช้เอง — ไม่ต้องลบก็ได้
ขั้น 2 ท่าจับจริง (1 ครั้งขึ้นไป): วางกล่องที่ตำแหน่งปกติ จัดนิ้ว tray_grip จับหูกล่องจริง -> g
  สอนเพิ่มตอนกล่องอยู่ที่อื่น = ครอบคลุมกว้างขึ้น (ตอนหยิบเลือกท่าที่กล่องใกล้สุด) + วัดความแม่นของทางลัดได้จริง
คำนวณ (tray.delta_rotation): ข้อมือเลื่อน = R · ป้ายเลื่อน -> ทิศกล้องเทียบแขน R (ไม่ต้องรู้ตำแหน่งกล้อง/จุดแตะบนนิ้ว)
ตอนหยิบ (tray.delta_grasp): ข้อมือ = ท่าจับที่สอน + R · (ป้ายตอนนี้ − ป้ายตอนสอน) ข้อมือคงมุมเดิม
  ใช้ได้เมื่อกล่องห่างจากตอนสอนไม่เกิน ~8 cm และหมุนไม่เกิน ~4° (การหมุนไม่ถูกชดเชย)

คำสั่ง: Enter = เก็บท่าแตะ · u = ลบท่าแตะล่าสุด · d <n> = ลบท่าแตะครั้งที่ n · g = เก็บท่าจับจริง
       gu = ลบท่าจับล่าสุด · l = ดูที่เก็บ · w = บันทึกผล · q = ออก
ทุกครั้งที่เก็บ/ลบ บันทึกข้อมูลดิบไว้ที่ config/tray_teach_<arm>.captures.npz (โปรแกรมปิดไปก็ไม่หาย — เปิดใหม่โหลดคืน)
ผล (w): humanoi_description/config/tray_grasp_<arm>.yaml (ทิศกล้อง + ท่าจับ: ป้ายในกล้อง ข้อมือในฐานแขน มุมข้อต่อ)
  ไม่แก้ camera_to_<arm>_arm.yaml — ข้อมูลเลื่อนอย่างเดียวบอกได้แค่ทิศกล้อง ไม่บอกตำแหน่ง
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
MIN_TOUCHES = 4     # ทิศกล้อง 3 ค่า · เลื่อน 2 ทิศ + เหลือไว้ตรวจกันเอง
SAME_DEG = 3.0      # ข้อมือ/กล่องหมุนต่างจากกลุ่มเกินนี้ = ไม่ใช้ท่าแตะนั้น
GOOD_MM = 15.0      # คลาดต่อครั้งเกินนี้ = ครั้งนั้นน่าสงสัย (แตะคนละจุด / ข้อ 1 รูด) — แตะด้วยตาคลาด ~1 cm เป็นปกติ
SAME_MM = 2.0       # ข้อมือห่างครั้งก่อนไม่ถึงนี้ = น่าจะกด Enter ซ้ำโดยแขนไม่ได้ขยับ
MAX_SHIFT = 0.08    # m — กล่องห่างท่าจับที่สอนเกินนี้ ตัวหยิบไม่หยิบ (เขียนลงไฟล์ผล)
MAX_TURN = 4.0      # องศา — การหมุนไม่ถูกชดเชย: หูกล่องห่างป้าย ~13 cm -> 4° คลาด ~9 mm


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


def vec(v, fmt=".6f"):
    return "[" + ", ".join(f"{x:{fmt}}" for x in v) + "]"


class TrayTeach(Node):
    def __init__(self):
        super().__init__("tray_teach")
        self.declare_parameter("arm", "left")
        self.arm = str(self.get_parameter("arm").value)
        self.base, self.tool = f"{self.arm}_base_link", f"{self.arm}_link_6"
        self.save_path = os.path.join(CONFIG, f"tray_teach_{self.arm}.captures.npz")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lock = threading.Lock()
        self.tray_msg, self.tray_t, self.status = None, 0.0, "-"
        self.joints = collections.deque(maxlen=300)  # (เวลา, ชื่อ, มุม)
        self.create_subscription(PoseStamped, "/tray/pose", self._on_tray, 10)
        self.create_subscription(String, "/tray/status", lambda m: setattr(self, "status", m.data), 10)
        self.create_subscription(JointState, "/joint_states", self._on_js, 50)
        self.caps = []    # ท่าแตะ (หาทิศกล้อง) dict: A, C, names, joints, frame
        self.grasps = []  # ท่าจับจริง dict แบบเดียวกัน

    def _on_tray(self, msg):
        with self.lock:
            self.tray_msg, self.tray_t = msg, time.monotonic()

    def _on_js(self, msg):
        with self.lock:
            self.joints.append((time.monotonic(), list(msg.name), np.array(msg.position, float)))

    # ---------- เก็บ / โหลดข้อมูลดิบ ----------
    def autosave(self):
        if not self.caps and not self.grasps:
            if os.path.exists(self.save_path):
                os.remove(self.save_path)
            return
        first = (self.caps or self.grasps)[0]
        nj = len(first["joints"])

        def arr(lst, key, shape):
            return np.array([c[key] for c in lst]) if lst else np.zeros(shape)

        np.savez(self.save_path, A=arr(self.caps, "A", (0, 4, 4)), C=arr(self.caps, "C", (0, 4, 4)),
                 joints=arr(self.caps, "joints", (0, nj)), GA=arr(self.grasps, "A", (0, 4, 4)),
                 GC=arr(self.grasps, "C", (0, 4, 4)), Gjoints=arr(self.grasps, "joints", (0, nj)),
                 names=np.array(first["names"]), frame=np.array(first["frame"]))

    def load_saved(self):
        if not os.path.exists(self.save_path):
            return 0
        d = np.load(self.save_path)
        names, frame = [str(n) for n in d["names"]], str(d["frame"])
        self.caps = [{"A": A, "C": C, "names": names, "joints": j, "frame": frame}
                     for A, C, j in zip(d["A"], d["C"], d["joints"])]
        if "GA" in d:
            self.grasps = [{"A": A, "C": C, "names": names, "joints": j, "frame": frame}
                           for A, C, j in zip(d["GA"], d["GC"], d["Gjoints"])]
        return len(self.caps) + len(self.grasps)

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

    def snapshot(self):
        """ตอนนี้: dict (A ข้อมือในฐานแขน, C ป้ายในกล้อง, มุมข้อต่อ) หรือข้อความบอกว่าทำไมยังเก็บไม่ได้"""
        with self.lock:
            msg, age = self.tray_msg, time.monotonic() - self.tray_t
        if msg is None or age > 1.0 or self.status != "ok":
            return f"✗ ยังไม่เห็นกล่องนิ่ง (สถานะ {self.status}) — เปิด arm tray และดูว่ากล้องเห็นป้ายทั้งแผ่น"
        still = self.arm_still()
        if still is None:
            return "✗ แขนยังขยับอยู่ (หรือไม่มี /joint_states) — รอแขนนิ่งแล้วกดใหม่"
        try:
            A = tf_mat(self.tf_buffer.lookup_transform(self.base, self.tool, Time()))
        except TransformException as exc:
            return f"✗ ไม่มี TF {self.base} -> {self.tool} ({exc}) — เปิด arm start ก่อน"
        return {"A": A, "C": pose_mat(msg), "names": still[0], "joints": still[1], "frame": msg.header.frame_id}

    def capture(self):
        """เก็บท่าแตะ (หาทิศกล้อง) -> ข้อความผล"""
        snap = self.snapshot()
        if isinstance(snap, str):
            return snap
        A, C = snap["A"], snap["C"]
        warn = ""
        if self.caps:
            dA = np.linalg.norm(A[:3, 3] - self.caps[-1]["A"][:3, 3]) * 1000
            dC = np.linalg.norm(C[:3, 3] - self.caps[-1]["C"][:3, 3]) * 1000
            if dA < SAME_MM and dC > 5:
                warn = (f"\n  ⚠ ข้อมือแทบไม่ขยับ ({dA:.1f} mm) แต่กล่องเลื่อนไป {dC:.0f} mm — นิ้วแตะจุดจริงไหม? "
                        "ถ้าไม่ใช่ กด u ลบ")
            elif dA < SAME_MM and dC < 2:
                warn = "\n  ⚠ ซ้ำกับครั้งก่อน (กล่องกับแขนไม่ได้ขยับ) — กด u ลบ"
        self.caps.append(snap)
        self.autosave()
        _, skipped = tray.same_wrist_touches([c["A"] for c in self.caps], [c["C"] for c in self.caps], SAME_DEG)
        if len(self.caps) - 1 in skipped:
            w, t = skipped[len(self.caps) - 1]
            warn += (f"\n  ⚠ ข้อมือหมุน {w:.1f}° · กล่องหมุน {t:+.1f}° จากครั้งอื่น — ครั้งนี้จะไม่ถูกใช้ "
                     "(ข้อมือมุมเดิม: ลากแค่ลูกศรเลื่อน · กล่องเลื่อนอย่างเดียว) กด u ลบแล้วแตะใหม่")
        return (f"✓ ครั้งที่ {len(self.caps)}: ป้ายห่างกล้อง {np.linalg.norm(C[:3, 3]):.3f} m · "
                f"ข้อมือ ({A[0, 3]:+.3f}, {A[1, 3]:+.3f}, {A[2, 3]:+.3f}) m" + warn)

    def grasp(self):
        """เก็บท่าจับจริง -> ข้อความผล"""
        snap = self.snapshot()
        if isinstance(snap, str):
            return snap
        self.grasps.append(snap)
        self.autosave()
        s = self.solve()
        check = "" if isinstance(s, str) or len(self.grasps) < 2 else "\n" + self.grasp_checks(s["R"])[-1]
        return (f"✓ ท่าจับที่ {len(self.grasps)}: ป้ายห่างกล้อง {np.linalg.norm(snap['C'][:3, 3]):.3f} m" + check
                + ("\n  บันทึกด้วย w" if not isinstance(s, str) else f"\n  ยังบันทึกไม่ได้: {s}"))

    # ---------- คำนวณ ----------
    def solve(self):
        """ทิศกล้องจากท่าแตะ -> dict หรือข้อความบอกว่ายังขาดอะไร"""
        n = len(self.caps)
        if n < MIN_TOUCHES:
            return f"ท่าแตะ {n} ครั้ง — ต้องอย่างน้อย {MIN_TOUCHES} (ข้อมือมุมเดิม กล่องเลื่อน 2 ทิศ)"
        As, Cs = [c["A"] for c in self.caps], [c["C"] for c in self.caps]
        used, skipped = tray.same_wrist_touches(As, Cs, SAME_DEG)
        if len(used) < MIN_TOUCHES:
            return (f"ท่าแตะที่ข้อมือมุมเดียวกันมีแค่ {len(used)} ครั้ง — ต้องอย่างน้อย {MIN_TOUCHES} "
                    "(ลากแค่ลูกศรเลื่อน ไม่หมุนข้อมือ)")
        R, err, spread, wobble = tray.delta_rotation([As[i] for i in used], [Cs[i] for i in used])
        if spread < 0.3:
            return f"กล่องเลื่อนทิศเดียว (การกระจาย {spread:.2f}) — เลื่อนอีกทิศ (หน้า-หลัง หรือ ซ้าย-ขวา) แล้วแตะเพิ่ม"
        return {"R": R, "used": used, "skipped": skipped, "err": err, "spread": spread, "wobble": wobble}

    def grasp_checks(self, R):
        """ท่าจับที่ 2, 3, ... ทายจากท่าจับที่ 1 ด้วยทางลัด เทียบกับที่สอนจริง = ความแม่นของทางลัดจริงๆ"""
        if len(self.grasps) < 2:
            return []
        g0, lines = self.grasps[0], []
        for k, g in enumerate(self.grasps[1:], 2):
            T, shift, turn = tray.delta_grasp(R, g0["A"], g0["C"], g["C"])
            miss = 1000 * np.linalg.norm(T[:3, 3] - g["A"][:3, 3])
            wrist = tray.rot_deg(g0["A"][:3, :3], g["A"][:3, :3])
            lines.append(f"    ท่าจับ {k} ทายจากท่าจับ 1 (กล่องเลื่อน {1000 * shift:.0f} mm หมุน {turn:+.1f}°): "
                         f"คลาด {miss:.0f} mm" + (f" · ข้อมือสอนต่างมุมกัน {wrist:.1f}°" if wrist > SAME_DEG else ""))
        return lines

    def report(self):
        s = self.solve()
        if isinstance(s, str):
            lines = ["  " + s]
        else:
            lines = [f"  ทิศกล้องจากท่าแตะ {len(s['used'])} ครั้ง: การกระจาย {s['spread']:.2f} · แกว่ง ±{s['wobble']:.1f}° "
                     f"-> กล่องเลื่อน 5 cm มือคลาดเพิ่ม ~{50 * np.radians(s['wobble']):.0f} mm"]
            rows = {i: f"    #{i + 1}: {1000 * e:.1f} mm" + ("  ← ไม่เข้าพวก (แตะคนละจุด? d <n> ลบแล้วแตะใหม่ได้)"
                                                          if 1000 * e > GOOD_MM else "")
                    for i, e in zip(s["used"], s["err"])}
            rows.update({j: f"    #{j + 1}: ไม่ใช้ (ข้อมือหมุน {w:.1f}° · กล่องหมุน {t:+.1f}°)"
                         for j, (w, t) in s["skipped"].items()})
            lines += [rows[i] for i in sorted(rows)]
        lines.append(f"  ท่าจับจริง: {len(self.grasps)} ครั้ง" + ("" if self.grasps else
                     " — วางกล่องที่ตำแหน่งปกติ จัดนิ้ว tray_grip จับหูกล่องจริง แล้วกด g"))
        if not isinstance(s, str):
            lines += self.grasp_checks(s["R"])
        return "\n".join(lines)

    def write(self):
        s = self.solve()
        if isinstance(s, str):
            return "✗ " + s
        if not self.grasps:
            return "✗ ยังไม่มีท่าจับจริง — วางกล่องที่ตำแหน่งปกติ จัดนิ้ว tray_grip จับหูกล่องจริง แล้วกด g ก่อน"
        R, rms = s["R"], 1000 * float(np.sqrt(np.mean(s["err"] ** 2)))
        stamp = time.strftime("%Y-%m-%d %H:%M %Z")
        path = os.path.join(CONFIG, f"tray_grasp_{self.arm}.yaml")
        if os.path.exists(path):
            shutil.copy2(path, path + time.strftime(".bak-%Y%m%d-%H%M%S"))
        up = R @ self.grasps[0]["C"][:3, 2]  # แกน z ป้าย (ตั้งฉากพื้นกล่อง ชี้ขึ้น) ในฐานแขน
        grasps = "".join(f"  - marker_in_camera:\n{yaml_pose(g['C'], '      ')}"
                         f"    wrist_in_arm:\n{yaml_pose(g['A'], '      ')}"
                         f"    joints: {vec(g['joints'], '.5f')}\n"
                         for g in self.grasps)
        with open(path, "w", encoding="utf-8") as f:
            f.write(
                f"# ท่าจับหูกล่องของแขน{self.arm} — tray_teach {stamp} แบบทางลัด ({len(self.grasps)} ท่าจับ · "
                f"ทิศกล้องจากท่าแตะ {len(s['used'])} ครั้ง คลาดเฉลี่ย {rms:.1f} mm แกว่ง ±{s['wobble']:.1f}°)\n"
                "# ตอนหยิบ (tray.delta_grasp): เลือกท่าที่ marker_in_camera ใกล้ป้ายที่เห็นที่สุด แล้ว\n"
                "#   ข้อมือ = wrist_in_arm เลื่อนไป camera_rotation · (ป้ายที่เห็น − marker_in_camera) ข้อมือคงมุมเดิม\n"
                "# ไม่รู้ตำแหน่งกล้อง -> ไม่ชดเชยกล่องหมุน: ห่างเกิน max_shift_m หรือหมุนเกิน max_turn_deg ไม่หยิบ\n"
                "method: delta\n"
                f"arm_frame: {self.base}\ntool_frame: {self.tool}\ncamera_frame: {self.grasps[0]['frame']}\n"
                f"camera_rotation: {vec(hand_eye.mat_to_quat(R))}   # x y z w — ทิศกล้อง (camera_frame) ในฐานแขน\n"
                f"table_up: {vec(up, '.5f')}   # ตั้งฉากพื้นกล่อง ชี้ขึ้น ในฐานแขน — ทิศยกมือเข้า/ออกจากหู\n"
                f"max_shift_m: {MAX_SHIFT}\nmax_turn_deg: {MAX_TURN}\n"
                f"touch_rms_mm: {rms:.2f}\nrotation_wobble_deg: {s['wobble']:.2f}\n"
                f"joint_names: [{', '.join(self.grasps[0]['names'])}]\ngrasps:\n{grasps}")
        return (f"✓ บันทึกแล้ว ({len(self.grasps)} ท่าจับ · ทิศกล้องจากท่าแตะ {len(s['used'])} ครั้ง)\n  {path}\n"
                f"  ใช้ได้เมื่อกล่องห่างท่าจับที่สอนไม่เกิน {100 * MAX_SHIFT:.0f} cm และหมุนไม่เกิน {MAX_TURN:.0f}°")


def main():
    rclpy.init()
    node = TrayTeach()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    print(f"สอนหยิบกล่องแบบทางลัด แขน{node.arm}\n"
          f"ขั้น 1 ({MIN_TOUCHES}-6 ครั้ง): จ็อกให้จุดเดิมบนนิ้วแตะจุดเดิมบนกล่อง -> แขนนิ่ง -> Enter\n"
          "  ข้อมือมุมเดิมทุกครั้ง (ลากแค่ลูกศรเลื่อน) · ระหว่างครั้ง เลื่อนกล่อง 3-7 cm 2 ทิศ ไม่หมุนกล่อง\n"
          "ขั้น 2 (1 ครั้งขึ้นไป): กล่องที่ตำแหน่งปกติ นิ้วท่า tray_grip จับหูกล่องจริง -> g\n"
          "คำสั่ง: Enter แตะ · u / d <n> ลบท่าแตะ · g ท่าจับ · gu ลบท่าจับล่าสุด · l ดู · w บันทึก · q ออก")
    k = node.load_saved()
    if k:
        print(f"  โหลดที่เก็บไว้ครั้งก่อน: ท่าแตะ {len(node.caps)} · ท่าจับ {len(node.grasps)} ({node.save_path})")
        print(node.report())
    try:
        while True:
            cmd = input(f"[แตะ {len(node.caps)} · จับ {len(node.grasps)}] > ").strip().lower()
            if cmd == "q":
                break
            if cmd == "":
                print(node.capture())
                if len(node.caps) >= MIN_TOUCHES:
                    print(node.report())
            elif cmd == "u" or cmd.startswith("d "):
                k = len(node.caps) if cmd == "u" else int(cmd[2:]) if cmd[2:].strip().isdigit() else -1
                if 1 <= k <= len(node.caps):
                    node.caps.pop(k - 1)
                    node.autosave()
                    print(f"  ลบครั้งที่ {k} แล้ว เหลือ {len(node.caps)} ครั้ง")
                    if len(node.caps) >= MIN_TOUCHES:
                        print(node.report())
                else:
                    print("  ไม่มีครั้งนั้น (d ตามด้วยเลขครั้ง เช่น d 2)")
            elif cmd == "g":
                print(node.grasp())
            elif cmd == "gu":
                if node.grasps:
                    node.grasps.pop()
                    node.autosave()
                    print(f"  ลบท่าจับล่าสุดแล้ว เหลือ {len(node.grasps)}")
                else:
                    print("  ยังไม่มีท่าจับ")
            elif cmd == "l":
                print(node.report())
            elif cmd == "w":
                print(node.write())
            else:
                print("  Enter แตะ · u / d <n> ลบท่าแตะ · g ท่าจับ · gu ลบท่าจับล่าสุด · l ดู · w บันทึก · q ออก")
    except (KeyboardInterrupt, EOFError):
        print()
    finally:  # หยุดตัวรับข้อมูล (อีกเธรด) ให้เสร็จก่อนทำลายโหนด — ไม่งั้นมีข้อความ "exception was never retrieved"
        ex.shutdown(timeout_sec=2.0)
        spin.join(timeout=2.0)
        ex.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
