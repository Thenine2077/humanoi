#!/usr/bin/env python3
"""tray_pick — หยิบกล่องตามที่กล้องเห็น ด้วยท่าจับที่สอนไว้ (tray_teach แบบทางลัด -> config/tray_grasp_<arm>.yaml)

  arm start left · arm player (คุมมือ) · arm tray (กล้อง + หากล่อง) · arm tray-pick

หยิบ: นิ้ว tray_grip0 -> ข้อมือไปรอเหนือที่จับ approach (ข้อต่อวิ่งจากท่าปัจจุบัน — แขนต้องอยู่สูงกว่ากล่อง)
      -> ลงเป็นเส้นตรงตั้งฉากพื้นกล่อง -> นิ้ว tray_grip -> ยกขึ้นเส้นตรง lift แล้วค้างไว้
วางคืน (r): ลงเส้นตรงที่เดิม -> นิ้ว tray_grip0 -> ขึ้นไปจุดรอ
ที่จับ = ท่าจับที่สอน (ตัวที่กล่องวางใกล้สุด) เลื่อนตามกล่อง (tray.delta_grasp) · IK จากโมเดลแขนเอง (ไม่ต้องเปิด MoveIt)
กล่องห่างท่าที่สอนเกิน max_shift_m หรือหมุนเกิน max_turn_deg (ในไฟล์ท่าจับ) = ไม่หยิบ

คำสั่ง: Enter = ดูแผน (ยังไม่ขยับ) แล้วถามยืนยัน · r = วางคืน · h <cm> = ยกที่จับขึ้น/ลง (เก็บในไฟล์) · q = ออก
       ระหว่างแขนขยับ Ctrl+C = หยุดทันที (ยกเลิกคำสั่งที่ตัวคุมแขน)
"""
import math
import os
import re
import sys
import threading
import time

import numpy as np
import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

from humanoi_vision import arm_chain, hand_eye, tray

CONFIG = "/humanoi_ws/src/humanoi_description/config"
BOX_WINDOW = 1.0   # s — เก็บตำแหน่งป้ายช่วงนี้ ต้องนิ่ง (แกว่งไม่เกิน BOX_STILL_MM)
BOX_STILL_MM = 3.0
LINE_STEP = 0.005   # m ระยะระหว่างจุด IK บนเส้นตรง
MIN_CLEAR = 0.04    # m จุดรอ/ยก ต้องสูงจากที่จับอย่างน้อยเท่านี้


class PickError(Exception):
    pass


def pose_mat(p):
    return tray.mat(hand_eye.quat_to_mat(*p["quat"]), p["xyz"])


def msg_mat(msg):
    q, p = msg.pose.orientation, msg.pose.position
    return tray.mat(hand_eye.quat_to_mat(q.x, q.y, q.z, q.w), [p.x, p.y, p.z])


def _duration(secs):
    return Duration(sec=int(secs), nanosec=int((secs - int(secs)) * 1e9))


class TrayPick(Node):
    def __init__(self):
        super().__init__("tray_pick")
        self.declare_parameter("arm", "left")
        self.declare_parameter("grasp_file", "")
        self.declare_parameter("approach", 0.08)      # m เหนือที่จับ (ตั้งฉากพื้นกล่อง) ก่อนลง
        self.declare_parameter("lift", 0.10)          # m ยกขึ้นหลังจับ
        self.declare_parameter("speed", 0.15)         # rad/s ข้อต่อวิ่งไปจุดรอ
        self.declare_parameter("line_speed", 0.06)    # rad/s ช่วงลง/ยกเส้นตรง (ช้า)
        self.declare_parameter("accel", 0.08)         # rad/s^2 (สายพาน J1 เคยรูดตอนเร่งแรงขณะแขนยื่นไกล)
        self.declare_parameter("hand_open", "tray_grip0")
        self.declare_parameter("hand_close", "tray_grip")
        self.declare_parameter("hand_wait", 1.5)      # s รอนิ้วถึงท่าหลังมือรับคำสั่ง
        self.declare_parameter("hand_topic", "/hand/gesture")
        self.p = {n: self.get_parameter(n).value for n in
                  ("arm", "grasp_file", "approach", "lift", "speed", "line_speed", "accel", "hand_open", "hand_close",
                   "hand_wait", "hand_topic")}
        arm = self.p["arm"]
        self.base, self.tool = f"{arm}_base_link", f"{arm}_link_6"
        self.grasp_file = self.p["grasp_file"] or os.path.join(CONFIG, f"tray_grasp_{arm}.yaml")
        self.lock = threading.Lock()
        self.box, self.status, self.joints, self.urdf = [], "-", None, None
        self.hand_status, self.arm_state = [], "idle"
        self.chain, self.holding = None, None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PoseStamped, "/tray/pose", self._on_box, 10)
        self.create_subscription(String, "/tray/status", lambda m: setattr(self, "status", m.data), 10)
        self.create_subscription(JointState, "/joint_states", self._on_js, 50)
        self.create_subscription(String, "/robot_description", lambda m: setattr(self, "urdf", m.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, "/hand/status", self._on_hand, 10)
        self.create_subscription(String, "/arm/state", lambda m: setattr(self, "arm_state", m.data), 10)
        self.hand_pub = self.create_publisher(String, self.p["hand_topic"], 10)
        self.action = ActionClient(self, FollowJointTrajectory, f"/{arm}_arm_controller/follow_joint_trajectory")
        self.goal = None

    def _on_box(self, msg):
        with self.lock:
            now = time.monotonic()
            self.box = [b for b in self.box if now - b[0] <= BOX_WINDOW] + [(now, msg)]

    def _on_js(self, msg):
        with self.lock:
            self.joints = dict(zip(msg.name, msg.position))

    def _on_hand(self, msg):
        with self.lock:
            self.hand_status.append((time.monotonic(), msg.data))

    # ---------- ข้อมูล ----------
    def load_grasps(self):
        if not os.path.exists(self.grasp_file):
            raise PickError(f"ไม่มี {self.grasp_file} — สอนท่าจับก่อน: arm tray-teach (g แล้ว w)")
        with open(self.grasp_file, encoding="utf-8") as f:
            g = yaml.safe_load(f)
        if g.get("method") != "delta" or not g.get("grasps"):
            raise PickError(f"{self.grasp_file} ไม่ใช่ไฟล์ท่าจับแบบทางลัด (method: delta) — สอนใหม่ด้วย arm tray-teach")
        return g

    def current_q(self):
        with self.lock:
            js = dict(self.joints or {})
        if not self.chain or any(j not in js for j in self.chain.joints):
            raise PickError("ไม่มี /joint_states ของแขนนี้ — เปิด arm start ก่อน")
        return np.array([js[j] for j in self.chain.joints])

    def ensure_chain(self, wait=5.0):
        """โมเดลแขนจาก /robot_description + เช็กว่า FK ตรงกับ TF ที่ระบบแขนส่งอยู่"""
        if self.chain is None:
            t_end = time.monotonic() + wait
            while self.urdf is None and time.monotonic() < t_end:
                time.sleep(0.1)
            if self.urdf is None:
                raise PickError("ไม่ได้ /robot_description — เปิด arm start ก่อน")
            self.chain = arm_chain.Chain(self.urdf, self.base, self.tool)
        q = self.current_q()
        try:
            tf = self.tf_buffer.lookup_transform(self.base, self.tool, Time())
        except TransformException as exc:
            raise PickError(f"ไม่มี TF {self.base} -> {self.tool} ({exc})")
        r, t = tf.transform.rotation, tf.transform.translation
        T = tray.mat(hand_eye.quat_to_mat(r.x, r.y, r.z, r.w), [t.x, t.y, t.z])
        pos, rot = self.chain.pose_error(self.chain.fk(q), T)
        if pos > 0.003 or rot > math.radians(1):
            raise PickError(f"โมเดลแขนไม่ตรงกับ TF ({1000 * pos:.1f} mm) — แขนกำลังขยับ หรือเปิด arm start ใหม่แล้ว")

    def box_pose(self, frame):
        t_end = time.monotonic() + BOX_WINDOW + 0.5
        while True:
            with self.lock:
                msgs = [m for t, m in self.box if time.monotonic() - t <= BOX_WINDOW]
            if self.status == "ok" and len(msgs) >= 3:
                break
            if time.monotonic() > t_end:
                raise PickError(f"ไม่เห็นกล่องนิ่ง (สถานะ {self.status}) — arm tray เปิดอยู่ไหม กล้องเห็นป้ายทั้งแผ่นไหม")
            time.sleep(0.1)
        if msgs[-1].header.frame_id != frame:
            raise PickError(f"ป้ายมาในเฟรม {msgs[-1].header.frame_id} แต่ท่าจับสอนในเฟรม {frame}")
        Cs = [msg_mat(m) for m in msgs]
        spread = 1000 * max(np.linalg.norm(C[:3, 3] - Cs[-1][:3, 3]) for C in Cs)
        if spread > BOX_STILL_MM:
            raise PickError(f"กล่องยังไม่นิ่ง (ป้ายแกว่ง {spread:.1f} mm) — รอให้นิ่งแล้วกดใหม่")
        return Cs[-1]

    # ---------- วางแผน ----------
    def plan(self):
        g = self.load_grasps()
        self.ensure_chain()
        C = self.box_pose(g["camera_frame"])
        R = hand_eye.quat_to_mat(*g["camera_rotation"])
        Cg = [pose_mat(x["marker_in_camera"]) for x in g["grasps"]]
        Ag = [pose_mat(x["wrist_in_arm"]) for x in g["grasps"]]
        names = g["joint_names"]
        seeds = [np.array([x["joints"][names.index(j)] for j in self.chain.joints]) for x in g["grasps"]]
        A, k = tray.nearest_grasp(Cg, Ag, C)
        T, shift, turn = tray.delta_grasp(R, A, Cg[k], C)
        info = f"กล่องห่างท่าจับที่ {k + 1} ที่สอน {1000 * shift:.0f} mm หมุน {turn:+.1f}°"
        if shift > float(g["max_shift_m"]):
            raise PickError(f"{info} — เกิน {1000 * float(g['max_shift_m']):.0f} mm วางกล่องใกล้จุดที่สอน "
                            "หรือสอนท่าจับเพิ่มที่ตำแหน่งนี้ (arm tray-teach g)")
        if abs(turn) > float(g["max_turn_deg"]):
            raise PickError(f"{info} — หมุนเกิน {float(g['max_turn_deg']):.0f}° (วิธีนี้ชดเชยการหมุนไม่ได้) วางกล่องให้ตรง")
        up = R @ C[:3, 2]
        up /= np.linalg.norm(up)
        off = float(g.get("grasp_offset_up", 0.0))  # ปรับความสูงที่จับจากท่าที่สอน (h <cm>)
        if off:
            T[:3, 3] += off * up
            info += f" · ยกที่จับ {100 * off:+.1f} cm"
        q_T, pos, rot = self.chain.ik(T, seeds[k])
        if pos > 1e-3 or rot > math.radians(0.5):
            raise PickError(f"{info} — แขนไปที่จับไม่ถึง (คลาด {1000 * pos:.0f} mm)")
        # เส้นตรงจากที่จับขึ้นไป — ใช้ทั้งเป็นทางลง (กลับด้าน) และทางยก · ข้อมือคงมุมเดิมยิ่งสูงข้อ 5 ยิ่งพับ
        # จนชนลิมิต (สวิตช์) -> ยก/รอได้เท่าที่เส้นตรงไปถึง ไม่ต่ำกว่า MIN_CLEAR
        H = max(float(self.p["approach"]), float(self.p["lift"]))
        n = max(1, int(round(H / LINE_STEP)))
        top = T.copy()
        top[:3, 3] += H * up
        path = self.chain.line(q_T, T, top, step_m=H / n, partial=True)
        reach = H * len(path) / n
        if reach < MIN_CLEAR:
            raise PickError(f"{info} — จากที่จับยกตรงขึ้นได้แค่ {100 * reach:.0f} cm ({self.chain.line_stop}) "
                            "· ท่าจับที่สอนพับข้อมือเกือบสุดแล้ว — วางกล่องใกล้จุดที่สอน หรือสอนท่าจับใหม่")
        i_h = max(1, int(round(min(float(self.p["approach"]), reach) / H * n)))
        i_l = max(1, int(round(min(float(self.p["lift"]), reach) / H * n)))
        q_hover = path[i_h - 1]
        hover, lift = T.copy(), T.copy()
        hover[:3, 3] += H * i_h / n * up
        lift[:3, 3] += H * i_l / n * up
        down = path[:i_h - 1][::-1] + [q_T]
        up_path = path[:i_l]
        short = ("" if H * i_l / n >= float(self.p["lift"]) - 1e-6
                 else f" (ตั้งไว้ {100 * float(self.p['lift']):.0f} cm แต่สูงกว่านี้ไม่ได้: {self.chain.line_stop})")
        q_now = self.current_q()
        T_now = self.chain.fk(q_now)
        height = float((T_now[:3, 3] - T[:3, 3]) @ up)
        if height < 0.5 * H * i_h / n:
            raise PickError(f"ข้อมือตอนนี้สูงกว่าที่จับแค่ {100 * height:.0f} cm — ยกแขนให้สูงกว่ากล่องก่อน "
                            "(ช่วงไปจุดรอข้อต่อวิ่งตรงจากท่าปัจจุบัน อาจกวาดโดนกล่อง)")
        return {"info": info, "T": T, "hover": hover, "lift": lift, "q_now": q_now, "q_hover": q_hover,
                "down": down, "up": up_path, "height": height, "h_hover": H * i_h / n, "h_lift": H * i_l / n,
                "short": short}

    def set_offset(self, cm):
        """ยกที่จับขึ้น cm (ลบ = ลง) ตั้งฉากพื้นกล่อง จากท่าที่สอน — เก็บในไฟล์ท่าจับ (grasp_offset_up)"""
        if abs(cm) > 3.0:
            raise PickError("ปรับได้ไม่เกิน ±3 cm — ต่างมากกว่านี้ให้สอนท่าจับใหม่ (arm tray-teach)")
        self.load_grasps()
        with open(self.grasp_file, encoding="utf-8") as f:
            text = f.read()
        line = f"grasp_offset_up: {cm / 100:.4f}   # m ยกที่จับขึ้นจากท่าที่สอน ตั้งฉากพื้นกล่อง (arm tray-pick: h <cm>)"
        if re.search(r"^grasp_offset_up:.*$", text, flags=re.M):
            text = re.sub(r"^grasp_offset_up:.*$", lambda _: line, text, flags=re.M)
        else:
            text = text.replace("\ngrasps:\n", f"\n{line}\ngrasps:\n", 1)
        tmp = self.grasp_file + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            f.write(text)
        os.replace(tmp, self.grasp_file)

    def describe(self, pl):
        go = arm_chain.timed_path([pl["q_now"], pl["q_hover"]], float(self.p["speed"]), float(self.p["accel"]))
        dn = arm_chain.timed_path([pl["q_hover"]] + pl["down"], float(self.p["line_speed"]), float(self.p["accel"]))
        lf = arm_chain.timed_path([pl["down"][-1]] + pl["up"], float(self.p["line_speed"]), float(self.p["accel"]))
        move = np.degrees(np.abs(pl["q_hover"] - pl["q_now"]))
        return (f"  {pl['info']}\n"
                f"  1. นิ้ว {self.p['hand_open']}\n"
                f"  2. ไปรอเหนือที่จับ {100 * pl['h_hover']:.1f} cm: ข้อต่อขยับมากสุด {move.max():.0f}° "
                f"(ข้อ {int(np.argmax(move)) + 1}) {go[-1][0]:.1f} s — ตอนนี้ข้อมือสูงกว่าที่จับ {100 * pl['height']:.0f} cm\n"
                f"  3. ลงเส้นตรง {dn[-1][0]:.1f} s\n"
                f"  4. นิ้ว {self.p['hand_close']}\n"
                f"  5. ยกขึ้นเส้นตรง {100 * pl['h_lift']:.1f} cm {lf[-1][0]:.1f} s แล้วค้างไว้{pl['short']}")

    # ---------- ขยับ ----------
    def check_ready(self):
        if self.arm_state.startswith("running"):
            raise PickError(f"ตัวเล่นโปรแกรมกำลังเล่นอยู่ ({self.arm_state}) — รอจบหรือ arm stop")
        if self.count_subscribers(self.p["hand_topic"]) == 0:
            raise PickError(f"ไม่มีตัวคุมมือฟัง {self.p['hand_topic']} — เปิด arm player ก่อน")
        if not self.action.wait_for_server(timeout_sec=2.0):
            raise PickError(f"ไม่พบตัวคุมแขน /{self.p['arm']}_arm_controller — เปิด arm start ก่อน")

    def hand(self, name):
        with self.lock:
            self.hand_status.clear()
        self.hand_pub.publish(String(data=name))
        t_end = time.monotonic() + 3.0
        while time.monotonic() < t_end:
            with self.lock:
                got = [s for _, s in self.hand_status]
            if any(s.startswith("error") for s in got):
                raise PickError(f"มือ: {[s for s in got if s.startswith('error')][-1]}")
            if f"gesture:{name}" in got:
                time.sleep(float(self.p["hand_wait"]))
                return
            time.sleep(0.05)
        raise PickError(f"มือไม่ตอบรับท่า {name} ใน 3 วินาที — ตัวคุมมือ (arm player) ยังทำงานอยู่ไหม")

    def move(self, waypoints, speed, label):
        """waypoints[0] = ท่าปัจจุบัน — ส่งแผนเวลาให้ตัวคุมแขน แล้วรอจนถึง (Ctrl+C = ยกเลิก)"""
        q_now = self.current_q()
        if np.max(np.abs(q_now - waypoints[0])) > math.radians(1.0):
            raise PickError(f"{label}: แขนไม่ได้อยู่ที่จุดเริ่มของแผน (ห่าง "
                            f"{math.degrees(np.max(np.abs(q_now - waypoints[0]))):.1f}°) — มีอะไรสั่งแขนแทรก?")
        pts = arm_chain.timed_path([q_now] + list(waypoints[1:]), speed, float(self.p["accel"]))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.chain.joints)
        for t, pos, vel in pts:
            goal.trajectory.points.append(JointTrajectoryPoint(positions=pos, velocities=vel,
                                                               time_from_start=_duration(t)))
        print(f"  → {label} ({pts[-1][0]:.1f} s)", flush=True)
        fut = self.action.send_goal_async(goal)
        if not _wait(fut, 5.0) or not fut.result().accepted:
            raise PickError(f"{label}: ตัวคุมแขนไม่รับคำสั่ง")
        self.goal = fut.result()
        try:
            res = self.goal.get_result_async()
            if not _wait(res, pts[-1][0] + 15.0):
                self.goal.cancel_goal_async()
                raise PickError(f"{label}: แขนไม่ถึงในเวลา — ยกเลิกแล้ว")
            code = res.result().result.error_code
            if code != FollowJointTrajectory.Result.SUCCESSFUL:
                raise PickError(f"{label}: ตัวคุมแขนรายงาน error {code} {res.result().result.error_string}")
        except KeyboardInterrupt:
            stopped = _wait(self.goal.cancel_goal_async(), 2.0)
            raise PickError(f"{label}: หยุดแล้ว (Ctrl+C) — แขนค้างอยู่ตรงนั้น" if stopped else
                            f"{label}: ส่งคำสั่งหยุดไม่ทัน — ปิดไฟแขน/arm off ถ้าแขนยังวิ่งอยู่")
        finally:
            self.goal = None

    def pick(self, pl):
        self.check_ready()
        self.hand(self.p["hand_open"])
        self.move([pl["q_now"], pl["q_hover"]], float(self.p["speed"]), "ไปรอเหนือที่จับ")
        self.move([pl["q_hover"]] + pl["down"], float(self.p["line_speed"]), "ลงไปที่จับ")
        self.hand(self.p["hand_close"])
        self.holding = pl
        self.move([pl["down"][-1]] + pl["up"], float(self.p["line_speed"]), "ยกขึ้น")

    def place_back(self):
        pl = self.holding
        self.check_ready()
        self.ensure_chain()
        q = self.current_q()
        try:
            down = self.chain.line(q, self.chain.fk(q), pl["T"])
            up = self.chain.line(down[-1], pl["T"], pl["hover"])
        except ValueError as exc:
            raise PickError(f"วางคืนไม่ได้ — {exc}")
        self.move([q] + down, float(self.p["line_speed"]), "ลงวางที่เดิม")
        self.hand(self.p["hand_open"])
        self.holding = None
        self.move([down[-1]] + up, float(self.p["line_speed"]), "ขึ้นไปจุดรอ")


def _wait(fut, timeout):
    t_end = time.monotonic() + timeout
    while not fut.done():
        if time.monotonic() > t_end:
            return False
        time.sleep(0.02)
    return True


def main():
    # Ctrl+C ต้องเป็น KeyboardInterrupt ธรรมดา — ตัวจัดการของ rclpy ปิด ROS ทันที คำสั่งหยุดแขนจะส่งไม่ออก
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TrayPick()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    print(f"หยิบกล่องตามกล้อง แขน{node.p['arm']} — ท่าจับจาก {node.grasp_file}\n"
          "Enter = ดูแผน (ยังไม่ขยับ) แล้วยืนยัน · r = วางคืนที่เดิม · h <cm> = ยกที่จับ · q = ออก · "
          "Ctrl+C ระหว่างขยับ = หยุด")
    try:
        while True:
            cmd = input("[ถือกล่องอยู่] > " if node.holding else "> ").strip().lower()
            if cmd == "q":
                break
            try:
                if cmd == "":
                    if node.holding:
                        print("  ถือกล่องอยู่ — วางคืนก่อน (r)")
                        continue
                    pl = node.plan()
                    print(node.describe(pl))
                    if input("  หยิบเลย? พิมพ์ y แล้ว Enter (อย่างอื่น = ยกเลิก) ").strip().lower() == "y":
                        node.pick(pl)
                        print("✓ หยิบแล้ว — ค้างไว้ · r = วางคืน")
                elif cmd == "h" or cmd.startswith("h "):
                    if cmd == "h":
                        off = float(node.load_grasps().get("grasp_offset_up", 0.0))
                        print(f"  ยกที่จับจากท่าที่สอน {100 * off:+.1f} cm · ตั้งใหม่: h <cm> เช่น h 0.5 (ลบ = ลง)")
                        continue
                    try:
                        cm = float(cmd[2:])
                    except ValueError:
                        print("  h ตามด้วยตัวเลข cm เช่น h 0.5")
                        continue
                    node.set_offset(cm)
                    print(f"✓ ยกที่จับ {cm:+.1f} cm จากท่าที่สอน (บันทึกในไฟล์ท่าจับแล้ว — มีผลตั้งแต่ครั้งหยิบถัดไป)")
                elif cmd == "r":
                    if not node.holding:
                        print("  ยังไม่ได้ถือกล่อง")
                        continue
                    if input("  วางคืนที่เดิม? y + Enter ").strip().lower() == "y":
                        node.place_back()
                        print("✓ วางคืนแล้ว")
                else:
                    print("  Enter = ดูแผนแล้วหยิบ · r = วางคืน · h <cm> = ยกที่จับ (h = ดูค่า) · q = ออก")
            except PickError as exc:
                print(f"✗ {exc}")
    except (KeyboardInterrupt, EOFError):
        print()
        if node.goal is not None:
            node.goal.cancel_goal_async()
    finally:
        ex.shutdown(timeout_sec=2.0)
        spin.join(timeout=2.0)
        ex.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
