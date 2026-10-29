#!/usr/bin/env python3
"""rps_game — เป่ายิ้งฉุบกับหุ่น

ลำดับ: แขนไปท่าตั้ง (rps_ready — สอนเองด้วย arm teach) มือกำหลวมๆ → ข้อ 1 ยกขึ้น-ลง 3 ครั้ง (เป่า-ยิ้ง-ฉุบ)
→ จังหวะ "ฉุบ" มือออกท่าที่สุ่มไว้ → ดูท่าผู้เล่นจากกล้อง → แขนไปท่าโชว์ (ถ้าสอนไว้) แล้วมือค่อยทำท่า:
ผู้เล่นชนะ = แขนไป victory_pose + ชูนิ้วโป้ง (ไลค์), ผู้เล่นแพ้ = แขนไป lose_pose + ส่ายนิ้ว (nonono),
เสมอ = เป่าใหม่ (ไม่เกิน max_draws ครั้ง), ไม่เห็นมือ = ส่ายนิ้ว
โชว์เสร็จแขนกลับท่าตั้งผ่านท่า return_moves (move_readyP1 → move_readyP2 → rps_ready) — ไม่ไปตรงๆ เพราะข้อ 1
รับน้ำหนักการเหวี่ยงแขนทั้งแขนตรงๆ ไม่ไหว (ตอนเริ่มเกม ถ้าแขนอยู่ไกลจากท่าตั้งก็เดินทางนี้ก่อนเหมือนกัน)
จบเกมแล้วผู้เล่นชูนิ้วชี้ค้างไว้ (replay_hold วินาที) = เล่นตาใหม่ วนไปเรื่อยๆ
(รอนิ้วชี้ได้ replay_timeout วินาทีหลังจบเกม)

  ros2 launch humanoi_games rps.launch.py                     # เปิดเกมค้างไว้ (แขน + มือ + กล้อง)
  ros2 topic pub --once /rps/play std_msgs/msg/Empty "{}"     # เล่น 1 ตา
  ros2 topic echo /rps/result

วิธีเรียกเล่นยังไม่ได้เลือก — อะไรก็ได้ที่ส่ง /rps/play ได้ (สวิตช์รีโมท, ปุ่ม, เสียง, หน้าเว็บ)

หัวข้อ
  /rps/play            (sub, Empty)   เริ่ม 1 ตา (ระหว่างเล่นอยู่ไม่รับซ้ำ)
  /rps/stop            (sub, Empty)   หยุดกลางคัน แขนหยุดที่ตำแหน่งปัจจุบัน
  /rps/player_gesture  (sub, String)  ท่าผู้เล่นทีละเฟรม rock|paper|scissors|point|none (rps_vision หรือพิมพ์ส่งเอง)
  /rps/state           (pub, String)  idle | ready | pump:<ตาที่>:<วินาทีของแต่ละจังหวะ,...> (แขนเริ่มเป่า)
                                      | beat:1..3 | reveal:<ท่าหุ่น> | result:<ผล>:<ท่าหุ่น>:<ท่าผู้เล่น|none>
                                      | pose:<ท่าโชว์> | return:<ท่า> | waiting_replay:<วินาทีที่รอ, 0 = ไม่จำกัด>
                                      — จอ rps_hud ใช้ (นับถอยหลังตามจังหวะแขน)
  /rps/result          (pub, String)  ผลสุดท้ายของเกม (เสมอแล้วเป่าใหม่ = ไม่ส่ง รอผลตาสุดท้าย)
                                      "<ท่าหุ่น>:<ท่าผู้เล่น|none>:<ผล>"
                                      ผล = player_win | robot_win | draw | no_player | stopped | error:<เหตุผล>
  /hand/gesture        (pub, String)  สั่งมือ (amazinghand_node)

แขนสั่งตรงเข้า <arm>_arm_controller แบบเดียวกับ ar_program (เร่ง → วิ่ง → เบรก ไม่ผ่าน MoveIt ไม่เช็คการชน)
อย่าเปิด arm player สั่งแขนเดียวกันพร้อมกับเกม · หยุดฉุกเฉิน = ปิดสวิตช์ไฟแขน
"""
import functools
import math
import random
import threading
import time
from collections import deque

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

from ar_program import programs
from humanoi_games import rps_logic
from humanoi_games.amazinghand_node import topic_prefix
from humanoi_games.rps_logic import THAI

ARM_TH = {"left": "ซ้าย", "right": "ขวา"}
RESULT_TH = {"player_win": "ผู้เล่นชนะ → ชูนิ้วโป้ง", "robot_win": "ผู้เล่นแพ้ → ส่ายนิ้ว",
             "draw": "เสมอ", "no_player": "ไม่เห็นมือผู้เล่น → ส่ายนิ้ว"}
# ผล (มุมผู้เล่น) -> (ท่ามือ, พารามิเตอร์ชื่อท่าแขนตอนโชว์ — ไม่ได้สอนท่านั้นไว้ = แขนค้างที่ท่าตั้ง)
REACTION = {"player_win": ("like", "player_win_moves"), "robot_win": ("nonono", "robot_win_moves"),
            "draw": ("paper", None), "no_player": ("nonono", None)}
PARAMS = {
    "arm": "left",                # แขนที่ติดมือ
    "layout": "dual",             # dual = ar_dual (<arm>_joint_N) · single = แขนเดี่ยวแบบเดิม (joint_N)
    "programs_file": programs.DEFAULT_FILE,
    "ready_pose": "rps_ready",    # ท่าตั้งก่อนเป่า (ต้องสอนก่อน: arm teach left rps_ready)
    # ท่าแขนตอนโชว์ = ชื่อท่าเรียงกัน (คั่นด้วยเว้นวรรค) แขนไปทีละท่าจนครบ แล้วมือค่อยทำท่า
    # ท่าที่ยังไม่ได้สอนข้ามไป (ไม่มีสักท่า = แขนค้างที่ท่าตั้ง) — สอน: arm teach left victory_pose / lose_pose
    "player_win_moves": "victory_pose",  # ผู้เล่นชนะ (ชูโป้ง)
    "robot_win_moves": "lose_pose",      # ผู้เล่นแพ้ (ส่ายนิ้ว)
    "show_speed": 0.19,           # ท่าโชว์ที่ใกล้กัน (ไม่เกิน show_fast_max_deg) ขยับด้วยเพดาน (เท่า J1 ตอน calibrate):
    "show_accel": 0.17,           # 10.9°/s, 9.7°/s^2 (เฟิร์มแวร์ 10°/s^2) — แกว่ง 39° ครั้งละ ~4.7 s
    "show_fast_max_deg": 45.0,    # ท่าที่ห่างกว่านี้ (เช่นจากท่าตั้ง) ใช้ move_speed/move_accel นุ่มๆ แทน
    "return_moves": "move_readyP1 move_readyP2",  # ท่าที่เดินผ่านก่อนเข้าท่าตั้ง (คั่นเว้นวรรค) — ข้อ 1 ไม่ต้องรับการเหวี่ยงตรงๆ
    "near_deg": 5.0,              # ห่างท่าตั้งไม่เกินกี่องศา (ทุกข้อ) ถึงไปตรงๆ ได้โดยไม่ต้องเดินผ่าน return_moves
    "pump_joint": 1,              # ข้อที่ขยับขึ้นลง
    "pump_deg": 5.0,              # ยกกี่องศา (8° = 11.6 วินาที, 5° = 8.6, 4° = 7.7, 3° = 6.7 — มือยกราว 0.6 ซม./องศา)
    "auto_up": True,              # true = เลือกทิศข้อ 1 ให้มือ "ขึ้น" เอง (คำนวณจาก TF ที่ท่าตั้ง) ·
                                  # false = ใช้เครื่องหมายของ pump_deg ตรงๆ
    "up_frame": "torso",          # เฟรมที่แกน z ชี้ขึ้นฟ้า
    "pump_count": 3,              # เป่า-ยิ้ง-ฉุบ
    "pump_speed": 0.19,           # rad/s (เพดาน 0.19 เหมือน ar_program — บอร์ดจำกัด J1 ไว้ 11.25°/s เกินนี้ตามไม่ทัน)
    "pump_accel": 0.17,           # rad/s^2 — เพดานคือเฟิร์มแวร์ (JOINT_MAX_ACCEL 10°/s^2 = 0.1745) สั่งเกินก็ไม่เร็วขึ้น
                                  # แขนแค่ตามหลัง · เร็วกว่านี้ต้องแก้เฟิร์มแวร์ (สายพาน J1 เคยรูด)
    "move_speed": 0.19,           # ไป/กลับท่าตั้งและท่าโชว์ — ค่าเดียวกับ arm player
    "move_accel": 0.08,
    "hand_lead": 0.4,             # สั่งมือออกท่าก่อนแขนลงสุดกี่วินาที (นิ้วใช้เวลาราว 0.3 s)
    "robot_choice": "random",     # random | rock | paper | scissors (บังคับท่าหุ่น ไว้ทดสอบ/โชว์)
    "player_source": "camera",    # camera = โหวตจาก /rps/player_gesture · random = สุ่มแทนผู้เล่น (ทดสอบ)
    "settle": 0.2,                # หลัง "ฉุบ" รอให้มือผู้เล่นนิ่งก่อนเริ่มนับ (วินาที)
    "player_window": 1.2,         # นับท่าผู้เล่นนานเท่าไหร่ (กล้อง ~12 fps = ~14 เฟรม)
    "min_votes": 4,               # ต้องเห็นมืออย่างน้อยกี่เฟรมในช่วงนั้น
    "max_draws": 2,               # เสมอแล้วเป่าใหม่ได้กี่ครั้ง
    "react_hold": 3.0,            # ค้างท่าโชว์กี่วินาที
    "end_gesture": "paper",       # ท่ามือตอนจบเกม
    "windup_gesture": "windup",   # ท่ามือตอนง้าง (เป่า-ยิ้ง) — ต่างจากค้อน (กำสุด) ให้เห็นชัดว่าออกท่าแล้ว
    "start_delay": 1.0,           # ถึงท่าตั้งแล้วรอกี่วินาทีก่อนเริ่มเป่า (ให้ผู้เล่นเตรียมมือ)
    "replay_gesture": "point",    # ท่าผู้เล่นหลังจบเกม = เล่นใหม่ (ชูนิ้วชี้ 1 นิ้ว)
    "replay_hold": 0.8,           # ต้องค้างท่านั้นกี่วินาที (กันมือที่กำลังลดลงผ่านท่าชี้แวบเดียว)
    "replay_timeout": 60.0,       # รอนิ้วชี้หลังจบเกมได้กี่วินาที (0 = รอตลอด) — หมดเวลาต้องสั่ง /rps/play ใหม่
}
MAX_SPEED = 0.19  # = ar_program max_speed · บอร์ดจำกัด J1 11.25°/s (ar_dual/config/joint_speed.yaml)


class GameError(Exception):
    pass


class Stopped(Exception):
    pass


def _wait(future, timeout):
    end = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > end:
            return None
        time.sleep(0.01)
    return future.result()


def _duration(secs):
    sec = int(secs)
    return Duration(sec=sec, nanosec=int(round((secs - sec) * 1e9)))


class RpsGame(Node):
    def __init__(self):
        super().__init__("rps_game")
        any_type = ParameterDescriptor(dynamic_typing=True)  # pump_deg:=8 (int) ก็รับ — ค่าถูกแปลงตอนใช้
        for name, default in PARAMS.items():
            self.declare_parameter(name, default, any_type)
        self.arm = programs.arm_of(self.get_parameter("arm").value)
        layout = self.get_parameter("layout").value
        self.joints = programs.joint_names(self.arm, layout)
        action = (f"/{self.arm}_arm_controller/follow_joint_trajectory" if layout == "dual"
                  else "/joint_trajectory_controller/follow_joint_trajectory")
        cb = ReentrantCallbackGroup()
        self.client = ActionClient(self, FollowJointTrajectory, action, callback_group=cb)
        self.current = {}
        self.samples = deque(maxlen=600)  # (เวลา, ท่าผู้เล่น)
        self.busy = False
        self.lock = threading.Lock()
        self.stop_flag = threading.Event()
        self.goal_handle = None
        self.replay_until = 0.0  # เวลา (monotonic) ที่ยังรับนิ้วชี้ = เล่นใหม่ · 0 = ไม่รับ
        self.create_timer(0.5, self._check_replay_timeout, callback_group=cb)
        self.create_subscription(JointState, "/joint_states", self._on_js, 10, callback_group=cb)
        self.create_subscription(Empty, "/rps/play", self._on_play, 10, callback_group=cb)
        self.create_subscription(Empty, "/rps/stop", self._on_stop, 10, callback_group=cb)
        self.create_subscription(String, "/rps/player_gesture", self._on_player, 50, callback_group=cb)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.hand_pub = self.create_publisher(String, f"{topic_prefix(self.arm)}/gesture", 10)  # มือบนแขนที่เล่น
        self.state_pub = self.create_publisher(String, "/rps/state", 10)
        self.result_pub = self.create_publisher(String, "/rps/result", 10)
        self._state("idle")
        p = self._params()
        stroke = rps_logic.stroke_time(math.radians(float(p["pump_deg"])),
                                       min(float(p["pump_speed"]), MAX_SPEED), float(p["pump_accel"]))
        self.get_logger().info(
            f"rps_game พร้อม — แขน{ARM_TH[self.arm]} ({action}) ท่าตั้ง '{p['ready_pose']}' · "
            f"เป่า {p['pump_count']} ครั้ง ข้อ {p['pump_joint']} {float(p['pump_deg']):+.0f}° "
            f"(~{2 * stroke * p['pump_count']:.0f} วินาที) · ผู้เล่นจาก {p['player_source']} · "
            "เล่น: ros2 topic pub --once /rps/play std_msgs/msg/Empty \"{}\"")

    # ---------- callbacks ----------
    def _params(self):
        return {name: self.get_parameter(name).value for name in PARAMS}

    def _on_js(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.current[name] = pos

    def _on_player(self, msg):
        now = time.monotonic()
        self.samples.append((now, msg.data.strip()))
        if self.busy or now > self.replay_until:
            return
        hold = float(self.get_parameter("replay_hold").value)
        if rps_logic.held(list(self.samples), now - hold, now, self.get_parameter("replay_gesture").value):
            self.get_logger().info("ผู้เล่นชูนิ้วชี้ — เล่นตาใหม่")
            self._start()

    def _on_play(self, _msg):
        self._start()

    def _start(self):
        with self.lock:
            if self.busy:
                self.get_logger().warn("กำลังเล่นอยู่ — ไม่รับคำสั่งเล่นซ้ำ")
                return
            self.busy = True
            self.replay_until = 0.0
            self.stop_flag.clear()
        threading.Thread(target=self._run, daemon=True).start()

    def _check_replay_timeout(self):
        if self.replay_until and not self.busy and time.monotonic() > self.replay_until:
            self.replay_until = 0.0
            self._state("idle")
            self.get_logger().info("หมดเวลารอนิ้วชี้ — เล่นต่อด้วย arm rps play")

    def _on_stop(self, _msg):
        if self.busy:
            self.get_logger().warn("ได้รับ /rps/stop — หยุดเกม")
            self.stop_flag.set()
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
        elif self.replay_until:
            self.replay_until = 0.0
            self._state("idle")
            self.get_logger().info("ได้รับ /rps/stop — เลิกรอนิ้วชี้")

    # ---------- เกม (worker thread) ----------
    def _run(self):
        robot = player = None
        replay = False
        timeout = 0.0
        try:
            try:
                robot, player, result = self._play()
            except Stopped:
                result = "stopped"
            except (GameError, programs.ProgramError) as exc:
                result = f"error:{exc}"
            except Exception as exc:  # ไม่ให้ thread ตายเงียบแล้ว busy ค้าง
                result = f"error:{type(exc).__name__}: {exc}"
            self.result_pub.publish(String(data=f"{robot}:{player or 'none'}:{result}"))
            if result.startswith("error") or result == "stopped":
                self.get_logger().error(f"เกมจบ: {result}")
            else:
                replay = True
        finally:
            self.goal_handle = None
            with self.lock:
                self.busy = False
                if replay:  # จบเกมปกติ — รอนิ้วชี้ = เล่นตาใหม่
                    timeout = float(self.get_parameter("replay_timeout").value)
                    self.replay_until = time.monotonic() + timeout if timeout > 0 else math.inf
            if replay:
                self._state(f"waiting_replay:{timeout:.0f}")
                self.get_logger().info(
                    "ชูนิ้วชี้ค้างไว้ = เล่นตาใหม่" + (f" (รอ {timeout:.0f} วินาที)" if timeout > 0 else ""))
            else:
                self._state("idle")

    def _play(self):
        p = self._params()
        data = programs.load(p["programs_file"])
        ready = self._pose(data, p["ready_pose"], required=True)
        j = int(p["pump_joint"]) - 1
        amp = math.radians(float(p["pump_deg"]))
        for sign in (1, -1):  # ตรวจทั้งสองทิศก่อนขยับ (ทิศ "ขึ้น" รู้หลังแขนถึงท่าตั้ง)
            up = list(ready)
            up[j] += sign * abs(amp)
            programs.check_pose(f"ท่ายก ({p['ready_pose']} ข้อ {j + 1} {sign * abs(float(p['pump_deg'])):+.0f}°)", up, self.arm)
        if not self.client.wait_for_server(timeout_sec=3.0):
            raise GameError(f"ไม่พบตัวคุมแขน{ARM_TH[self.arm]} — เปิดระบบแขนก่อน (arm start)")
        if not all(n in self.current for n in self.joints):
            raise GameError(f"ยังไม่ได้รับ joint_states ของแขน{ARM_TH[self.arm]}")
        if not self.hand_pub.get_subscription_count():
            self.get_logger().warn("ไม่มีโหนดมือฟังอยู่ (amazinghand_node) — เล่นต่อแบบแขนอย่างเดียว")
        pump_speed = min(float(p["pump_speed"]), MAX_SPEED)
        pump_accel = float(p["pump_accel"])

        self._return_poses(data, p)  # ตรวจท่าทางกลับก่อนขยับอะไร (ท่าหาย = ไม่ขยับเลย)
        self._hand(p["windup_gesture"])
        self._state("ready")
        self._go_ready(data, ready, p)
        if p["auto_up"] and j == 0:
            amp = self._upward(abs(amp), p["up_frame"])
        self._sleep(float(p["start_delay"]))
        for rnd in range(1, int(p["max_draws"]) + 2):
            robot = p["robot_choice"] if p["robot_choice"] in rps_logic.CHOICES else rps_logic.robot_pick()
            self._hand(p["windup_gesture"])
            points, beats = rps_logic.pump_plan(ready, j, amp, int(p["pump_count"]), pump_speed, pump_accel,
                                                programs.trapezoid)
            events = [(0.0, functools.partial(self._state, f"pump:{rnd}:" + ",".join(f"{b:.2f}" for b in beats)))]
            events += [(t, functools.partial(self._state, f"beat:{i}")) for i, t in enumerate(beats, 1)]
            events.append((max(0.0, beats[-1] - float(p["hand_lead"])), functools.partial(self._reveal, robot)))
            self.get_logger().info(f"ตาที่ {rnd}: เป่า-ยิ้ง-ฉุบ ({beats[-1]:.1f} วินาที)")
            t0 = self._trajectory(points, events)
            player = self._read_player(t0 + beats[-1], p)
            result = rps_logic.judge(robot, player) if player else "no_player"
            self._state(f"result:{result}:{robot}:{player or 'none'}")
            self.get_logger().info(
                f"หุ่นออก {THAI[robot]} · ผู้เล่นออก {THAI[player]}"
                f"{' (สุ่มทดสอบ)' if p['player_source'] == 'random' else ''} → {RESULT_TH[result]}")
            if result == "draw" and rnd <= int(p["max_draws"]):  # /rps/result ส่งเฉพาะผลสุดท้าย
                self.get_logger().info("เสมอ — เป่าใหม่")
                self._sleep(1.0)
                continue
            self._react(result, data, ready, p)
            return robot, player, result

    def _pose(self, data, name, required=False):
        if not name or name not in data["poses"][self.arm]:
            if required:
                raise GameError(f"แขน{ARM_TH[self.arm]}ไม่มีท่า '{name}' — สอนก่อน: arm teach {self.arm} {name}")
            return None
        return programs.check_pose(f"{self.arm}:{name}", data["poses"][self.arm][name], self.arm)

    def _upward(self, amp, frame):
        """ทิศของข้อ 1 ที่ทำให้มือยกขึ้น ณ ท่าปัจจุบัน — แขนติดข้างเสา แกนข้อ 1 นอนขวาง ทิศขึ้นจึงขึ้นกับท่าตั้ง

        แกนข้อ 1 = แกน z ของเฟรม <arm>_link_1 (URDF: axis 0 0 1) ผ่านจุดกำเนิดของเฟรมนั้น
        ความเร็วมือเมื่อข้อ 1 หมุนบวก = แกน × (มือ − จุดบนแกน) — ดูแค่ส่วน z (ขึ้นฟ้า) ไม่ต้องขยับแขนทดลอง
        """
        time.sleep(0.3)  # ให้ TF ตามทันท่าตั้งที่เพิ่งไปถึง
        try:
            t1 = self.tf.lookup_transform(frame, f"{self.arm}_link_1", rclpy.time.Time())
            t6 = self.tf.lookup_transform(frame, f"{self.arm}_link_6", rclpy.time.Time())
        except Exception as exc:
            self.get_logger().warn(f"หาทิศขึ้นไม่ได้ (TF {frame}: {exc}) — ใช้ pump_deg ตามเครื่องหมายเดิม")
            return math.copysign(amp, float(self.get_parameter("pump_deg").value))
        q, o, h = t1.transform.rotation, t1.transform.translation, t6.transform.translation
        ax = (2 * (q.x * q.z + q.w * q.y), 2 * (q.y * q.z - q.w * q.x))  # x, y ของแกน z เฟรม link_1
        vz = ax[0] * (h.y - o.y) - ax[1] * (h.x - o.x)  # ส่วน z ของ แกน × (มือ − จุดบนแกน) ม./rad
        lift = abs(vz) * amp * 100
        self.get_logger().info(f"ยกขึ้น = ข้อ 1 {'+' if vz >= 0 else '-'}{math.degrees(amp):.0f}° "
                               f"(มือขึ้นราว {lift:.1f} ซม.)")
        if lift < 2.0:
            self.get_logger().warn("ท่าตั้งนี้ข้อ 1 พามือขึ้นลงได้น้อย (มือเกือบอยู่บนแนวแกนข้อ 1) "
                                   "— ลองสอนท่าตั้งให้มือยื่นไปข้างหน้ามากขึ้น")
        return amp if vz >= 0 else -amp

    def _read_player(self, shot, p):
        """ท่าผู้เล่นหลังจังหวะฉุบ (เวลา monotonic) — None = ไม่เห็นมือชัดพอ"""
        start = shot + float(p["settle"])
        if p["player_source"] == "random":
            self._sleep_until(start)
            return rps_logic.robot_pick()
        end = start + float(p["player_window"])
        self._sleep_until(end)
        return rps_logic.vote(list(self.samples), start, end, int(p["min_votes"]))

    def _react(self, result, data, ready, p):
        """แขนไปท่าโชว์ทีละท่าจนครบก่อน (ท่าที่สอนไว้) แล้วมือค่อยทำท่า ค้างไว้ แล้วกลับท่าตั้งผ่าน return_moves"""
        gesture, moves_param = REACTION[result]
        names = str(p[moves_param]).split() if moves_param else []
        missing = sorted({n for n in names if self._pose(data, n) is None})
        if missing:
            self.get_logger().info("ยังไม่ได้สอนท่า " + ", ".join(missing) + f" (arm teach {self.arm} <ท่า>) — ข้ามไป")
        for name in names:
            target = self._pose(data, name)
            if target:
                self._state(f"pose:{name}")
                self._move_to(target, p, fast=True)
        self._hand(gesture)
        self._sleep(float(p["react_hold"]))
        self._hand(p["end_gesture"])
        self._go_ready(data, ready, p)

    def _return_poses(self, data, p):
        """[(ชื่อท่า, มุม)] ของ return_moves — ต้องสอนครบทุกท่า"""
        return [(name, self._pose(data, name, required=True)) for name in str(p["return_moves"]).split()]

    def _go_ready(self, data, ready, p):
        """ไปท่าตั้ง — อยู่ไกลกว่า near_deg ให้เดินผ่าน return_moves ก่อน (เส้นทางที่สอนไว้) แล้วค่อยเข้าท่าตั้ง"""
        start = [self.current[n] for n in self.joints]
        if math.degrees(max(abs(a - b) for a, b in zip(ready, start))) > float(p["near_deg"]):
            steps = self._return_poses(data, p)
            self.get_logger().info("กลับท่าตั้ง: " + " → ".join([n for n, _ in steps] + [p["ready_pose"]]))
            for name, target in steps:
                self._state(f"return:{name}")
                self._move_to(target, p)
        self._move_to(ready, p)

    def _reveal(self, robot):
        self._hand(robot)
        self._state(f"reveal:{robot}")

    # ---------- แขน ----------
    def _move_to(self, target, p, fast=False):
        """fast = ท่าโชว์: ถ้าห่างไม่เกิน show_fast_max_deg ใช้ show_speed/show_accel (เร็ว) ไม่งั้นนุ่มเหมือนเดิม"""
        start = [self.current[n] for n in self.joints]
        far = math.degrees(max(abs(t - s) for t, s in zip(target, start)))
        if fast and far <= float(p["show_fast_max_deg"]):
            speed, accel, min_time = float(p["show_speed"]), float(p["show_accel"]), 0.3
        else:
            speed, accel, min_time = float(p["move_speed"]), float(p["move_accel"]), 1.0
        self._trajectory(programs.trapezoid(start, target, min(speed, MAX_SPEED), accel, min_time))

    def _trajectory(self, points, events=()):
        """ส่ง trajectory แล้วรอจนจบ ระหว่างนั้นเรียก events [(วินาทีจากเริ่ม, ฟังก์ชัน)] ตามเวลา — คืนเวลาเริ่ม"""
        if self.stop_flag.is_set():
            raise Stopped()
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.joints)
        for t, pos, vel in points:
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in pos]
            pt.velocities = [float(v) for v in vel]
            pt.time_from_start = _duration(t)
            goal.trajectory.points.append(pt)
        gh = _wait(self.client.send_goal_async(goal), 5.0)
        if gh is None or not gh.accepted:
            raise GameError(f"ตัวคุมแขน{ARM_TH[self.arm]} ไม่รับคำสั่ง")
        t0 = time.monotonic()
        self.goal_handle = gh
        pending = sorted(events, key=lambda e: e[0])
        result = gh.get_result_async()
        deadline = t0 + points[-1][0] + 10.0
        while not result.done():
            while pending and pending[0][0] <= time.monotonic() - t0:
                pending.pop(0)[1]()
            if self.stop_flag.is_set():
                gh.cancel_goal_async()
                raise Stopped()
            if time.monotonic() > deadline:
                gh.cancel_goal_async()
                raise GameError(f"แขน{ARM_TH[self.arm]} ไปไม่ถึงในเวลา")
            time.sleep(0.01)
        for _, fn in pending:
            fn()
        res = result.result()
        if self.stop_flag.is_set():
            raise Stopped()
        if res.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise GameError(f"ตัวคุมแขนรายงาน error {res.result.error_code} {res.result.error_string}".strip())
        return t0

    # ---------- อื่นๆ ----------
    def _hand(self, gesture):
        self.hand_pub.publish(String(data=gesture))

    def _state(self, text):
        self.state_pub.publish(String(data=text))

    def _sleep(self, secs):
        self._sleep_until(time.monotonic() + secs)

    def _sleep_until(self, t):
        while time.monotonic() < t:
            if self.stop_flag.is_set():
                raise Stopped()
            time.sleep(0.02)


def main():
    rclpy.init()
    node = RpsGame()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
