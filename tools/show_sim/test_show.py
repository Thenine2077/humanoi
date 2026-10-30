#!/usr/bin/env python3
"""ทดสอบตัวคุมโชว์ (humanoi_mission show) กับชุดจำลองทั้งชุด — ไม่ต่อฮาร์ดแวร์ ไม่ใช้ Gazebo

เปิด: แขน mock (ar_dual) + ตัวเล่นโปรแกรม + มือ dry_run + show_mock (ฐาน/lidar/รีโมท/กล้องจำลอง) + show (mock:=true)
แล้วเล่นโชว์ test_ab หลายรอบ:
  1 ครบรอบ                     จบที่จุดเริ่ม (เข้าจอด A ห่าง 0.12 m) — กันชนต้องยอมให้จอดใกล้กว่า 0.30 m
  2 กล่องหาย 2 ครั้ง           ลองใหม่เอง 1 ครั้ง -> หยุดรอ -> SWB ลองใหม่ -> ครบ
  3 คนเดินตัดหน้า             ฐานหยุดรอ -> คนออก -> วิ่งต่อจนครบ
  4 SWD กลางทาง แล้ว SWB       หยุดนิ่ง -> ทำต่อเฉพาะระยะที่เหลือ -> ครบ จบที่จุดเริ่ม
  5 SWD สองครั้ง               จบรอบกลางคัน
แล้วทดสอบ show_setup route-record (ขับจำลอง -> จดขา -> สร้างขากลับ) + GuardLogic เข้าจอด (ไม่ใช้ ROS)
รัน: tools/show_sim/run_show_tests.sh
"""
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

HERE = os.path.dirname(os.path.realpath(__file__))
OUT = tempfile.mkdtemp(prefix="show_test_")
ENV = dict(os.environ, ROS_DOMAIN_ID=os.environ.get("SHOW_TEST_DOMAIN", "74"), ARM_LOG_DIR=f"{OUT}/logs",
           AR_PROGRAM_FILE=f"{OUT}/programs.yaml", PYTHONIOENCODING="utf-8", PYTHONUNBUFFERED="1")
os.environ["ROS_DOMAIN_ID"] = ENV["ROS_DOMAIN_ID"]
ROUTES = f"{OUT}/routes.yaml"
procs = {}
results = []


def start(name, cmd, stdin=subprocess.DEVNULL):
    log = open(f"{OUT}/{name}.log", "w")
    p = subprocess.Popen(cmd, env=ENV, stdout=log, stderr=subprocess.STDOUT, stdin=stdin, start_new_session=True)
    procs[name] = p
    return p


def stop(name, sig=signal.SIGINT, wait=5.0):
    p = procs.pop(name, None)
    if p is None or p.poll() is not None:
        return
    try:
        os.killpg(p.pid, sig)
        p.wait(timeout=wait)
    except (subprocess.TimeoutExpired, ProcessLookupError):
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def wait_log(name, text, timeout=30.0):
    """รอจนโปรแกรมพิมพ์ text ลง log (เริ่มโปรแกรมบน /mnt/c ช้า เวลาตายตัวไม่พอ)"""
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        if text in open(f"{OUT}/{name}.log", encoding="utf-8", errors="replace").read():
            return
        time.sleep(0.2)
    raise TimeoutError(f"{name} ไม่พิมพ์ '{text}' ใน {timeout:.0f} วิ")


def run(cmd, timeout=60):
    r = subprocess.run(cmd, env=ENV, capture_output=True, text=True, timeout=timeout)
    return r.returncode, r.stdout + r.stderr


def check(name, ok, detail=""):
    results.append((name, bool(ok)))
    print(f"  {'✓' if ok else '✗'} {name}{(' — ' + detail) if detail else ''}", flush=True)


class Client(Node):
    def __init__(self):
        super().__init__("show_test_client")
        self.state, self.pose = {}, None
        self.lock = threading.Lock()
        self.auto_confirm, self.confirmed = True, set()
        self.cmd = self.create_publisher(String, "/show/cmd", 10)
        self.mock = self.create_publisher(String, "/show_mock/cmd", 10)
        self.vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(String, "/show/state", self._on_state, 10)
        self.create_subscription(Odometry, "/wheel_odom", self._on_odom, 10)

    def _on_state(self, msg):
        st = json.loads(msg.data)
        with self.lock:
            self.state = st
        key = (st.get("round"), st.get("step"))
        if self.auto_confirm and st.get("phase") == "confirm" and key not in self.confirmed:
            self.confirmed.add(key)
            self.send("go")

    def _on_odom(self, msg):
        p = msg.pose.pose
        with self.lock:
            self.pose = (p.position.x, p.position.y, 2 * math.atan2(p.orientation.z, p.orientation.w))

    def send(self, text):
        self.cmd.publish(String(data=text))

    def mock_cmd(self, text):
        self.mock.publish(String(data=text))

    def get(self):
        with self.lock:
            return dict(self.state), self.pose

    def wait(self, pred, timeout, what):
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            st, pose = self.get()
            if pose is not None and pred(st, pose):
                return st, pose
            time.sleep(0.1)
        st, pose = self.get()
        raise TimeoutError(f"รอ {what} เกิน {timeout:.0f} วิ (สถานะล่าสุด {st}, ตำแหน่ง {pose})")

    def flick(self, sw):
        for pos, dt in (("U", 0.5), ("D", 0.6), ("U", 0.5)):
            self.mock_cmd(f"rc {sw}={pos}")
            time.sleep(dt)

    def round_end(self, n, timeout=240):
        st, pose = self.wait(lambda s, p: s.get("round") == n and s.get("phase") == "idle" and "last" in s
                             and s.get("message", "").startswith(f"รอบที่ {n}"), timeout, f"จบรอบ {n}")
        return st["last"], pose

    def still(self, seconds=1.5):
        _, a = self.get()
        time.sleep(seconds)
        _, b = self.get()
        return math.hypot(b[0] - a[0], b[1] - a[1]) < 0.01 and abs(b[2] - a[2]) < math.radians(1)


def rounds_log():
    out = []
    for f in sorted(os.listdir(f"{OUT}/logs")) if os.path.isdir(f"{OUT}/logs") else []:
        if f.startswith("show-"):
            out += [json.loads(line) for line in open(f"{OUT}/logs/{f}", encoding="utf-8") if line.strip()]
    return out


def near_start(pose):
    return abs(pose[0]) < 0.05 and abs(pose[1]) < 0.06 and abs(math.degrees(pose[2])) < 4.0


def unit_guard():
    from humanoi_base.guard import FWD, GuardLogic
    def gaps(front):
        return {"forward": front, "back": None, "left": None, "right": None, "turn": None}

    g = GuardLogic({})
    check("กันชน: ปกติห้ามเดินหน้าที่ 0.12 m", g.update(gaps(0.12), 0.0) & FWD)
    g = GuardLogic({})  # เข้าจอดจริง: วิ่งเข้าหาโต๊ะจากไกล ระหว่างนั้นส่งคำสั่งจอดซ้ำ
    g.update(gaps(0.5), 0.0)
    g.update(gaps(0.5), 0.6)  # โล่งพ้น clear_time -> ปลด
    g.set_dock(0.09, 0.7)
    check("กันชน: ตอนจอด (ห้ามที่ 0.09) เข้าใกล้ถึง 0.12 m ได้", not g.update(gaps(0.12), 0.8) & FWD)
    check("กันชน: ใกล้กว่าระยะจอด = ห้าม", g.update(gaps(0.07), 0.9) & FWD)
    check("กันชน: หยุดส่งคำสั่งจอดเกิน 1 วิ -> กลับมาห้ามที่ 0.30", g.update(gaps(0.12), 1.8) & FWD)
    g.set_dock(0.01, 3.0)
    check("กันชน: ระยะจอดไม่ต่ำกว่า 0.05 m", abs(g.dock[0] - 0.05) < 1e-9)


def unit_routes():
    from humanoi_mission import show_routes as R
    legs = R.parse_text("turn 90, forward 1.5, turn -90")
    back = R.reverse(legs)
    check("เส้นทาง: ขากลับของ หมุน90-เดิน1.5-หมุน-90", back == [("turn", -90.0), ("forward", 1.5), ("turn", 90.0)],
          R.describe(back))
    check("เส้นทาง: ชื่อขากลับ", R.reverse_name("A_to_B") == "B_to_A" and R.reverse_name("x") == "x_back")
    check("เส้นทาง: โต๊ะต้นทาง/ปลายทาง", R.ends_of("A_to_B") == ("A", "B") and R.ends_of("x") == (None, None))
    check("เส้นทาง: เหลือระยะหลังหยุด", R.remaining("forward", 1.5, 0.6) == 0.9 and
          R.remaining("turn", -90.0, math.radians(30)) == -60.0 and R.remaining("forward", 1.5, 1.49) is None)


def main():
    shutil.copy(f"{HERE}/programs_test.yaml", f"{OUT}/programs.yaml")
    print(f"ผลทดสอบเก็บที่ {OUT}", flush=True)
    unit_guard()
    unit_routes()
    rclpy.init()
    cl = Client()
    spin = threading.Thread(target=rclpy.spin, args=(cl,), daemon=True)
    spin.start()
    try:
        start("arms", ["ros2", "launch", "ar_dual", "dual_hardware.launch.py", "left_hw:=mock", "right_hw:=mock"])
        start("player", ["ros2", "run", "ar_program", "arm_program", "--ros-args",
                         "-p", "speed:=1.0", "-p", "max_speed:=1.0", "-p", "accel:=1.0"])
        start("hand", ["ros2", "run", "humanoi_games", "amazinghand_node", "--ros-args",
                       "-p", "arm:=left", "-p", "dry_run:=true"])
        start("mock", ["ros2", "run", "humanoi_mission", "show_mock", "--ros-args", "-p", f"world_file:={HERE}/world.yaml",
                       "-p", "lin_speed:=0.3", "-p", "ang_speed:=0.8"])
        time.sleep(8)
        rc, out = run(["ros2", "run", "humanoi_mission", "show_setup", "route-set", "A_to_B",
                       "turn 90, forward 1.5, turn -90", "--ros-args", "-p", f"routes_file:={ROUTES}"])
        check("route-set A_to_B + สร้าง B_to_A", rc == 0 and "B_to_A" in yaml.safe_load(open(ROUTES))["routes"],
              out.strip().splitlines()[-1] if out.strip() else "")
        rc, out = run(["ros2", "run", "humanoi_mission", "show_setup", "dock-set", "B", "0.12",
                       "--ros-args", "-p", f"routes_file:={ROUTES}"])
        check("dock-set B 0.12", rc == 0)
        rc, out = run(["ros2", "run", "humanoi_mission", "show_setup", "dock-save", "A",
                       "--ros-args", "-p", f"routes_file:={ROUTES}"])
        dock_a = yaml.safe_load(open(ROUTES))["docks"].get("A")
        check("dock-save A (จอดอยู่หน้าโต๊ะ A) ≈ 0.12 m", rc == 0 and dock_a is not None and abs(dock_a - 0.12) < 0.02,
              f"ได้ {dock_a}")
        # ระยะถอยแบบที่ route-record จดให้ (world.yaml: เส้นทางเริ่มห่างโต๊ะ A 0.37 จบห่างโต๊ะ B 0.30) -> undock ใช้ lidar
        data = yaml.safe_load(open(ROUTES))
        data["approach"] = {"A": 0.37, "B": 0.30}
        yaml.safe_dump(data, open(ROUTES, "w"), allow_unicode=True)

        start("show", ["ros2", "run", "humanoi_mission", "show", "run", "test_ab", "--ros-args",
                       "-p", f"shows_file:={HERE}/test_shows.yaml", "-p", f"routes_file:={ROUTES}", "-p", "mock:=true"])
        cl.wait(lambda s, p: s.get("phase") == "idle", 60, "โชว์พร้อม")
        time.sleep(1.0)

        print("1) ครบรอบ", flush=True)
        cl.send("go")
        last, pose = cl.round_end(1)
        check("รอบ 1 จบครบ", last == "done", last)
        check("รอบ 1 กลับมาจอดที่ A", near_start(pose), f"ตำแหน่ง {pose[0]:+.3f}, {pose[1]:+.3f}, "
              f"{math.degrees(pose[2]):+.1f}°")

        print("2) หากล่องไม่เจอ 2 ครั้ง", flush=True)
        cl.mock_cmd("pick fail 2")
        time.sleep(0.5)
        cl.send("go")
        st, _ = cl.wait(lambda s, p: s.get("phase") == "error" and s.get("kind") == "pick", 60, "หยุดรอที่ขั้นหยิบ")
        check("หยิบพังเกิน retry -> หยุดรอคนคุม", True, st.get("message", "")[:60])
        cl.flick("B")
        last, pose = cl.round_end(2)
        pick = [s for s in rounds_log()[-1]["steps"] if s["kind"] == "pick"][0]
        check("รอบ 2 จบครบหลัง SWB ลองใหม่", last == "done" and pick["tries"] == 3, f"หยิบ {pick['tries']} ครั้ง")

        print("3) คนเดินตัดหน้าระหว่างวิ่ง", flush=True)
        cl.send("go")
        cl.wait(lambda s, p: s.get("kind") == "route" and p[1] > 0.3, 90, "ฐานวิ่งขึ้นเหนือ")
        cl.mock_cmd("obstacle front 0.5")
        time.sleep(1.5)
        check("ฐานหยุดรอเมื่อมีคนขวาง", cl.still(1.5))
        cl.mock_cmd("obstacle clear")
        last, pose = cl.round_end(3)
        check("รอบ 3 วิ่งต่อจนครบ", last == "done" and near_start(pose), last)

        print("4) SWD กลางทาง แล้ว SWB ทำต่อ", flush=True)
        cl.send("go")
        cl.wait(lambda s, p: s.get("kind") == "route" and p[1] > 0.6, 90, "ฐานวิ่งกลางทาง")
        cl.flick("D")
        st, _ = cl.wait(lambda s, p: s.get("phase") == "paused", 10, "หยุดหลัง SWD")
        check("SWD = หยุดนิ่ง", cl.still(1.5), st.get("message", ""))
        cl.flick("B")
        last, pose = cl.round_end(4)
        route = [s for s in rounds_log()[-1]["steps"] if s["kind"] == "route"][0]
        check("ทำต่อแล้วจบครบ จบที่จุดเริ่ม (วิ่งแค่ระยะที่เหลือ)", last == "done" and near_start(pose)
              and route["stops"] == 1, f"ตำแหน่ง {pose[0]:+.3f}, {pose[1]:+.3f}")

        print("5) SWD สองครั้ง = จบรอบ", flush=True)
        cl.send("go")
        cl.wait(lambda s, p: s.get("kind") == "route" and p[1] > 0.6, 90, "ฐานวิ่งกลางทาง")
        cl.flick("D")
        cl.wait(lambda s, p: s.get("phase") == "paused", 10, "หยุดหลัง SWD")
        cl.flick("D")
        last, _ = cl.round_end(5, timeout=20)
        check("SWD ครั้งที่สอง = จบรอบกลางคัน", last == "ended", last)
        check("บันทึกครบ 5 รอบ", len(rounds_log()) == 5, f"{len(rounds_log())} รอบ")

        cl.send("quit")
        time.sleep(2.0)
        check("q = ออกจากโชว์", procs["show"].poll() is not None)
        stop("show")

        print("6) สอนเส้นทางด้วยการขับ", flush=True)
        cl.mock_cmd("reset")
        time.sleep(1.0)
        rec = start("record", ["ros2", "run", "humanoi_mission", "show_setup", "route-record", "R_to_S",
                               "--ros-args", "-p", f"routes_file:={ROUTES}"], stdin=subprocess.PIPE)
        wait_log("record", "สอนเส้นทาง")
        time.sleep(1.0)
        for vx, wz, secs in ((-0.2, 0.0, 1.5), (0.0, 0.0, 1.5), (0.0, 0.5, 1.96), (0.0, 0.0, 1.5),
                             (0.2, 0.0, 1.0), (0.0, 0.0, 1.5)):
            t_end = time.monotonic() + secs
            while time.monotonic() < t_end:
                t = Twist()
                t.linear.x, t.angular.z = vx, wz
                cl.vel.publish(t)
                time.sleep(0.05)
        cl.send("save")
        rec.wait(timeout=10)
        data = yaml.safe_load(open(ROUTES))
        routes = data["routes"]
        legs = [tuple(x.items())[0] for x in routes.get("R_to_S", [])]
        ok = (len(legs) == 3 and legs[0][0] == "back" and abs(legs[0][1] - 0.47) < 0.05 and legs[1][0] == "turn"
              and abs(legs[1][1] - 92) < 6 and legs[2][0] == "forward" and abs(legs[2][1] - 0.32) < 0.05)
        check("route-record จดได้ ถอย 0.47 / หมุน 92° / เดินหน้า 0.32", ok, str(legs))
        check("route-record สร้างขากลับ S_to_R", "S_to_R" in routes)
        r = (data.get("approach") or {}).get("R")
        check("route-record จดระยะถอยของต้นทาง R ≈ 0.12 m (เริ่มจากจุดจอด A)", r is not None and abs(r - 0.12) < 0.03,
              f"ได้ {r}")
    except Exception as exc:  # noqa: BLE001
        check(f"ทดสอบล้ม: {exc}", False)
    finally:
        for name in list(procs):
            stop(name)
        cl.destroy_node()
        rclpy.try_shutdown()
    bad = [n for n, ok in results if not ok]
    print(f"\n{len(results) - len(bad)}/{len(results)} ผ่าน" + (f" · ไม่ผ่าน: {bad}" if bad else "") + f" · log: {OUT}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
