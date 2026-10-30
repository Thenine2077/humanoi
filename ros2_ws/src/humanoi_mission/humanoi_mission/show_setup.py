#!/usr/bin/env python3
"""show_setup — ตั้งโชว์หน้างาน: สอนเส้นทาง · จดระยะจอด · เช็กกล้องกับแขน (เขียนลง config/routes.yaml)

  arm show route-record A_to_B          ขับรีโมทจากจุดเริ่มไปถึงหน้าโต๊ะ B ทีละขา (ตรง หรือหมุนอยู่กับที่)
                                        หยุดนิ่งเกือบ 1 วิ = จบขา · Enter = บันทึก · u = ลบขาล่าสุด · q = ยกเลิก
  arm show route-set A_to_B "turn 90, forward 1.5, turn -90"    พิมพ์เส้นทางเอง (วัดด้วยตลับเมตร)
  arm show dock-save B                  จอดหุ่นหน้าโต๊ะ B ในตำแหน่งที่ใช่ (ขับเอง) แล้วจดระยะที่ lidar เห็น
  arm show dock-set B 0.12              ตั้งระยะจอดเอง (m จากขอบหน้าตัวหุ่น)
  arm show verify [left]                เช็กกล้อง: แขนไปค้างเหนือที่จับกล่อง (ไม่จับ) ให้ดูว่าตรงไหม แล้วกลับ

ชื่อ X_to_Y ได้เส้นทางขากลับ Y_to_X ให้อัตโนมัติ (หมุน 180 → ย้อนขา → หมุน 180) ใช้แต่เดินหน้า
ไฟล์: config/routes.yaml (ข้าง missions.yaml) — ภารกิจ / โชว์อ่านใหม่ทุกรอบ ไม่ต้อง build
"""
import argparse
import math
import queue
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from humanoi_base.base_motion import BaseMotion, MoveError, declare, wrap
from humanoi_mission import show_routes

STILL_S = 0.8        # นิ่งนานเท่านี้ = จบขา
MOVE_M = 0.002       # ต่อรอบ 0.05 s — เปลี่ยนเกินนี้ = กำลังขยับ
MOVE_DEG = 0.3
TURN_MIN_DEG = 10.0  # ขาที่หมุนเกินนี้และเลื่อนไม่ถึง TURN_MAX_SHIFT = ขาหมุน
TURN_MAX_SHIFT = 0.10
LINE_MIN = 0.03      # ขาตรงสั้นกว่านี้ไม่นับ (ขยับนิดหน่อย)


def classify(dx, dy, dyaw_deg):
    """การขยับหนึ่งขา (ในเฟรมตอนเริ่มขา) -> ((ทิศ, ระยะ) หรือ None, คำเตือน)"""
    dist = math.hypot(dx, dy)
    if abs(dyaw_deg) >= TURN_MIN_DEG and dist < TURN_MAX_SHIFT:
        return ("turn", round(dyaw_deg, 1)), ""
    if dist < LINE_MIN:
        return None, ""
    if abs(dx) >= abs(dy):
        leg = ("forward" if dx > 0 else "back", round(abs(dx), 3))
        minor = abs(dy)
    else:
        leg = ("left" if dy > 0 else "right", round(abs(dy), 3))
        minor = abs(dx)
    warn = []
    if abs(dyaw_deg) > 8.0:
        warn.append(f"หัวหมุนไป {dyaw_deg:+.0f}° ระหว่างวิ่งตรง")
    if minor > 0.25 * dist:
        warn.append(f"เบี้ยวข้าง {100 * minor:.0f} cm")
    return leg, ("⚠ " + " · ".join(warn) + " — ขับให้ตรง/หมุนอยู่กับที่ทีละอย่าง แล้วลบ (u) ขับใหม่") if warn else ""


class Setup(Node):
    def __init__(self):
        super().__init__("show_setup")
        self.declare_parameter("routes_file", "")
        self.path = self.get_parameter("routes_file").value or show_routes.routes_file()
        self.motion = BaseMotion(self, declare(self))
        self.keys = queue.Queue()
        self.create_subscription(String, "/show/cmd", lambda m: self.keys.put(m.data), 10)

    def save_route(self, name, legs, gaps=(None, None)):
        """gaps = (ข้างหน้าห่างเท่าไหร่ตอนเริ่ม, ตอนจบ) -> approach ของโต๊ะต้นทาง / ปลายทาง (ขั้น undock ใช้)"""
        data = show_routes.load(self.path)
        back = show_routes.reverse(legs)
        rname = show_routes.reverse_name(name)
        data["routes"][name] = show_routes.as_items(legs)
        data["routes"][rname] = show_routes.as_items(back)
        notes = []
        for table, g in zip(show_routes.ends_of(name), gaps):
            if table and g is not None:
                data["approach"][table] = round(float(g), 3)
                notes.append(f"{table} {g:.2f} m")
        show_routes.save(self.path, data)
        print(f"✓ บันทึกเส้นทาง {name}: {show_routes.describe(legs)}\n"
              f"✓ ขากลับ {rname}: {show_routes.describe(back)}")
        if notes:
            print(f"✓ ถอยออกจากโต๊ะ (undock) จนห่าง: {', '.join(notes)}")
        print(f"  ไฟล์ {self.path}")

    def save_dock(self, name, value):
        data = show_routes.load(self.path)
        data["docks"][name] = round(float(value), 3)
        show_routes.save(self.path, data)
        print(f"✓ ระยะจอด {name} = {value:.3f} m (ขอบหน้าตัวหุ่นถึงสิ่งที่ lidar เห็น) · ไฟล์ {self.path}")

    def wait_odom(self, timeout=5.0):
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            o = self.motion.pose()
            if o and time.monotonic() - o[0] < self.motion.cfg["sensor_timeout"]:
                return o
            time.sleep(0.05)
        raise MoveError(f"ไม่มี odometry ที่ {self.motion.cfg['odom_topic']} — เปิดฐาน (arm base) ก่อน")

    def gap_now(self):
        """ข้างหน้าห่างเท่าไหร่ (m) — ไม่มี lidar / ไม่เห็นอะไร = None"""
        try:
            self.motion.wait_ready()  # เพิ่งเปิดโปรแกรม scan แรกอาจยังไม่มา
            return self.motion.front_gap(0.5)
        except MoveError:
            return None

    def record(self, name):
        self.wait_odom()
        g0 = self.gap_now()  # ตอนเริ่ม = ระยะที่ขั้น undock ของโต๊ะต้นทางถอยออกมา
        print(f"สอนเส้นทาง '{name}' — ขับรีโมททีละขา: วิ่งตรง (หน้า/หลัง/สไลด์) หรือหมุนอยู่กับที่ อย่าทำพร้อมกัน\n"
              f"  หยุดนิ่ง ~1 วิ = จบขา · Enter = บันทึก · u = ลบขาล่าสุด · q = ยกเลิก", flush=True)
        legs, start, still_since, last, acc_yaw = [], None, None, None, 0.0
        while True:
            k = None
            if start is None:  # รับคำสั่งตอนนิ่งเท่านั้น (กำลังวิ่งอยู่ = เก็บไว้ทำหลังจบขา)
                try:
                    k = self.keys.get(timeout=0.05).strip().lower()
                except queue.Empty:
                    pass
            else:
                time.sleep(0.05)
            if k is not None:
                if k in ("", "save"):
                    if not legs:
                        print("  ยังไม่มีขา — ขับก่อน", flush=True)
                        continue
                    g1 = self.gap_now()
                    self.save_route(name, legs, (g0, g1))
                    self.check_end_gap(g1)
                    return 0
                if k in ("q", "cancel"):
                    print("ยกเลิก — ไม่บันทึก", flush=True)
                    return 1
                if k == "u" and legs:
                    print(f"  ลบขา {len(legs)}: {show_routes.leg_text(*legs.pop())}", flush=True)
            o = self.motion.pose()
            if o is None or time.monotonic() - o[0] > self.motion.cfg["sensor_timeout"]:
                continue
            now = time.monotonic()
            if last is None:
                last = o
                continue
            moved = (math.hypot(o[1] - last[1], o[2] - last[2]) > MOVE_M
                     or abs(math.degrees(wrap(o[3] - last[3]))) > MOVE_DEG)
            if start is None:
                if moved:
                    start, acc_yaw, still_since = last, 0.0, None
            if start is not None:
                acc_yaw += wrap(o[3] - last[3])
                if moved:
                    still_since = None
                else:
                    still_since = still_since or now
                    if now - still_since >= STILL_S:
                        px, py = o[1] - start[1], o[2] - start[2]
                        c, s = math.cos(start[3]), math.sin(start[3])
                        leg, warn = classify(c * px + s * py, -s * px + c * py, math.degrees(acc_yaw))
                        if leg:
                            legs.append(leg)
                            print(f"  ขา {len(legs)}: {show_routes.leg_text(*leg)} {warn}".rstrip(), flush=True)
                        start = None
            last = o

    @staticmethod
    def check_end_gap(g):
        """เส้นทางควรจบห่างโต๊ะ 0.45–0.7 m ให้ dock เข้าต่อ — ใกล้กว่านี้ฐานจะหยุดรอกันชน (0.35 m) จนหมดเวลา"""
        if g is None:
            print("⚠ ตอนจบเส้นทางไม่เห็นอะไรข้างหน้า — ควรจบหันหน้าเข้าโต๊ะ ห่าง 0.45–0.7 m ให้ dock เข้าต่อได้", flush=True)
            return
        if g < 0.45:
            print(f"⚠ ตอนจบเส้นทางข้างหน้าห่าง {g:.2f} m — ใกล้เกิน: ขาสุดท้ายที่วิ่งเข้าหาโต๊ะจะโดนกันชนหยุดรอที่ 0.35 m\n"
                  "  สอนใหม่ให้หยุดห่างโต๊ะ 0.45–0.7 m แล้วให้ขั้น dock เข้าจอดต่อ", flush=True)
        else:
            print(f"  ตอนจบเส้นทางข้างหน้าห่าง {g:.2f} m — ขั้น dock เข้าจอดต่อจากตรงนี้ได้", flush=True)

    def dock_save(self, name):
        self.motion.wait_ready()
        g = self.motion.front_gap(1.0)
        if g is None:
            print("✗ ไม่เห็นอะไรข้างหน้าตัวหุ่น (ในแนวกว้างตัวหุ่น) — หันหน้าเข้าโต๊ะแล้วลองใหม่")
            return 1
        if g > 0.6:
            print(f"⚠ ข้างหน้าห่าง {g:.2f} m — ไกลกว่าปกติสำหรับจอดหน้าโต๊ะ เช็กว่าจอดถูกที่ (บันทึกให้แล้ว)")
        self.save_dock(name, g)
        return 0


def verify(ex, arm):
    """แขนไปค้างเหนือที่จับ (ไม่จับ) ให้ดูว่ากล้องกับแขนยังตรงกัน แล้วกลับท่าเดิม"""
    from humanoi_vision.tray_pick import PickError, TrayPick
    p = TrayPick(f"show_verify_{arm}", parameter_overrides=[Parameter("arm", value=arm)])
    ex.add_node(p)
    keys = queue.Queue()
    threading.Thread(target=lambda: [keys.put(line.strip()) for line in sys.stdin], daemon=True).start()
    try:
        time.sleep(1.5)
        pl = p.plan()
        print(p.describe(pl).split("\n")[0])
        if not p.action.wait_for_server(timeout_sec=2.0):
            raise PickError(f"ไม่พบตัวคุมแขน /{arm}_arm_controller — เปิด arm start ก่อน")
        print(f"แขน{arm} จะไปค้างเหนือที่จับ {100 * pl['h_hover']:.0f} cm (ไม่จับ) — Enter = ไป · q = ยกเลิก", flush=True)
        if keys.get() == "q":
            return 1
        p.move([pl["q_now"], pl["q_hover"]], float(p.p["speed"]), "ไปค้างเหนือที่จับ")
        print("ดูปลายนิ้วเทียบหูกล่อง: ตรงกลาง คลาดไม่เกิน ~5 mm = ใช้ได้ · คลาดมาก = กล้องขยับ สอนใหม่ (arm tray-teach)\n"
              "Enter = กลับท่าเดิม", flush=True)
        keys.get()
        p.move([p.current_q(), pl["q_now"]], float(p.p["speed"]), "กลับท่าเดิม")
        print("✓ กลับท่าเดิมแล้ว")
        return 0
    except PickError as exc:
        print(f"✗ {exc}")
        return 1
    finally:
        p.destroy_node()


def main():
    ap = argparse.ArgumentParser(description="ตั้งโชว์หน้างาน — เส้นทาง ระยะจอด เช็กกล้อง")
    ap.add_argument("action", help="route-record | route-set | dock-save | dock-set | verify")
    ap.add_argument("name", nargs="?")
    ap.add_argument("value", nargs="?")
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Setup()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    rc = 1
    try:
        if args.action == "route-set":
            if not (args.name and args.value):
                ap.error('ใช้: route-set <ชื่อ> "forward 1.2, turn 90, forward 0.5"')
            node.save_route(args.name, show_routes.parse_text(args.value))
            rc = 0
        elif args.action == "dock-set":
            if not (args.name and args.value):
                ap.error("ใช้: dock-set <ชื่อ> <ระยะ m>")
            node.save_dock(args.name, float(args.value))
            rc = 0
        elif args.action == "dock-save":
            if not args.name:
                ap.error("ใช้: dock-save <ชื่อ>")
            rc = node.dock_save(args.name)
        elif args.action == "route-record":
            if not args.name:
                ap.error("ใช้: route-record <ชื่อ เช่น A_to_B>")
            threading.Thread(target=lambda: [node.keys.put(line) for line in sys.stdin], daemon=True).start()
            rc = node.record(args.name)
        elif args.action == "verify":
            rc = verify(ex, args.name or "left")
        else:
            ap.error(f"ไม่รู้จัก {args.action}")
    except (ValueError, MoveError) as exc:
        print(f"✗ {exc}")
        rc = 1
    except KeyboardInterrupt:
        print("\nยกเลิก")
    finally:
        ex.shutdown(timeout_sec=1.0)
        spin.join(timeout=1.0)
        node.destroy_node()
        rclpy.try_shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
