#!/usr/bin/env python3
"""show_mock — ฐาน + lidar + รีโมท + กล้องจำลอง สำหรับลองโชว์โดยไม่ต่อฮาร์ดแวร์ (ไม่ต้องมี Gazebo)

แทน relay_bridge + Mega + lidar + tray_detector:
  /cmd_vel -> ฐานวิ่งความเร็วคงที่ตามทิศ (เหมือนลูกบิด HW-687) + ไหลต่อนิดหน่อยหลังตัด -> /wheel_odom
  /scan    -> ยิงแสงจากตำแหน่ง lidar จริง (lidar_mount.yaml) ชนกล่องสี่เหลี่ยม / คนในโลกจำลอง (world_file)
  กันชน   -> ใช้ GuardLogic ตัวเดียวกับ relay_bridge (รวม /base_guard/dock) ทิศที่ห้าม = ฐานไม่ขยับ (เหมือน Mega)
  /motor_state "MODE:ROS" + "SW:BxDy" (รีโมทจำลอง) · /base_guard · /tray/status ok + /tray/pose (หลอก)
คุมจาก /show_mock/cmd (String):
  rc B=D | rc B=U | rc D=D | rc D=U    สวิตช์รีโมท    obstacle front 0.5 | obstacle at x y [r] | obstacle clear
  reset                                  กลับจุดเริ่ม   (pick fail N — ตัวคุมโชว์แบบจำลองฟังเอง)
"""
import math
import threading
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String, UInt8

from humanoi_base import lidar_mount
from humanoi_base.base_motion import scan_to_points
from humanoi_base.guard import GuardLogic

FOOT_FRONT = 0.265
DEFAULT_WORLD = {"start": [0.0, 0.0, 0.0],
                 "boxes": {"table_A": [0.385, -0.6, 0.985, 0.6]}}


def ray_box(ox, oy, dx, dy, box):
    """ระยะจาก (ox, oy) ตามทิศ (dx, dy) ถึงกล่อง [x0, y0, x1, y1] — None = ไม่ชน"""
    t_in, t_out = 0.0, math.inf
    for o, d, lo, hi in ((ox, dx, box[0], box[2]), (oy, dy, box[1], box[3])):
        if abs(d) < 1e-12:
            if not lo <= o <= hi:
                return None
            continue
        a, b = (lo - o) / d, (hi - o) / d
        if a > b:
            a, b = b, a
        t_in, t_out = max(t_in, a), min(t_out, b)
        if t_in > t_out:
            return None
    return t_in if t_in > 1e-6 else None


def ray_circle(ox, oy, dx, dy, cx, cy, r):
    fx, fy = ox - cx, oy - cy
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - r * r
    disc = b * b - c
    if disc < 0:
        return None
    t = -b - math.sqrt(disc)
    return t if t > 1e-6 else None


class ShowMock(Node):
    def __init__(self):
        super().__init__("show_mock")
        for name, default in (("world_file", ""), ("lin_speed", 0.15), ("ang_speed", 0.4),
                              ("coast", 0.02), ("coast_deg", 2.0)):
            self.declare_parameter(name, default)
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        world = DEFAULT_WORLD
        if gp("world_file"):
            with open(gp("world_file"), encoding="utf-8") as f:
                world = yaml.safe_load(f) or DEFAULT_WORLD
        self.start = [float(v) for v in world.get("start", [0, 0, 0])]
        self.boxes = {k: [float(v) for v in b] for k, b in (world.get("boxes") or {}).items()}
        self.lin, self.ang = float(gp("lin_speed")), float(gp("ang_speed"))
        self.coast_m, self.coast_rad = float(gp("coast")), math.radians(float(gp("coast_deg")))
        self.mount = lidar_mount.load()
        self.lock = threading.Lock()
        self.x, self.y, self.yaw = self.start
        self.cmd, self.cmd_t = (0.0, 0.0, 0.0), 0.0
        self.last_dir, self.coast_left = None, 0.0
        self.circles = []
        self.rc = {"B": "U", "D": "U"}
        self.guard = GuardLogic({})
        self.odom_pub = self.create_publisher(Odometry, "/wheel_odom", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 5)
        self.state_pub = self.create_publisher(String, "/motor_state", 20)
        self.guard_pub = self.create_publisher(UInt8, "/base_guard", 10)
        self.tray_status_pub = self.create_publisher(String, "/tray/status", 10)
        self.tray_pose_pub = self.create_publisher(PoseStamped, "/tray/pose", 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 20)
        self.create_subscription(String, "/show_mock/cmd", self._on_mock, 10)
        self.create_subscription(Float32, "/base_guard/dock",
                                 lambda m: self.guard.set_dock(m.data, time.monotonic()), 10)
        self.create_timer(0.05, self._tick)
        self.create_timer(0.1, self._scan_tick)
        self.create_timer(0.2, self._slow_tick)
        self.get_logger().info(f"ฐานจำลอง: {self.lin} m/s · {math.degrees(self.ang):.0f}°/s · "
                               f"โลก {', '.join(self.boxes) or '(ว่าง)'}")

    # ---------- รับคำสั่ง ----------
    def _on_cmd(self, msg):
        with self.lock:
            self.cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
            self.cmd_t = time.monotonic()

    def _on_mock(self, msg):
        w = msg.data.split()
        with self.lock:
            if len(w) == 2 and w[0] == "rc" and "=" in w[1]:
                sw, pos = w[1].split("=", 1)
                if sw in self.rc:
                    self.rc[sw] = pos
            elif w[:2] == ["obstacle", "clear"]:
                self.circles = []
            elif len(w) == 3 and w[:2] == ["obstacle", "front"]:
                d = FOOT_FRONT + float(w[2]) + 0.15
                self.circles.append((self.x + d * math.cos(self.yaw), self.y + d * math.sin(self.yaw), 0.15))
            elif len(w) >= 4 and w[:2] == ["obstacle", "at"]:
                self.circles.append((float(w[2]), float(w[3]), float(w[4]) if len(w) > 4 else 0.15))
            elif w == ["reset"]:
                self.x, self.y, self.yaw = self.start
                self.circles, self.coast_left, self.last_dir = [], 0.0, None

    # ---------- โลก ----------
    def lidar_pose(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        mx, my = self.mount["x"], self.mount["y"]
        return self.x + c * mx - s * my, self.y + s * mx + c * my, self.yaw + self.mount["yaw"]

    def ranges(self, n=360, rmax=12.0):
        lx, ly, lyaw = self.lidar_pose()
        out = []
        for i in range(n):
            a = -math.pi + 2 * math.pi * i / n
            dx, dy = math.cos(lyaw + a), math.sin(lyaw + a)
            best = rmax + 1.0
            for b in self.boxes.values():
                t = ray_box(lx, ly, dx, dy, b)
                if t is not None and t < best:
                    best = t
            for cx, cy, r in self.circles:
                t = ray_circle(lx, ly, dx, dy, cx, cy, r)
                if t is not None and t < best:
                    best = t
            out.append(best if best <= rmax else float("inf"))
        return out

    def _scan_tick(self):
        with self.lock:
            rng = self.ranges()
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser_link"
        msg.angle_min, msg.angle_max = -math.pi, math.pi - 2 * math.pi / 360
        msg.angle_increment = 2 * math.pi / 360
        msg.range_min, msg.range_max = 0.05, 12.0
        msg.ranges = rng
        self.scan_pub.publish(msg)
        pts = scan_to_points(msg, self.mount["x"], self.mount["y"], self.mount["yaw"])
        with self.lock:
            self.guard.update(self.guard.gaps(pts), time.monotonic())
        self.guard_pub.publish(UInt8(data=self.guard.mask))

    # ---------- ฐาน ----------
    @staticmethod
    def direction(cmd):
        vx, vy, wz = cmd
        if abs(wz) > 1e-3:
            return (0, 0, 1 if wz > 0 else -1)
        sx = 0 if abs(vx) < 1e-3 else (1 if vx > 0 else -1)
        sy = 0 if abs(vy) < 1e-3 else (1 if vy > 0 else -1)
        return None if (sx, sy) == (0, 0) else (sx, sy, 0)

    def blocked(self, d):
        need = 0
        if d[2]:
            need |= 16
        if d[0] > 0:
            need |= 1
        if d[0] < 0:
            need |= 2
        if d[1] > 0:
            need |= 4
        if d[1] < 0:
            need |= 8
        return bool(self.guard.mask & need)

    def step(self, d, dt, limit=None):
        """ขยับตามทิศ d หนึ่งช่วง dt — คืนระยะ (m หรือ rad) ที่ขยับ"""
        if d[2]:
            a = self.ang * dt if limit is None else min(self.ang * dt, limit)
            self.yaw = math.atan2(math.sin(self.yaw + d[2] * a), math.cos(self.yaw + d[2] * a))
            return a
        norm = math.hypot(d[0], d[1])
        dist = self.lin * dt if limit is None else min(self.lin * dt, limit)
        bx, by = d[0] / norm * dist, d[1] / norm * dist
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += c * bx - s * by
        self.y += s * bx + c * by
        return dist

    def _tick(self):
        dt = 0.05
        with self.lock:
            fresh = time.monotonic() - self.cmd_t < 0.5
            d = self.direction(self.cmd) if fresh else None
            if d is not None and not self.blocked(d):
                self.step(d, dt)
                self.last_dir, self.coast_left = d, (self.coast_rad if d[2] else self.coast_m)
            elif self.last_dir is not None and self.coast_left > 1e-6:
                if self.blocked(self.last_dir):
                    self.coast_left = 0.0
                else:
                    self.coast_left -= self.step(self.last_dir, dt, self.coast_left)
            else:
                self.last_dir = None
            x, y, yaw = self.x, self.y, self.yaw
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id, o.child_frame_id = "odom", "base_footprint"
        o.pose.pose.position.x, o.pose.pose.position.y = x, y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.odom_pub.publish(o)

    def _slow_tick(self):
        with self.lock:
            b, d = self.rc["B"], self.rc["D"]
        self.state_pub.publish(String(data="MODE:ROS"))
        self.state_pub.publish(String(data=f"SW:B{b}D{d}"))
        self.tray_status_pub.publish(String(data="ok"))
        p = PoseStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = "camera_color_optical_frame"
        p.pose.position.z, p.pose.orientation.w = 0.67, 1.0
        self.tray_pose_pub.publish(p)


def main():
    rclpy.init()
    node = ShowMock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
