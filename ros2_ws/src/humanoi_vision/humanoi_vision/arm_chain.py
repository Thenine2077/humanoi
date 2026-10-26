"""โซ่ข้อต่อของแขนจาก URDF (robot_description) — FK / IK / แผนเวลาเคลื่อนที่ ด้วย numpy ล้วน (ทดสอบบน PC ได้)

ใช้โมเดลเดียวกับที่ระบบแขนใช้อยู่ (รวมค่าชดเชยศูนย์ link_N_raw -> link_N และลิมิตที่เลื่อนตามบอร์ดแล้ว)
จึงไม่ต้องเปิด MoveIt ตอนหยิบ
"""
import math
import xml.etree.ElementTree as ET

import numpy as np

from humanoi_vision import tray

ANGLE_WEIGHT = 0.1  # m ต่อ rad — คลาดมุม 1 rad นับเท่าคลาดตำแหน่ง 10 cm (ระยะข้อมือถึงปลายนิ้ว)


def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


class Chain:
    """ข้อต่อจาก base ถึง tip ตาม URDF — q = มุมข้อ revolute เรียงจากโคนถึงปลาย (rad)"""

    def __init__(self, urdf_xml, base, tip):
        root = ET.fromstring(urdf_xml)
        by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
        segs, link = [], tip
        while link != base:
            j = by_child.get(link)
            if j is None:
                raise ValueError(f"URDF ไม่มีโซ่ข้อต่อ {base} -> {tip} (ขาดที่ {link})")
            o = j.find("origin")
            xyz = [float(v) for v in ((o.get("xyz") if o is not None else None) or "0 0 0").split()]
            rpy = [float(v) for v in ((o.get("rpy") if o is not None else None) or "0 0 0").split()]
            axis = None
            if j.get("type") in ("revolute", "continuous"):
                axis = np.array([float(v) for v in j.find("axis").get("xyz").split()])
                lim = j.find("limit")
                lo, hi = ((float(lim.get("lower")), float(lim.get("upper"))) if lim is not None
                          else (-math.pi, math.pi))
            elif j.get("type") != "fixed":
                raise ValueError(f"ข้อต่อ {j.get('name')} ชนิด {j.get('type')} — รองรับแค่ revolute/fixed")
            segs.append((j.get("name"), tray.mat(rpy_mat(*rpy), xyz), axis, (lo, hi) if axis is not None else None))
            link = j.find("parent").get("link")
        self.segs = segs[::-1]
        self.joints = [s[0] for s in self.segs if s[2] is not None]
        self.lower = np.array([s[3][0] for s in self.segs if s[2] is not None])
        self.upper = np.array([s[3][1] for s in self.segs if s[2] is not None])

    def fk(self, q):
        T, k = np.eye(4), 0
        for _, origin, axis, _ in self.segs:
            T = T @ origin
            if axis is not None:
                T = T @ tray.mat(tray.exp_so3(axis * q[k]), [0, 0, 0])
                k += 1
        return T

    def pose_error(self, T, target):
        """(คลาดตำแหน่ง m, คลาดมุม rad) — ใช้เช็กผล IK"""
        return (float(np.linalg.norm(target[:3, 3] - T[:3, 3])),
                float(np.linalg.norm(tray.log_so3(target[:3, :3] @ T[:3, :3].T))))

    def ik(self, target, seed, iters=300, tol_pos=2e-4, tol_rot=2e-3):
        """ท่าข้อต่อที่พาปลายโซ่ไป target (4x4 ในเฟรม base) เริ่มค้นจาก seed — damped least squares อยู่ในลิมิต
        -> (q, คลาดตำแหน่ง m, คลาดมุม rad) · ผลอยู่ใกล้ seed (ไม่กระโดดไปท่ากลับด้าน) เพราะค้นจาก seed ทีละน้อย"""
        q = np.clip(np.array(seed, float), self.lower, self.upper)

        def err(q):
            T = self.fk(q)
            return np.concatenate([target[:3, 3] - T[:3, 3],
                                   ANGLE_WEIGHT * tray.log_so3(target[:3, :3] @ T[:3, :3].T)])

        lam = 0.01
        e = err(q)
        for _ in range(iters):
            if np.linalg.norm(e[:3]) < tol_pos and np.linalg.norm(e[3:]) / ANGLE_WEIGHT < tol_rot:
                break
            J = np.zeros((6, len(q)))
            for k in range(len(q)):
                d = np.zeros(len(q))
                d[k] = 1e-6
                J[:, k] = (e - err(q + d)) / 1e-6  # e ลดลงเมื่อเข้าใกล้เป้า -> J = -de/dq
            step = J.T @ np.linalg.solve(J @ J.T + lam * lam * np.eye(6), e)
            step = np.clip(step, -0.2, 0.2)  # ก้าวละไม่เกิน ~11° กันกระโดดข้ามไปท่าอื่น
            qn = np.clip(q + step, self.lower, self.upper)
            en = err(qn)
            if en @ en < e @ e:
                q, e, lam = qn, en, max(lam * 0.5, 1e-4)
            else:
                lam *= 4
                if lam > 10:
                    break
        pos, rot = self.pose_error(self.fk(q), target)
        return q, pos, rot

    def line(self, start_q, T_from, T_to, step_m=0.005, partial=False):
        """ปลายโซ่วิ่งเป็นเส้นตรงจาก T_from ไป T_to (มุมเท่าเดิมตลอดทาง) — IK ทีละ ~step_m ต่อจากจุดก่อนหน้า
        แบ่ง n = round(ระยะ / step_m) ช่วงเท่ากัน -> [q ทุกจุด] (n จุด ไม่รวมจุดเริ่ม)
        ไปไม่ได้ (เกินระยะ / ชนลิมิต / ข้อต่อกระโดด): raise ValueError หรือ partial=True คืนเท่าที่ไปได้"""
        n = max(1, int(round(np.linalg.norm(T_to[:3, 3] - T_from[:3, 3]) / step_m)))
        qs, q = [], np.array(start_q, float)
        for k in range(1, n + 1):
            T = T_from.copy()
            T[:3, 3] = T_from[:3, 3] + (T_to[:3, 3] - T_from[:3, 3]) * k / n
            qn, pos, rot = self.ik(T, q)
            why = None
            if pos > 1e-3 or rot > math.radians(0.5):
                at = [f"ข้อ {i + 1}" for i in range(len(qn))
                      if qn[i] <= self.lower[i] + 1e-4 or qn[i] >= self.upper[i] - 1e-4]
                why = (f"แขนไปไม่ถึงจุดที่ {k}/{n} ของเส้นตรง (คลาด {1000 * pos:.1f} mm {math.degrees(rot):.1f}°) — "
                       + (f"{', '.join(at)} ชนลิมิต" if at else "เกินระยะแขน"))
            elif np.max(np.abs(qn - q)) > math.radians(8):
                why = (f"ข้อต่อกระโดด {math.degrees(np.max(np.abs(qn - q))):.0f}° กลางเส้นตรง (จุดที่ {k}/{n}) — "
                       "ท่าแขนเปลี่ยนรูป ไม่เดินต่อ")
            if why:
                if partial:
                    self.line_stop = why
                    return qs
                raise ValueError(why)
            qs.append(qn)
            q = qn
        self.line_stop = None
        return qs


def timed_path(waypoints, speed, accel, dt=0.1):
    """แผนเวลาผ่านจุดข้อต่อทุกจุดตามลำดับ: เร่ง -> ความเร็วคงที่ -> เบรก (ข้อที่ขยับมากสุดวิ่งไม่เกิน speed rad/s)
    เหมือน ar_program.programs.trapezoid แต่ผ่านหลายจุด (เช่นเส้นตรงที่ IK ไว้) — จุดแรก = ท่าปัจจุบัน
    -> [(t, positions, velocities)] ไม่รวมจุดเริ่ม · จุดสุดท้ายความเร็ว 0"""
    P = np.array(waypoints, float)
    seg = np.max(np.abs(np.diff(P, axis=0)), axis=1) if len(P) > 1 else np.zeros(0)
    s_at = np.concatenate([[0.0], np.cumsum(seg)])
    dist = float(s_at[-1])
    if dist < 1e-9:
        return [(dt, list(P[-1]), [0.0] * P.shape[1])]
    v, a = speed, accel
    if dist >= v * v / a:
        t_acc = v / a
        total = 2 * t_acc + (dist - v * v / a) / v
    else:
        t_acc = math.sqrt(dist / a)
        v = a * t_acc
        total = 2 * t_acc

    def s_of(t):
        if t < t_acc:
            return 0.5 * a * t * t
        if t < total - t_acc:
            return 0.5 * a * t_acc * t_acc + v * (t - t_acc)
        r = total - t
        return dist - 0.5 * a * r * r

    def q_of(s):
        k = int(np.clip(np.searchsorted(s_at, s, side="right") - 1, 0, len(seg) - 1))
        f = 0.0 if seg[k] < 1e-12 else (s - s_at[k]) / seg[k]
        return P[k] + (P[k + 1] - P[k]) * min(max(f, 0.0), 1.0)

    out, prev_q, prev_t = [], P[0], 0.0
    for i in range(1, int(math.ceil(total / dt)) + 1):
        t = min(i * dt, total)
        q = q_of(min(s_of(t), dist))
        out.append((t, q, (q - prev_q) / max(t - prev_t, 1e-9)))
        prev_q, prev_t = q, t
    out[-1] = (out[-1][0], P[-1], np.zeros(P.shape[1]))
    return [(t, [float(x) for x in q], [float(x) for x in vel]) for t, q, vel in out]
