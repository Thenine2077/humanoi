"""คณิตของ hand-eye calibration แบบกล้องอยู่กับที่ (eye-to-hand) จากจุด 3D — ไม่พึ่ง ROS (ทดสอบบน PC ได้)

ป้ายติดอยู่ที่มือ ตำแหน่งป้ายในเฟรมฐานแขน = R_i m + t_i
  (R_i, t_i = ท่าของ link ปลายแขนจาก TF · m = ป้ายอยู่ตรงไหนบนมือ ในเฟรมปลายแขน — ไม่ต้องรู้ล่วงหน้า)
กล้องวัดตำแหน่งป้ายได้ p_i (เฟรมกล้อง) -> หา R, t (เฟรมกล้อง -> ฐานแขน) และ m ที่ทำให้
  R p_i + t ใกล้ R_i m + t_i ที่สุด (least squares)
"""
import numpy as np


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def exp_so3(w):
    """เวกเตอร์หมุน -> เมทริกซ์หมุน (Rodrigues)"""
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3) + skew(w)
    k = skew(w / th)
    return np.eye(3) + np.sin(th) * k + (1 - np.cos(th)) * k @ k


def kabsch(P, Q):
    """R, t ที่ทำให้ R P + t ใกล้ Q ที่สุด (P, Q: Nx3)"""
    cp, cq = P.mean(0), Q.mean(0)
    U, _, Vt = np.linalg.svd((P - cp).T @ (Q - cq))
    R = Vt.T @ np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))]) @ U.T
    return R, cq - R @ cp


def marker_in_base(Rs, ts, m):
    return np.einsum("nij,j->ni", Rs, m) + ts


def _linear_guess(P, Rs, ts):
    """เดาเริ่มแบบเชิงเส้น: ให้ R เป็นเมทริกซ์อิสระ 9 ค่า + t + m (15 ตัวแปร) แล้วค่อยบังคับ R ให้เป็นการหมุนจริง"""
    n = len(P)
    A = np.zeros((3 * n, 15))
    for i in range(n):
        for r in range(3):
            A[3 * i + r, 3 * r:3 * r + 3] = P[i]
            A[3 * i + r, 9 + r] = 1.0
            A[3 * i + r, 12:15] = -Rs[i][r]
    x = np.linalg.lstsq(A, ts.reshape(-1), rcond=None)[0]
    U, _, Vt = np.linalg.svd(x[:9].reshape(3, 3))
    R = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
    return R, x[12:15]


def _refine(R, m, P, Rs, ts, iters=100):
    """Gauss-Newton บน (หมุน 3, เลื่อน 3, ตำแหน่งป้าย 3)"""
    R, t = kabsch(P, marker_in_base(Rs, ts, m))
    n = len(P)
    for _ in range(iters):
        RP = P @ R.T
        r = (RP + t - marker_in_base(Rs, ts, m)).reshape(-1)
        J = np.zeros((3 * n, 9))
        for i in range(n):
            J[3 * i:3 * i + 3, 0:3] = -skew(RP[i])    # R <- exp(d) R  =>  d(R p) = -[R p]x d
            J[3 * i:3 * i + 3, 3:6] = np.eye(3)
            J[3 * i:3 * i + 3, 6:9] = -Rs[i]
        d = np.linalg.lstsq(J, -r, rcond=None)[0]
        R, t, m = exp_so3(d[:3]) @ R, t + d[3:6], m + d[6:9]
        if np.linalg.norm(d) < 1e-10:
            break
    res = np.linalg.norm(P @ R.T + t - marker_in_base(Rs, ts, m), axis=1)
    return R, t, m, res


def solve(P, Rs, ts):
    """P: ป้ายในเฟรมกล้อง (Nx3) · Rs, ts: ท่าปลายแขนในเฟรมฐาน (Nx3x3, Nx3)
    คืน R, t (กล้อง -> ฐาน: X_ฐาน = R X_กล้อง + t), m (ป้ายในเฟรมปลายแขน), ระยะคลาดของแต่ละท่า (m)"""
    P, Rs, ts = np.asarray(P, float), np.asarray(Rs, float), np.asarray(ts, float)
    best = None
    # เริ่มสองแบบ (เชิงเส้น / ถือว่าป้ายอยู่ที่ปลายแขนพอดี) เอาอันที่คลาดน้อยกว่า — กันติดค่าต่ำสุดเฉพาะที่
    for R0, m0 in (_linear_guess(P, Rs, ts), (None, np.zeros(3))):
        if R0 is None:
            R0, _ = kabsch(P, ts)
        out = _refine(R0, m0, P, Rs, ts)
        if best is None or np.sqrt(np.mean(out[3] ** 2)) < np.sqrt(np.mean(best[3] ** 2)):
            best = out
    return best


def spread(Rs, ts):
    """ความหลากหลายของท่า: (ช่วงตำแหน่งแคบสุดตามแกนหลัก m, มุมข้อมือต่างกันมากสุด องศา)
    ตำแหน่งแบนเป็นเส้น -> หมุนกล้องไม่ออก · ข้อมือแทบไม่หมุน -> หาตำแหน่งป้ายบนมือไม่ได้"""
    ts = np.asarray(ts, float)
    ev = np.linalg.eigvalsh(np.cov(ts.T)) if len(ts) > 2 else np.zeros(3)
    ang = max((np.degrees(np.arccos(np.clip((np.trace(a.T @ b) - 1) / 2, -1, 1)))
               for i, a in enumerate(Rs) for b in Rs[i + 1:]), default=0.0)
    return float(2 * np.sqrt(max(ev[1], 0))), float(ang)


def mat_to_quat(R):
    """เมทริกซ์หมุน -> (x, y, z, w)"""
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    y = np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    z = np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    x = np.copysign(x, R[2, 1] - R[1, 2])
    y = np.copysign(y, R[0, 2] - R[2, 0])
    z = np.copysign(z, R[1, 0] - R[0, 1])
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def quat_to_mat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
