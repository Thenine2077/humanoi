"""ตำแหน่งถาด (กล่อง) จากป้าย ArUco ที่แปะกลางพื้นกล่อง — คณิตล้วน ไม่พึ่ง ROS (ทดสอบบน PC ได้)

เฟรมป้าย (ตาม SOLVEPNP_IPPE_SQUARE / ArUco): จุดกลางป้าย, x = ขวาของป้าย, y = หัวป้าย (ขอบบนกระดาษ),
z = ตั้งฉากพื้นกล่องชี้ขึ้นหากล้อง · หน่วยเมตร
"""
import cv2
import numpy as np

from humanoi_vision.hand_eye import mat_to_quat, quat_to_mat


def object_points(size):
    """มุมป้าย 4 มุมในเฟรมป้าย เรียงตามลำดับมุมที่ ArUco คืน (บนซ้าย บนขวา ล่างขวา ล่างซ้าย)"""
    s = size / 2
    return np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float64)


def marker_pose(corners, K, D, size):
    """มุมป้ายในภาพ (4x2 px) -> (R, t, คลาดตอนฉายกลับ px) ของป้ายในเฟรมกล้อง (optical)"""
    c = np.asarray(corners, np.float64).reshape(4, 2)
    obj = object_points(size)
    ok, rvec, tvec = cv2.solvePnP(obj, c, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
    err = float(np.linalg.norm(proj.reshape(4, 2) - c, axis=1).mean())
    return cv2.Rodrigues(rvec)[0], tvec.ravel(), err


def facing_camera(R, t, max_tilt_deg=70.0):
    """ป้ายหันหน้าเข้ากล้องพอ (มองเฉียงมากไป = มุมที่ PnP คำนวณได้ไม่แม่น)"""
    n = R[:, 2]
    view = -np.asarray(t, float) / np.linalg.norm(t)
    return float(n @ view) >= np.cos(np.radians(max_tilt_deg))


def average_pose(Rs, ts, reject_mm=10.0):
    """เฉลี่ยหลายเฟรม ตัดเฟรมที่ตำแหน่งห่างค่ากลาง (median) เกิน reject_mm ทิ้ง

    -> (R, t, ค่าแกว่งของตำแหน่ง mm (ค่าสูงสุดจากค่าเฉลี่ย), จำนวนเฟรมที่ใช้) หรือ None ถ้าไม่มีเฟรม
    """
    if not len(ts):
        return None
    ts = np.asarray(ts, float)
    med = np.median(ts, axis=0)
    keep = np.linalg.norm(ts - med, axis=1) * 1000 <= reject_mm
    if not keep.any():
        return None
    t = ts[keep].mean(0)
    qs = np.array([mat_to_quat(R) for R, k in zip(Rs, keep) if k])
    qs *= np.sign(qs @ qs[0])[:, None]  # q กับ -q คือการหมุนเดียวกัน — จัดให้อยู่ฝั่งเดียวก่อนเฉลี่ย
    q = qs.mean(0)
    q /= np.linalg.norm(q)
    spread = float(np.linalg.norm(ts[keep] - t, axis=1).max() * 1000)
    return quat_to_mat(*q), t, spread, int(keep.sum())


def mat(R, t):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


def inv(T):
    R, t = T[:3, :3], T[:3, 3]
    return mat(R.T, -R.T @ t)


def log_so3(R):
    """เมทริกซ์หมุน -> เวกเตอร์หมุน (แกน × มุม rad)"""
    c = np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)
    th = np.arccos(c)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if th < 1e-9:
        return w / 2
    if np.pi - th < 1e-6:  # ใกล้ 180°: เอาแกนจากคอลัมน์ที่ใหญ่สุดของ (R + I) / 2
        B = (R + np.eye(3)) / 2
        a = B[:, np.argmax(np.diag(B))]
        return th * a / np.linalg.norm(a)
    return th * w / (2 * np.sin(th))


def exp_so3(w):
    th = np.linalg.norm(w)
    K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    if th < 1e-12:
        return np.eye(3) + K
    K /= th
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


ANGLE_WEIGHT = 0.10  # m ต่อ rad — คลาดมุม 1 rad นับเท่าคลาดตำแหน่ง 10 cm (ประมาณระยะนิ้วถึงข้อมือ)


def _apply(T, x):
    """ขยับ T เล็กน้อยด้วย x (rotvec 3 + translation 3)"""
    return mat(exp_so3(x[:3]) @ T[:3, :3], T[:3, 3] + x[3:])


def grasp_residuals(X, Y, As, Cs):
    """ท่าจับที่ทำนาย X·C·Y เทียบกับที่สอนจริง A -> (คลาดตำแหน่ง m, คลาดมุม rad) ต่อครั้ง"""
    pos, ang = [], []
    for A, C in zip(As, Cs):
        P = X @ C @ Y
        pos.append(np.linalg.norm(P[:3, 3] - A[:3, 3]))
        ang.append(np.linalg.norm(log_so3(A[:3, :3].T @ P[:3, :3])))
    return np.array(pos), np.array(ang)


def solve_grasp_camera(As, Cs, iters=60):
    """สอนจับกล่องหลายตำแหน่ง -> หากล้องเทียบแขน X (ฐานแขน <- กล้อง) และจุดจับเทียบป้าย Y (ป้าย <- ข้อมือ)

    As = ท่าข้อมือ (link_6) ในเฟรมฐานแขนตอนจับหูกล่อง (4x4) · Cs = ป้ายในเฟรมกล้องตอนนั้น (4x4)
    ความสัมพันธ์ A = X · C · Y (หาได้เมื่อกล่องเลื่อน 2 ทิศ และ/หรือหมุน ต่างกันพอ)
    เริ่มจากจับคู่ตำแหน่ง (Kabsch ป้าย -> ข้อมือ) แล้ว Gauss-Newton ทั้ง 12 ค่า
    -> (X, Y, คลาดตำแหน่งต่อครั้ง m, คลาดมุมต่อครั้ง rad, ค่าบอกว่าข้อมูลพอไหม (0..1))
    """
    As, Cs = [np.asarray(a, float) for a in As], [np.asarray(c, float) for c in Cs]
    P = np.array([C[:3, 3] for C in Cs])
    Q = np.array([A[:3, 3] for A in As])
    # ข้อมูลพอไหม: ป้ายต้องเลื่อนไปมากกว่าหนึ่งทิศ (ค่าเอกฐานลำดับ 2 เทียบลำดับ 1)
    sv = np.linalg.svd(P - P.mean(0), compute_uv=False)
    spread_ok = float(sv[1] / sv[0]) if sv[0] > 1e-9 else 0.0
    U, _, Vt = np.linalg.svd((P - P.mean(0)).T @ (Q - Q.mean(0)))
    R0 = Vt.T @ np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))]) @ U.T
    X = mat(R0, Q.mean(0) - R0 @ P.mean(0))
    Ys = [inv(C) @ inv(X) @ A for A, C in zip(As, Cs)]
    qs = np.array([mat_to_quat(Y[:3, :3]) for Y in Ys])
    qs *= np.sign(qs @ qs[0])[:, None]
    q = qs.mean(0)
    Y = mat(quat_to_mat(*(q / np.linalg.norm(q))), np.mean([Y[:3, 3] for Y in Ys], axis=0))

    def res(X, Y):
        out = []
        for A, C in zip(As, Cs):
            Pr = X @ C @ Y
            out += list(Pr[:3, 3] - A[:3, 3])
            out += list(ANGLE_WEIGHT * log_so3(A[:3, :3].T @ Pr[:3, :3]))
        return np.array(out)

    lam = 1e-3
    r = res(X, Y)
    for _ in range(iters):
        J = np.zeros((len(r), 12))
        for k in range(12):
            d = np.zeros(12)
            d[k] = 1e-6
            J[:, k] = (res(_apply(X, d[:6]), _apply(Y, d[6:])) - r) / 1e-6
        step = -np.linalg.solve(J.T @ J + lam * np.eye(12), J.T @ r)
        Xn, Yn = _apply(X, step[:6]), _apply(Y, step[6:])
        rn = res(Xn, Yn)
        if rn @ rn < r @ r:
            X, Y, r, lam = Xn, Yn, rn, lam * 0.3
            if np.linalg.norm(step) < 1e-9:
                break
        else:
            lam *= 10
    pos, ang = grasp_residuals(X, Y, As, Cs)
    return X, Y, pos, ang, spread_ok


def solve_contact_point(As, Cs, iters=80):
    """สอนจับกล่องหลายครั้ง โดยข้อมือเอียงต่างกันได้ — ขอแค่ "จุดบนนิ้ว" แตะ "จุดเดิมบนหูกล่อง" ทุกครั้ง

    จุดบนนิ้ว p (เฟรมข้อมือ link_6) = จุดบนหู q (เฟรมป้าย) ทุกครั้ง:  A·p = X·C·q
    ไม่รู้ X (ฐานแขน <- กล้อง, 6 ค่า), p (3), q (3) = 12 ค่า · แต่ละครั้งให้ 3 สมการ -> ต้อง >= 5 ครั้ง
    ข้อมือเอียงต่างกัน = หา p ได้, กล่องหมุนต่างกัน = หา q ได้ (ไม่หมุน: q กับตำแหน่งกล้องแยกไม่ออก
    — ไม่มีผลถ้ากล่องวางตรงเหมือนตอนสอน)
    -> (X, p, q, คลาดต่อครั้ง m, การกระจาย 0..1)
    """
    As, Cs = [np.asarray(a, float) for a in As], [np.asarray(c, float) for c in Cs]
    P = np.array([C[:3, 3] for C in Cs])
    Q = np.array([A[:3, 3] for A in As])
    sv = np.linalg.svd(P - P.mean(0), compute_uv=False)
    spread_ok = float(sv[1] / sv[0]) if sv[0] > 1e-9 else 0.0
    U, _, Vt = np.linalg.svd((P - P.mean(0)).T @ (Q - Q.mean(0)))
    R = Vt.T @ np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))]) @ U.T
    x = np.concatenate([np.zeros(3), Q.mean(0) - R @ P.mean(0), np.zeros(6)])  # rotvec แก้ R, t, p, q
    R0 = R

    def res(x):
        Rx = exp_so3(x[:3]) @ R0
        t, p, q = x[3:6], x[6:9], x[9:12]
        return np.concatenate([Rx @ (C[:3, :3] @ q + C[:3, 3]) + t - (A[:3, :3] @ p + A[:3, 3])
                               for A, C in zip(As, Cs)])

    lam, r = 1e-3, res(x)
    for _ in range(iters):
        J = np.zeros((len(r), 12))
        for k in range(12):
            d = np.zeros(12)
            d[k] = 1e-6
            J[:, k] = (res(x + d) - r) / 1e-6
        step = -np.linalg.solve(J.T @ J + lam * np.eye(12), J.T @ r)
        rn = res(x + step)
        if rn @ rn < r @ r:
            x, r, lam = x + step, rn, lam * 0.3
            if np.linalg.norm(step) < 1e-10:
                break
        else:
            lam *= 10
    X = mat(exp_so3(x[:3]) @ R0, x[3:6])
    err = np.linalg.norm(r.reshape(-1, 3), axis=1)
    return X, x[6:9], x[9:12], err, spread_ok


def nearest_grasp(Cs, Gs, C_new):
    """ท่าจับที่สอนไว้ซึ่งกล่องวางใกล้ตำแหน่งใหม่ที่สุด (Gs = ท่าข้อมือเทียบป้ายของแต่ละครั้ง) -> ข้อมือ = X·C_new·G"""
    d = [np.linalg.norm(C[:3, 3] - C_new[:3, 3]) + 0.1 * np.linalg.norm(log_so3(C[:3, :3].T @ C_new[:3, :3]))
         for C in Cs]
    return Gs[int(np.argmin(d))], int(np.argmin(d))


def turn_deg(R_a, R_b):
    """กล่องหมุนต่างกันกี่องศารอบแกนตั้งฉากพื้นกล่อง (z ของป้าย) — ไว้บอกว่าวางเบี้ยวจากตอนสอนเท่าไหร่"""
    x_b = R_a.T @ R_b[:, 0]  # แกน x ของ b ในเฟรมป้าย a
    return float(np.degrees(np.arctan2(x_b[1], x_b[0])))
