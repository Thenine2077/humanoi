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


def turn_deg(R_a, R_b):
    """กล่องหมุนต่างกันกี่องศารอบแกนตั้งฉากพื้นกล่อง (z ของป้าย) — ไว้บอกว่าวางเบี้ยวจากตอนสอนเท่าไหร่"""
    x_b = R_a.T @ R_b[:, 0]  # แกน x ของ b ในเฟรมป้าย a
    return float(np.degrees(np.arctan2(x_b[1], x_b[0])))
