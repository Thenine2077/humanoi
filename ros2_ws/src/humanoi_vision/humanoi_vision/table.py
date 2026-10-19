"""ผิวโต๊ะจาก depth + ตำแหน่งของที่วางบนโต๊ะโดยไม่ต้องใช้ depth ที่ตัวของ — ไม่พึ่ง ROS (ทดสอบบน PC ได้)

ใช้กับของที่อยู่ใกล้กว่าระยะต่ำสุดของ depth (~55-60 cm) ซึ่งเป็นพื้นที่ที่แขนเอื้อมถึง:
- ผิวโต๊ะ = แผ่นเรียบที่จุด depth ส่วนใหญ่วางอยู่และ "หงายขึ้น" หากล้อง (ไม่ใช่ผนัง) — โต๊ะเรียบทั้งแผ่น
  depth วัดส่วนไกลได้ก็รู้ส่วนใกล้ที่วัดไม่ได้ด้วย
- ของบนโต๊ะ: เส้นจากกล้องผ่านขอบล่างกลางกรอบ YOLO (ก้นของด้านหน้า) ชนผิวโต๊ะ = จุดเริ่ม แล้วมองของเป็น
  ทรงกระบอกตั้งบนโต๊ะ หาตำแหน่งฐาน รัศมี ความสูง ที่ฉายลงภาพแล้วได้กรอบตรงกับกรอบ YOLO
ทุกอย่างอยู่ในเฟรมกล้อง optical (X ขวา, Y ลง, Z ไปข้างหน้า) หน่วยเมตร
"""
import cv2
import numpy as np


def rays(uv, K, D):
    """พิกเซล (Nx2) -> ทิศรังสีจากกล้อง (Nx3, z = 1) หลังแก้ความโค้งเลนส์"""
    xy = cv2.undistortPoints(np.asarray(uv, np.float64).reshape(-1, 1, 2), K, D).reshape(-1, 2)
    return np.column_stack([xy, np.ones(len(xy))])


def fit_table(points, tol=0.008, iters=200, min_up=0.3, rng=None):
    """จุด 3D (Nx3) -> (n, d, สัดส่วนจุดบนโต๊ะ) ที่ n·X = d และ n ชี้จากโต๊ะขึ้นหากล้อง — หรือ None

    นับเฉพาะแผ่นที่หงายขึ้น (n_y < -min_up — แกน Y ของเฟรม optical ชี้ลง) จึงไม่เลือกผนังหรือด้านข้างของ"""
    if len(points) < 200:
        return None
    rng = rng if rng is not None else np.random.default_rng()
    best, best_n = None, 0
    for a, b, c in points[rng.integers(0, len(points), (iters, 3))]:
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n /= nn
        if n @ a > 0:          # ให้ชี้เข้าหากล้อง (กล้องอยู่ที่จุดกำเนิด)
            n = -n
        if n[1] > -min_up:     # ไม่หงายขึ้น = ผนัง / ด้านข้างของ
            continue
        cnt = np.count_nonzero(np.abs((points - a) @ n) < tol)
        if cnt > best_n:
            best, best_n = (n, a), cnt
    if best is None or best_n < max(200, 0.2 * len(points)):
        return None
    n, a = best
    inl = points[np.abs((points - a) @ n) < tol]
    c = inl.mean(0)
    n = np.linalg.svd(inl - c)[2][2]
    if n @ c > 0:
        n = -n
    return n, float(n @ c), len(inl) / len(points)


def hit(ray, plane):
    """จุดที่รังสีชนระนาบ หรือ None (ขนาน / อยู่หลังกล้อง)"""
    n, d = plane[0], plane[1]
    den = n @ ray
    if abs(den) < 1e-6 or d / den <= 0:
        return None
    return ray * (d / den)


def _cylinder_box(B, r, h, n, e1, e2, K, D):
    """กรอบในภาพ (x1, y1, x2, y2) ของทรงกระบอกตั้งบนโต๊ะ: ฐานกลางที่ B รัศมี r สูง h"""
    th = np.linspace(0, 2 * np.pi, 36, endpoint=False)
    ring = np.outer(np.cos(th), e1) + np.outer(np.sin(th), e2)
    pts = np.vstack([B + r * ring, B + r * ring + h * n])
    uv = cv2.projectPoints(pts, np.zeros(3), np.zeros(3), K, D)[0].reshape(-1, 2)
    return np.array([uv[:, 0].min(), uv[:, 1].min(), uv[:, 0].max(), uv[:, 1].max()])


def object_on_table(box, K, D, plane, img_w, img_h, margin=2):
    """ของที่ตั้งบนโต๊ะ -> (ใจกลางของที่ครึ่งความสูง, กว้าง, สูง) หรือ None

    มองของเป็นทรงกระบอกตั้งบนโต๊ะ แล้วหา ตำแหน่งฐาน (2) + รัศมี + ความสูง ที่ฉายลงภาพแล้วได้กรอบตรงกับกรอบ
    ของ YOLO มากที่สุด (กรอบ 4 ด้าน = 4 สมการ) — คิดแบบง่ายจากมุมกรอบตรงๆ คลาดได้ 1-4 cm เพราะหัวของ
    ที่อยู่ใกล้กล้องกว่าจะดูกว้างและเอียงออกด้านข้างตามมุมมอง
    None เมื่อก้นของเลยขอบล่างภาพ (ไม่เห็นจุดที่วางบนโต๊ะ) หรือเส้นไม่ชนโต๊ะ"""
    x1, y1, x2, y2 = box
    if y2 >= img_h - margin:
        return None
    n = plane[0]
    front = hit(rays([((x1 + x2) / 2, y2)], K, D)[0], plane)
    if front is None:
        return None
    e1 = front - (front @ n) * n               # ทิศถอยห่างจากกล้องตามแนวโต๊ะ
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    use = np.array([x1 > margin, y1 > margin, x2 < img_w - margin, True])  # ด้านที่ติดขอบภาพใช้ไม่ได้
    obs = np.array(box, float)
    r0 = float(np.clip((x2 - x1) / K[0, 0] * front[2] / 2 * 0.8, 0.01, 0.10))
    x = np.array([r0, 0.0, r0, 0.1])           # ฐานเลื่อนจากขอบหน้า (ถอยหลัง, ด้านข้าง), รัศมี, สูง

    def residual(v):
        return (_cylinder_box(front + v[0] * e1 + v[1] * e2, v[2], v[3], n, e1, e2, K, D) - obs)[use]

    lam = 1e-3
    f = residual(x)
    for _ in range(40):
        J = np.column_stack([(residual(x + dx) - f) / 1e-4 for dx in np.eye(4) * 1e-4])
        A = J.T @ J
        step = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-9), -J.T @ f)
        x_new = x + step
        x_new[2] = np.clip(x_new[2], 0.005, 0.15)
        x_new[3] = np.clip(x_new[3], 0.01, 0.6)
        f_new = residual(x_new)
        if f_new @ f_new < f @ f:
            x, f, lam = x_new, f_new, lam * 0.3
            if np.linalg.norm(step) < 1e-6:
                break
        else:
            lam *= 10
    base = front + x[0] * e1 + x[1] * e2
    return base + n * x[3] / 2, 2 * float(x[2]), float(x[3])
