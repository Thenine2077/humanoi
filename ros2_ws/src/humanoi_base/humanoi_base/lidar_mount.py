"""ตำแหน่ง lidar บนฐานจริง — อ่าน/เขียน config/lidar_mount.yaml (ที่เดียว ทุกตัวใช้ค่านี้)

lidar.launch.py (static TF + scan_mask) · กันชน (guard) และ arm drive (base_motion) ตอนไม่มี TF
x / y / z วัดด้วยตลับเมตร · yaw_deg: arm lidar-check วัดและบันทึกให้
"""
import math
import os

import yaml

FALLBACK = {"x": 0.175, "y": 0.0, "z": 0.235, "yaw_deg": 0.0}


def path():
    """config/lidar_mount.yaml ของแพ็กเกจ (symlink-install = ไฟล์ใน src)"""
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory("humanoi_base"), "config", "lidar_mount.yaml")
    except Exception:  # noqa: BLE001
        p = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "config", "lidar_mount.yaml")
    return os.path.realpath(p)


def load():
    """{x, y, z (m), yaw_deg, yaw (rad)} — ไม่มีไฟล์/อ่านไม่ได้ = ค่าตั้งต้น"""
    m = dict(FALLBACK)
    try:
        with open(path(), encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        m.update({k: float(v) for k, v in data.items() if k in FALLBACK})
    except (OSError, ValueError, TypeError, yaml.YAMLError):
        pass
    m["yaw"] = math.radians(m["yaw_deg"])
    return m


def save_yaw(yaw_deg):
    """แก้แค่บรรทัด yaw_deg คอมเมนต์ในไฟล์อยู่ครบ"""
    p = path()
    with open(p, encoding="utf-8") as f:
        lines = f.read().splitlines()
    for i, ln in enumerate(lines):
        if ln.split(":")[0].strip() == "yaw_deg":
            comment = ln[ln.index("#"):] if "#" in ln else ""
            lines[i] = f"yaw_deg: {yaw_deg:.1f}" + (f"  {comment}" if comment else "")
    tmp = p + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    os.replace(tmp, p)
