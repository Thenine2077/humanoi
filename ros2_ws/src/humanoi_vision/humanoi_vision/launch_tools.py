"""ส่วนที่ launch หลายไฟล์ใช้ร่วมกัน"""
import os

import yaml
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def camera_tf(context):
    """TF ฐานแขน -> camera_link จากผล hand_eye_calibrate (ไม่มีไฟล์ = ข้าม แค่บอกใน log) — ใช้กับ OpaqueFunction"""
    path = LaunchConfiguration("camera_tf").perform(context)
    if not path or not os.path.exists(path):
        return [LogInfo(msg=f"ยังไม่มีผล hand-eye ({path}) — พิกัดอยู่ในเฟรมกล้องอย่างเดียว")]
    with open(path) as f:
        c = yaml.safe_load(f)
    args = []
    for k in ("x", "y", "z", "qx", "qy", "qz", "qw"):
        args += [f"--{k}", str(c[k])]
    return [Node(package="tf2_ros", executable="static_transform_publisher", name="camera_tf",
                 arguments=args + ["--frame-id", c["parent_frame"], "--child-frame-id", c["child_frame"]])]
