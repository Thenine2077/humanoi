import os
from glob import glob

from setuptools import find_packages, setup

package_name = "humanoi_vision"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="User",
    maintainer_email="nayzalovehrk@gmail.com",
    description="YOLO บนการ์ด Hailo-10H ตรวจจับวัตถุจากภาพสีของกล้อง Astra Pro",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "yolo_node = humanoi_vision.yolo_node:main",
            "calibrate_color = humanoi_vision.calibrate_color:main",
            "check_depth_scale = humanoi_vision.check_depth_scale:main",
            "hand_eye_calibrate = humanoi_vision.hand_eye_calibrate:main",
            "tray_detector = humanoi_vision.tray_detector:main",
            "tray_teach = humanoi_vision.tray_teach:main",
        ],
    },
)
