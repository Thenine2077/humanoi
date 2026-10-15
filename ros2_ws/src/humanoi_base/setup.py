import os
from glob import glob

from setuptools import find_packages, setup

package_name = "humanoi_base"

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
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="User",
    maintainer_email="nayzalovehrk@gmail.com",
    description="สะพาน /cmd_vel -> serial -> Arduino relay H-bridge",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "relay_bridge = humanoi_base.relay_bridge:main",
            "scan_restamp = humanoi_base.scan_restamp:main",
            "rc_arm_trigger = humanoi_base.rc_arm_trigger:main",
        ],
    },
)
