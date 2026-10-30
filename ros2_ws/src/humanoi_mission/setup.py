import os
from glob import glob

from setuptools import find_packages, setup

package_name = "humanoi_mission"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="User",
    maintainer_email="nayzalovehrk@gmail.com",
    description="ลำดับงานของหุ่น: หยิบกล่อง พับแขน รอคนยืนยัน ฐานวิ่ง วาง",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mission = humanoi_mission.mission:main",
            "show = humanoi_mission.show:main",
            "show_setup = humanoi_mission.show_setup:main",
            "show_mock = humanoi_mission.show_mock:main",
        ],
    },
)
