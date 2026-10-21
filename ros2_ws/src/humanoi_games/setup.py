import os
from glob import glob

from setuptools import find_packages, setup

package_name = "humanoi_games"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "models"), glob("models/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="User",
    maintainer_email="nayzalovehrk@gmail.com",
    description="เกมสนุกๆ ของหุ่น Humanoi — เป่ายิ้งฉุบด้วยแขน AR4 + มือ AmazingHand + กล้อง",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rps_game = humanoi_games.rps_game:main",
            "rps_vision = humanoi_games.rps_vision:main",
            "amazinghand_node = humanoi_games.amazinghand_node:main",
            "hand_calibrate = humanoi_games.hand_calibrate:main",
            "hand_teach = humanoi_games.hand_teach:main",
            "rps_hud = humanoi_games.rps_hud:main",
        ],
    },
)
