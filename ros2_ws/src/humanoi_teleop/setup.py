from setuptools import find_packages, setup

package_name = "humanoi_teleop"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="User",
    maintainer_email="nayzalovehrk@gmail.com",
    description="คีย์บอร์ด teleop สำหรับฐาน mecanum ของหุ่น Humanoi",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "keyboard_teleop = humanoi_teleop.keyboard_teleop:main",
            "square_drive = humanoi_teleop.square_drive:main",
            "walker_patrol = humanoi_teleop.walker_patrol:main",
        ],
    },
)
