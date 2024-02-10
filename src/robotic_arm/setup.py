from setuptools import setup
import os
from glob import glob

package_name = "robotic_arm"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "msg"), glob("msg/*")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "world"), glob("world/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Husen Yusuf",
    maintainer_email="husenyusuf876@@gmail.com",
    description="A robotic arm with vision aided pick and place capability",
    license="MiT License",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "object_detector = robotic_arm.object_detector:main",
            "arm_commander = robotic_arm.arm_commander:main",
            "state_machine = robotic_arm.state_machine:main",
            "robot_controller = robotic_arm.test_controller:main",
        ],
    },
)
