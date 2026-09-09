import os
from glob import glob

from setuptools import find_packages, setup

package_name = "fr_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cyberbraindualarm",
    maintainer_email="cyberbraindualarm@todo.todo",
    description="FAIRINO FR3 MoveIt application nodes",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pose_sequence = fr_control.pose_sequence:main",
            "gripper_test = fr_control.gripper_test:main",
            "gripper_contact_test = fr_control.gripper_contact_test:main",
            "collision_drop_test = fr_control.collision_drop_test:main",
            "grasp_test = fr_control.grasp_test:main",
            "stage4_inspection_test = fr_control.stage4_inspection_test:main",
        ],
    },
)
