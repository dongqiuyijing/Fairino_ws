import os

from glob import glob

from setuptools import (
    find_packages,
    setup,
)


package_name = "fairino3_gazebo"


setup(

    name=package_name,

    version="0.0.1",

    packages=find_packages(
        exclude=["test"]
    ),

    data_files=[

        (
            "share/ament_index/"
            "resource_index/packages",

            [
                "resource/"
                + package_name
            ],
        ),

        (
            "share/"
            + package_name,

            [
                "package.xml"
            ],
        ),

        (
            os.path.join(
                "share",
                package_name,
                "launch",
            ),

            glob(
                "launch/*.launch.py"
            ),
        ),

        (
            os.path.join(
                "share",
                package_name,
                "urdf",
            ),

            glob(
                "urdf/*"
            ),
        ),

        (
            os.path.join(
                "share",
                package_name,
                "config",
            ),

            glob(
                "config/*"
            ),
        ),

        (
            os.path.join(
                "share",
                package_name,
                "worlds",
            ),

            glob(
                "worlds/*"
            ),
        ),
    ],

    install_requires=[
        "setuptools"
    ],

    zip_safe=True,

    maintainer="user",

    maintainer_email=
        "user@example.com",

    description=
        "FAIRINO FR3 Gazebo simulation",

    license="Apache-2.0",

    entry_points={

        "console_scripts": [

            "joint_demo = "
            "fairino3_gazebo."
            "joint_demo:main",

            "joint_axis_test = "
            "fairino3_gazebo."
            "joint_axis_test:main",

            "pick_inspect_demo = "
            "fairino3_gazebo."
            "pick_inspect_demo:main",

            "grasp_reset = "
            "fairino3_gazebo."
            "grasp_reset:main",
        ],
    },
)
