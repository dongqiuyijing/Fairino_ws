from setuptools import find_packages, setup

package_name = "fr3_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/fr3_teleop"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/teleop.yaml", "config/joystick.yaml", "config/teleop_controllers.yaml", "config/ros2_controllers_offline.yaml", "config/offline_initial_positions.yaml", "config/servo_arm_a.yaml", "config/servo_arm_b.yaml"]),
        (f"share/{package_name}/launch", ["launch/teleop.launch.py", "launch/teleop_joystick.launch.py", "launch/teleop_servo.launch.py", "launch/teleop_servo_offline_test.launch.py"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Fairino workspace",
    maintainer_email="cyberbraindualarm@todo.todo",
    description="Safety-first dual FR3 GUI teleoperation manager",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "teleop_manager = fr3_teleop.manager:main",
            "teleop_gui = fr3_teleop.gui:main",
            "joystick_adapter = fr3_teleop.joystick_adapter:main",
            "offline_servo_probe = fr3_teleop.offline_servo_probe:main",
        ],
    },
)
