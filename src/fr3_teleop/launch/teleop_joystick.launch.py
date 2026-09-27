"""Joy driver, joystick adapter, and optionally the mock teleop manager.

Pass start_manager:=false when teleop.launch.py already owns the single manager.
This launch does not start the GUI and does not start a real motion backend.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_manager = LaunchConfiguration("start_manager")
    return LaunchDescription([
        DeclareLaunchArgument(
            "start_manager",
            default_value="true",
            description="Start Teleop Manager. Set false if another launch already started the only manager.",
        ),
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[{
                "device_id": 0,
                "device_name": "Xbox 360 Controller",
                "deadzone": 0.0,
                "autorepeat_rate": 20.0,
                "sticky_buttons": False,
                "coalesce_interval_ms": 1,
            }],
        ),
        Node(
            package="fr3_teleop",
            executable="teleop_manager",
            output="screen",
            condition=IfCondition(start_manager),
        ),
        Node(package="fr3_teleop", executable="joystick_adapter", output="screen"),
    ])
