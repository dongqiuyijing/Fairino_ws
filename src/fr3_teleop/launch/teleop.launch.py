"""Launch the independent manager and desktop GUI in mock mode by default."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="fr3_teleop", executable="teleop_manager", output="screen"),
        Node(package="fr3_teleop", executable="teleop_gui", output="screen"),
    ])
