"""
Launch the FR3 pose-sequence control node.

Gazebo and move_group must already be running in other terminals.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Create the launch description for pose_sequence."""
    poses_default = os.path.join(
        get_package_share_directory("fr_control"),
        "config",
        "poses.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="与 Gazebo 联用时必须为 true",
            ),
            DeclareLaunchArgument(
                "poses_file",
                default_value=poses_default,
                description="姿态序列 YAML 路径",
            ),
            Node(
                package="fr_control",
                executable="pose_sequence",
                name="fr_pose_sequence",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"),
                            value_type=bool,
                        ),
                        "poses_file": LaunchConfiguration("poses_file"),
                    }
                ],
            ),
        ]
    )
