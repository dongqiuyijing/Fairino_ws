"""
MoveIt move_group launch for Gazebo.

Official move_group aborts trajectories that run slightly longer than the
plan. Gazebo tracking lag then shows up as CONTROL_FAILED. This overlay
keeps the official robot/MoveIt configs and only relaxes execution timing.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def generate_launch_description() -> LaunchDescription:
    """Launch official move_group with Gazebo-tolerant execution timing."""
    official = os.path.join(
        get_package_share_directory("fairino3_v6_moveit2_config"),
        "launch",
        "move_group.launch.py",
    )
    return LaunchDescription(
        [
            SetParameter(
                name="trajectory_execution.execution_duration_monitoring",
                value=False,
            ),
            SetParameter(
                name="trajectory_execution.allowed_execution_duration_scaling",
                value=5.0,
            ),
            SetParameter(
                name="trajectory_execution.allowed_goal_duration_margin",
                value=10.0,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(official),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration(
                        "use_sim_time", default="true"
                    ),
                }.items(),
            ),
        ]
    )
