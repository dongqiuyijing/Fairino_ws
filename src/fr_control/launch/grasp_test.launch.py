"""
Launch the FR3 known-object grasp test.

Gazebo (sim.launch.py) and move_group must already be running.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Create the launch description for grasp_test."""
    config_default = os.path.join(
        get_package_share_directory("fr_control"),
        "config",
        "grasp.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="与 Gazebo 联用时必须为 true",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=config_default,
                description="抓取配置 YAML 路径",
            ),
            DeclareLaunchArgument(
                "gripper_backend",
                default_value="gazebo",
                description="夹爪后端：gazebo 或 real",
            ),
            DeclareLaunchArgument(
                "sim_grasp_backend",
                default_value="none",
                description="仿真抓取辅助：必须为 none（禁止 attach 掩盖碰撞）",
            ),
            Node(
                package="fr_control",
                executable="grasp_test",
                name="fr_grasp_test",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"),
                            value_type=bool,
                        ),
                        "config_file": LaunchConfiguration("config_file"),
                        "gripper_backend": LaunchConfiguration(
                            "gripper_backend"
                        ),
                        "sim_grasp_backend": LaunchConfiguration(
                            "sim_grasp_backend"
                        ),
                    }
                ],
            ),
        ]
    )
