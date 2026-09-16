"""
Launch the FR3 real known-pose grasp test.

real_bringup.launch.py must already be running.
Does not start hardware, MoveIt, or Gazebo.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Create the launch description for stage4_real_grasp_test."""
    config_default = os.path.join(
        get_package_share_directory("fr_control"),
        "config",
        "stage4_config.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="真机必须为 false",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=config_default,
                description="阶段 4 YAML 配置路径",
            ),
            DeclareLaunchArgument(
                "gripper_backend",
                default_value="real",
                description="真机夹爪后端，必须为 real",
            ),
            DeclareLaunchArgument(
                "skip_confirm",
                default_value="false",
                description="true 时跳过真机 Enter 确认",
            ),
            DeclareLaunchArgument(
                "pre_move_delay_sec",
                default_value="3.0",
                description="Enter 之后、首次运动之前的倒计时秒数",
            ),
            Node(
                package="fr_control",
                executable="stage4_real_grasp_test",
                name="fr_stage4_real_grasp_test",
                output="screen",
                emulate_tty=True,
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
                        "skip_confirm": ParameterValue(
                            LaunchConfiguration("skip_confirm"),
                            value_type=bool,
                        ),
                        "pre_move_delay_sec": ParameterValue(
                            LaunchConfiguration("pre_move_delay_sec"),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
