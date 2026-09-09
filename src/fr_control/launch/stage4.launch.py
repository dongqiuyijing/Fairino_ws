"""
Launch the FR3 stage 4 inspection test.

Gazebo must already be running with the same stage4_config.yaml
(use stage4_sim.launch.py). move_group must also be running.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Create the launch description for stage4_inspection_test."""
    config_default = os.path.join(
        get_package_share_directory("fr_control"),
        "config",
        "stage4_config.yaml",
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
                description="阶段 4 YAML 配置路径",
            ),
            DeclareLaunchArgument(
                "gripper_backend",
                default_value="gazebo",
                description="夹爪后端：gazebo 或 real",
            ),
            DeclareLaunchArgument(
                "sim_grasp_backend",
                default_value="gazebo",
                description="仿真抓取辅助：gazebo 使用 DetachableJoint",
            ),
            Node(
                package="fr_control",
                executable="stage4_inspection_test",
                name="fr_stage4_inspection_test",
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
                        "sim_grasp_backend": LaunchConfiguration(
                            "sim_grasp_backend"
                        ),
                    }
                ],
            ),
        ]
    )
