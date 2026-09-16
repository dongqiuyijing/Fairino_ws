"""
One-launch Stage 4 simulation: YAML world + weld + MoveIt + RViz.

Includes stage4_sim.launch.py (which itself includes fairino3_gazebo
sim.launch.py). Does not start a second controller or move_group stack.

For Stage 1–3 use fairino3_gazebo/sim_full.launch.py instead.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from fr_control.stage4_config import load_yaml, robot_base_rpy, robot_base_xyz


def _delayed_moveit(context, *args, **kwargs):
    """Start move_group and optional RViz after Gazebo controllers exist."""
    delay = float(LaunchConfiguration("moveit_delay").perform(context))
    use_rviz = LaunchConfiguration("use_rviz").perform(context).lower() in (
        "true",
        "1",
        "yes",
    )
    config = load_yaml(LaunchConfiguration("config_file").perform(context))
    position = robot_base_xyz(config)
    rpy = robot_base_rpy(config)
    gazebo_share = get_package_share_directory("fairino3_gazebo")
    moveit_share = get_package_share_directory("fairino3_v6_moveit2_config")
    delayed = [
        LogInfo(
            msg=f"[stage4_full] starting move_group after {delay:.1f}s delay"
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, "launch", "move_group.launch.py")
            ),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "enable_grasp_weld": "false",
                "world_to_base_x": f"{position[0]:.8g}",
                "world_to_base_y": f"{position[1]:.8g}",
                "world_to_base_z": f"{position[2]:.8g}",
                "world_to_base_roll": f"{rpy[0]:.8g}",
                "world_to_base_pitch": f"{rpy[1]:.8g}",
                "world_to_base_yaw": f"{rpy[2]:.8g}",
            }.items(),
        ),
    ]
    if use_rviz:
        delayed.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        moveit_share, "launch", "moveit_rviz.launch.py"
                    )
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            )
        )
    return [TimerAction(period=delay, actions=delayed)]


def generate_launch_description() -> LaunchDescription:
    """Create the combined Stage 4 simulation launch description."""
    control_share = get_package_share_directory("fr_control")
    config_default = os.path.join(
        control_share, "config", "stage4_config.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Gazebo 仿真必须为 true",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="false 时不启动 RViz，仍启动 move_group",
            ),
            DeclareLaunchArgument(
                "moveit_delay",
                default_value="8.0",
                description="等待 Gazebo spawn 和 controller 的秒数",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=config_default,
                description="阶段 4 YAML 配置路径",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="true 时 Gazebo 无界面（ign gazebo -s）",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        control_share, "launch", "stage4_sim.launch.py"
                    )
                ),
                launch_arguments={
                    "config_file": LaunchConfiguration("config_file"),
                    "headless": LaunchConfiguration("headless"),
                }.items(),
            ),
            OpaqueFunction(function=_delayed_moveit),
        ]
    )
