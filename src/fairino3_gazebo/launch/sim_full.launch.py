"""
One-launch Gazebo + MoveIt + RViz bringup.

This file only includes existing launches. It does not start a second
controller stack or a second move_group.

Startup order:

  sim.launch.py
      Gazebo, robot_description, spawn, ros2_control
  wait moveit_delay seconds
      move_group.launch.py
      moveit_rviz.launch.py  (if use_rviz:=true)

The delay is required because sim.launch.py waits 3 s before spawning,
then starts controllers after spawn exits. move_group must not come up
before /clock, /joint_states and fairino3_controller exist.

This is the Stage 1–3 world (inspection_world.sdf). Stage 4 must use
fr_control/stage4_full.launch.py instead: that world is generated from
stage4_config.yaml and enables the grasp weld.
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


def _delayed_moveit(context, *args, **kwargs):
    """Start move_group and optional RViz after Gazebo controllers exist."""
    delay = float(LaunchConfiguration("moveit_delay").perform(context))
    use_rviz = LaunchConfiguration("use_rviz").perform(context).lower() in (
        "true",
        "1",
        "yes",
    )
    gazebo_share = get_package_share_directory("fairino3_gazebo")
    moveit_share = get_package_share_directory("fairino3_v6_moveit2_config")
    delayed = [
        LogInfo(
            msg=f"[sim_full] starting move_group after {delay:.1f}s delay"
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, "launch", "move_group.launch.py")
            ),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
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
    """Create the combined simulation launch description."""
    gazebo_share = get_package_share_directory("fairino3_gazebo")
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_share, "launch", "sim.launch.py")
                )
            ),
            OpaqueFunction(function=_delayed_moveit),
        ]
    )
