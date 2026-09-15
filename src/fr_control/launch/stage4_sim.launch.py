"""
Start Gazebo from stage4_config.yaml.

Robot spawn pose, table pose, object pose and object size all come from
the YAML. Stage 3 inspection_world.sdf is not used.

Installation pose is applied once, in the Gazebo URDF world_to_base
joint. The robot model is spawned at the world origin so Gazebo, TF and
RViz share the same world → base_link transform. Do not also publish a
static TF or spawn the model at base_pose; that double-applies the pose.

This launch does not start move_group. Use stage4_full.launch.py for
Gazebo + MoveIt + RViz. workcell_scene_loader waits for Planning Scene
services and then loads column/table from the same YAML.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from fr_control.stage4_config import load_yaml, robot_base_rpy, robot_base_xyz
from fr_control.stage4_world import write_world_sdf


def _launch_sim(context, *args, **kwargs):
    """Load YAML, write a world SDF, then include fairino3_gazebo sim."""
    config_file = LaunchConfiguration("config_file").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower()
    config = load_yaml(config_file)
    world_path = write_world_sdf(config)
    position = robot_base_xyz(config)
    rpy = robot_base_rpy(config)
    prefix = "-s -r " if headless in ("true", "1", "yes") else "-r "
    sim_launch = os.path.join(
        get_package_share_directory("fairino3_gazebo"),
        "launch",
        "sim.launch.py",
    )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                "gz_args": prefix + world_path,
                "spawn_x": "0",
                "spawn_y": "0",
                "spawn_z": "0",
                "spawn_roll": "0",
                "spawn_pitch": "0",
                "spawn_yaw": "0",
                "world_to_base_x": f"{position[0]:.8g}",
                "world_to_base_y": f"{position[1]:.8g}",
                "world_to_base_z": f"{position[2]:.8g}",
                "world_to_base_roll": f"{rpy[0]:.8g}",
                "world_to_base_pitch": f"{rpy[1]:.8g}",
                "world_to_base_yaw": f"{rpy[2]:.8g}",
                "enable_grasp_weld": "false",
            }.items(),
        ),
        Node(
            package="fr_control",
            executable="workcell_scene_loader",
            name="workcell_scene_loader",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "config_file": config_file,
                    "include_object": False,
                }
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Create the launch description for the stage 4 Gazebo world."""
    config_default = os.path.join(
        get_package_share_directory("fr_control"),
        "config",
        "stage4_config.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=config_default,
                description="阶段 4 YAML 配置路径",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="true 时使用 ign gazebo -s（无界面）",
            ),
            OpaqueFunction(function=_launch_sim),
        ]
    )
