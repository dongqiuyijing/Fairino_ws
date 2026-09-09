"""
Start Gazebo from stage4_config.yaml.

Robot spawn pose, table pose, object pose and object size all come from
the YAML. Stage 3 inspection_world.sdf is not used.
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

from fr_control.inspection_poses import as_rpy, as_vec3
from fr_control.stage4_config import load_yaml
from fr_control.stage4_world import write_world_sdf


def _launch_sim(context, *args, **kwargs):
    """Load YAML, write a world SDF, then include fairino3_gazebo sim."""
    config_file = LaunchConfiguration("config_file").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower()
    config = load_yaml(config_file)
    world_path = write_world_sdf(config)
    base = config["robot"]["base_pose"]
    position = as_vec3(base["position"])
    rpy = as_rpy(base["orientation_rpy"])
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
                "spawn_x": f"{position[0]:.6g}",
                "spawn_y": f"{position[1]:.6g}",
                "spawn_z": f"{position[2]:.6g}",
                "spawn_roll": f"{rpy[0]:.8g}",
                "spawn_pitch": f"{rpy[1]:.8g}",
                "spawn_yaw": f"{rpy[2]:.8g}",
                "enable_grasp_weld": "true",
            }.items(),
        )
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
