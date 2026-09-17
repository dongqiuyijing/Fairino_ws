"""
MoveIt move_group launch for Gazebo.

Uses the same fairino3_gazebo.urdf.xacro as sim.launch.py and
robot_state_publisher. Does not load official
fairino3_v6_robot.urdf.xacro (real TCP 0.150).

Official move_group aborts trajectories that run slightly longer than the
plan. Gazebo tracking lag then shows up as CONTROL_FAILED. This file
keeps the official SRDF / kinematics / controllers and only replaces
robot_description plus execution timing.
"""

import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import (
    DeclareBooleanLaunchArg,
    add_debuggable_node,
)


def generate_launch_description() -> LaunchDescription:
    """Launch move_group with the Gazebo URDF and relaxed execution timing."""
    gazebo_share = get_package_share_directory("fairino3_gazebo")
    robot_xacro = os.path.join(
        gazebo_share, "urdf", "fairino3_gazebo.urdf.xacro"
    )
    gz_ros2_control_plugin = os.path.join(
        get_package_prefix("gz_ros2_control"),
        "lib",
        "libgz_ros2_control-system.so",
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                robot_xacro,
                " ",
                "gz_ros2_control_plugin:=",
                gz_ros2_control_plugin,
                " ",
                "enable_grasp_weld:=",
                LaunchConfiguration("enable_grasp_weld"),
                " ",
                "world_to_base_x:=",
                LaunchConfiguration("world_to_base_x"),
                " ",
                "world_to_base_y:=",
                LaunchConfiguration("world_to_base_y"),
                " ",
                "world_to_base_z:=",
                LaunchConfiguration("world_to_base_z"),
                " ",
                "world_to_base_roll:=",
                LaunchConfiguration("world_to_base_roll"),
                " ",
                "world_to_base_pitch:=",
                LaunchConfiguration("world_to_base_pitch"),
                " ",
                "world_to_base_yaw:=",
                LaunchConfiguration("world_to_base_yaw"),
                " ",
                "initial_j1:=",
                LaunchConfiguration("initial_j1"),
                " ",
                "initial_j2:=",
                LaunchConfiguration("initial_j2"),
                " ",
                "initial_j3:=",
                LaunchConfiguration("initial_j3"),
                " ",
                "initial_j4:=",
                LaunchConfiguration("initial_j4"),
                " ",
                "initial_j5:=",
                LaunchConfiguration("initial_j5"),
                " ",
                "initial_j6:=",
                LaunchConfiguration("initial_j6"),
            ]
        ),
        value_type=str,
    )

    moveit_config = MoveItConfigsBuilder(
        "fairino3_v6_robot",
        package_name="fairino3_v6_moveit2_config",
    ).to_moveit_configs()
    moveit_params = moveit_config.to_dict()
    moveit_params["robot_description"] = robot_description

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Gazebo 仿真必须为 true，以订阅 /clock。",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "enable_grasp_weld",
            default_value="false",
            description="与 sim.launch.py 使用同一 xacro 开关",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "world_to_base_x",
            default_value="0",
            description="URDF world→base_link 的 x（米）",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "world_to_base_y",
            default_value="0",
            description="URDF world→base_link 的 y（米）",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "world_to_base_z",
            default_value="0",
            description="URDF world→base_link 的 z（米）",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "world_to_base_roll",
            default_value="0",
            description="URDF world→base_link 的 roll（弧度）",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "world_to_base_pitch",
            default_value="0",
            description="URDF world→base_link 的 pitch（弧度）",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "world_to_base_yaw",
            default_value="0",
            description="URDF world→base_link 的 yaw（弧度）",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "initial_j1",
            default_value="0.8377",
            description="Gazebo j1 初始位置（弧度）",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "initial_j2",
            default_value="-1.1927",
            description="Gazebo j2 初始位置（弧度）",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "initial_j3",
            default_value="1.1129",
            description="Gazebo j3 初始位置（弧度）",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "initial_j4",
            default_value="-1.1499",
            description="Gazebo j4 初始位置（弧度）",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "initial_j5",
            default_value="-3.1393",
            description="Gazebo j5 初始位置（弧度）",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "initial_j6",
            default_value="2.0996",
            description="Gazebo j6 初始位置（弧度）",
        )
    )
    ld.add_action(
        SetParameter(
            name="use_sim_time",
            value=LaunchConfiguration("use_sim_time"),
        )
    )
    ld.add_action(
        SetParameter(
            name="trajectory_execution.execution_duration_monitoring",
            value=False,
        )
    )
    ld.add_action(
        SetParameter(
            name="trajectory_execution.allowed_execution_duration_scaling",
            value=5.0,
        )
    )
    ld.add_action(
        SetParameter(
            name="trajectory_execution.allowed_goal_duration_margin",
            value=10.0,
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "publish_monitored_planning_scene", default_value=True
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "capabilities",
            default_value=moveit_config.move_group_capabilities["capabilities"],
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "disable_capabilities",
            default_value=moveit_config.move_group_capabilities[
                "disable_capabilities"
            ],
        )
    )

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration(
            "allow_trajectory_execution"
        ),
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }

    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        commands_file=str(
            moveit_config.package_path / "launch" / "gdb_settings.gdb"
        ),
        output="screen",
        parameters=[
            moveit_params,
            move_group_configuration,
        ],
        extra_debug_args=["--debug"],
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )
    return ld
