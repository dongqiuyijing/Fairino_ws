"""
One-launch Gazebo grasp test matching the real first-round sequence.

Reuses fairino3_gazebo/sim.launch.py, move_group, RViz, and
workcell_scene_loader. Does not include stage4_sim.launch.py because
that file hardcodes enable_grasp_weld:=false.

enable_grasp_weld:=true is passed into sim.launch.py, which xacro-loads
the Ignition DetachableJoint plugin on the gripper.
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
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from fr_control.stage4_config import load_yaml, robot_base_rpy, robot_base_xyz
from fr_control.stage4_world import write_world_sdf


def _gazebo_xacro_args(context) -> dict[str, str]:
    """Return the xacro args shared by sim.launch.py and move_group.launch.py."""
    config = load_yaml(LaunchConfiguration("config_file").perform(context))
    position = robot_base_xyz(config)
    rpy = robot_base_rpy(config)
    return {
        "world_to_base_x": f"{position[0]:.8g}",
        "world_to_base_y": f"{position[1]:.8g}",
        "world_to_base_z": f"{position[2]:.8g}",
        "world_to_base_roll": f"{rpy[0]:.8g}",
        "world_to_base_pitch": f"{rpy[1]:.8g}",
        "world_to_base_yaw": f"{rpy[2]:.8g}",
        "enable_grasp_weld": LaunchConfiguration("enable_grasp_weld").perform(
            context
        ),
    }


def _launch_gazebo(context, *args, **kwargs):
    """Write the YAML world and start Gazebo with the grasp-weld plugin."""
    config_file = LaunchConfiguration("config_file").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower()
    xacro_args = _gazebo_xacro_args(context)
    config = load_yaml(config_file)
    world_path = write_world_sdf(config)
    prefix = "-s -r " if headless in ("true", "1", "yes") else "-r "
    sim_launch = os.path.join(
        get_package_share_directory("fairino3_gazebo"),
        "launch",
        "sim.launch.py",
    )
    return [
        LogInfo(
            msg=(
                "[stage4_sim_grasp] enable_grasp_weld="
                f"{xacro_args['enable_grasp_weld']} "
                "-> xacro DetachableJoint + /fr3/grasp bridge"
            )
        ),
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
                **xacro_args,
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


def _delayed_moveit_and_test(context, *args, **kwargs):
    """Start MoveIt/RViz, then the grasp test after Gazebo is up."""
    delay = float(LaunchConfiguration("moveit_delay").perform(context))
    test_delay = float(LaunchConfiguration("test_delay").perform(context))
    use_rviz = LaunchConfiguration("use_rviz").perform(context).lower() in (
        "true",
        "1",
        "yes",
    )
    gazebo_share = get_package_share_directory("fairino3_gazebo")
    moveit_share = get_package_share_directory("fairino3_v6_moveit2_config")
    xacro_args = _gazebo_xacro_args(context)
    delayed = [
        LogInfo(
            msg=(
                "[stage4_sim_grasp] starting move_group after "
                f"{delay:.1f}s with the same Gazebo URDF xacro args"
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, "launch", "move_group.launch.py")
            ),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                **xacro_args,
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
    test_node = Node(
        package="fr_control",
        executable="stage4_sim_grasp_test",
        name="fr_stage4_sim_grasp_test",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"),
                    value_type=bool,
                ),
                "config_file": LaunchConfiguration("config_file"),
                "gripper_backend": LaunchConfiguration("gripper_backend"),
                "sim_grasp_backend": LaunchConfiguration(
                    "sim_grasp_backend"
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
    )
    return [
        TimerAction(period=delay, actions=delayed),
        TimerAction(
            period=test_delay,
            actions=[
                LogInfo(
                    msg=(
                        "[stage4_sim_grasp] starting sim grasp test after "
                        f"{test_delay:.1f}s"
                    )
                ),
                test_node,
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Create the combined Gazebo grasp-test launch description."""
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
                "test_delay",
                default_value="14.0",
                description="启动 stage4_sim_grasp_test 前的等待秒数",
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
            DeclareLaunchArgument(
                "enable_grasp_weld",
                default_value="true",
                description="必须为 true：加载 gripper DetachableJoint",
            ),
            DeclareLaunchArgument(
                "gripper_backend",
                default_value="gazebo",
                description="仿真夹爪后端，必须为 gazebo",
            ),
            DeclareLaunchArgument(
                "sim_grasp_backend",
                default_value="gazebo",
                description="仿真抓取辅助：gazebo 使用 DetachableJoint",
            ),
            DeclareLaunchArgument(
                "skip_confirm",
                default_value="true",
                description="Gazebo 默认跳过 Enter 确认",
            ),
            DeclareLaunchArgument(
                "pre_move_delay_sec",
                default_value="0.0",
                description="首次运动前的倒计时秒数",
            ),
            OpaqueFunction(function=_launch_gazebo),
            OpaqueFunction(function=_delayed_moveit_and_test),
        ]
    )
