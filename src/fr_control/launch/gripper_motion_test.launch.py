"""
Launch the unified gripper motion test.

Does not start Gazebo, MoveIt, or real_bringup. Those must already run.
Does not command FR3 arm motion and does not auto-reset the gripper.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _backend_from_mode(mode: str, backend: str) -> str:
    """Map launch mode to gripper backend. Tasks must not do this."""
    if backend:
        return backend
    if mode in ("real", "hardware", "fairino"):
        return "real"
    return "gazebo"


def _launch_setup(context, *args, **kwargs):
    """Resolve mode -> backend and start the test node."""
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    backend = _backend_from_mode(
        mode,
        LaunchConfiguration("backend").perform(context).strip().lower(),
    )
    use_sim_time = backend != "real"
    return [
        Node(
            package="fr_control",
            executable="gripper_motion_test",
            name="fr_gripper_motion_test",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "mode": mode,
                    "backend": backend,
                    "dry_run": ParameterValue(
                        LaunchConfiguration("dry_run"),
                        value_type=bool,
                    ),
                    "sequence": LaunchConfiguration("sequence"),
                    "do_activate": ParameterValue(
                        LaunchConfiguration("do_activate"),
                        value_type=bool,
                    ),
                    "do_reset": ParameterValue(
                        LaunchConfiguration("do_reset"),
                        value_type=bool,
                    ),
                    "skip_confirm": ParameterValue(
                        LaunchConfiguration("skip_confirm"),
                        value_type=bool,
                    ),
                }
            ],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """Create the launch description for gripper_motion_test."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="sim",
                description="sim 或 real；launch 用来选择 gripper backend",
            ),
            DeclareLaunchArgument(
                "backend",
                default_value="",
                description="覆盖 mode：gazebo 或 real",
            ),
            DeclareLaunchArgument(
                "dry_run",
                default_value="false",
                description="true 时只检查配置/通信，不让夹爪运动",
            ),
            DeclareLaunchArgument(
                "sequence",
                default_value="full",
                description="full=open/close/open；conservative=小行程",
            ),
            DeclareLaunchArgument(
                "do_activate",
                default_value="false",
                description="true 时显式 ActGripper(id,1)；默认不激活",
            ),
            DeclareLaunchArgument(
                "do_reset",
                default_value="false",
                description="true 时显式 ActGripper(id,0)；默认不复位",
            ),
            DeclareLaunchArgument(
                "skip_confirm",
                default_value="false",
                description="true 时跳过真机 Enter 确认",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
