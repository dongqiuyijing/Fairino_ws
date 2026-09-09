"""Launch the stationary-finger collision drop test. Gazebo must already be running."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Create the launch description for collision_drop_test."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="与 Gazebo 联用时必须为 true",
            ),
            Node(
                package="fr_control",
                executable="collision_drop_test",
                name="fr_collision_drop_test",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
