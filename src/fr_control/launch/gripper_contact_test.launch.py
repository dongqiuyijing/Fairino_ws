"""Launch the still-arm gripper contact test. Gazebo must already be running."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Create the launch description for gripper_contact_test."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="与 Gazebo 联用时必须为 true",
            ),
            DeclareLaunchArgument(
                "backend",
                default_value="gazebo",
                description="夹爪后端：gazebo 或 real",
            ),
            Node(
                package="fr_control",
                executable="gripper_contact_test",
                name="fr_gripper_contact_test",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"),
                            value_type=bool,
                        ),
                        "backend": LaunchConfiguration("backend"),
                    }
                ],
            ),
        ]
    )
