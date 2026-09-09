import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)

from launch.event_handlers import OnProcessExit

from launch.conditions import IfCondition

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)

from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration

from launch_ros.actions import Node

from launch_ros.parameter_descriptions import (
    ParameterValue,
)


def generate_launch_description():

    # --------------------------------------------------
    # Package paths
    # --------------------------------------------------

    package_dir = get_package_share_directory(
        "fairino3_gazebo"
    )

    fairino_description_dir = get_package_share_directory(
        "fairino_description"
    )

    # Gazebo 的 SDF 中使用了 filename="gz_ros2_control-system"。这个名称
    # 必须优先从当前工作空间加载，不能被 /opt/ros 中旧版本的同名插件覆盖。
    gz_ros2_control_plugin_dir = os.path.join(
        get_package_prefix("gz_ros2_control"), "lib"
    )
    gz_ros2_control_plugin = os.path.join(
        gz_ros2_control_plugin_dir, "libgz_ros2_control-system.so"
    )

    # model://fairino_description/... 会在该目录下寻找 fairino_description/
    gazebo_resource_parent_dir = os.path.dirname(
        fairino_description_dir
    )

    ros_gz_sim_dir = get_package_share_directory(
        "ros_gz_sim"
    )


    robot_xacro = os.path.join(
        package_dir,
        "urdf",
        "fairino3_gazebo.urdf.xacro",
    )

    world_file = os.path.join(
        package_dir,
        "worlds",
        "inspection_world.sdf",
    )


    # --------------------------------------------------
    # xacro -> robot_description
    # --------------------------------------------------

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
            ]
        ),

        value_type=str,
    )


    # --------------------------------------------------
    # Robot State Publisher
    # --------------------------------------------------

    robot_state_publisher = Node(

        package="robot_state_publisher",

        executable="robot_state_publisher",

        name="robot_state_publisher",

        output="screen",

        parameters=[
            {
                "robot_description":
                    robot_description,
                "use_sim_time": True,
            }
        ],
    )


    # --------------------------------------------------
    # Gazebo Sim
    # --------------------------------------------------

    # 兼容旧版 Ignition Gazebo
    ign_resource_path = AppendEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=gazebo_resource_parent_dir,
    )

    # 兼容新版 Gazebo Sim（gz-sim）
    gz_resource_path = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=gazebo_resource_parent_dir,
    )

    # Fortress 使用 IGN_GAZEBO_SYSTEM_PLUGIN_PATH；新版使用
    # GZ_SIM_SYSTEM_PLUGIN_PATH。把本工作空间的目录放在最前面，确保
    # Gazebo 实际加载刚刚构建的 gz_ros2_control 插件。
    ign_system_plugin_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
        value=[
            gz_ros2_control_plugin_dir,
            os.pathsep,
            EnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", default_value=""),
        ],
    )

    gz_system_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=[
            gz_ros2_control_plugin_dir,
            os.pathsep,
            EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
        ],
    )

    # 与其它仍在运行的 Gazebo 实例隔离，避免共享 default world。
    ign_partition = SetEnvironmentVariable(
        name="IGN_PARTITION",
        value="fairino3_gazebo",
    )

    gz_partition = SetEnvironmentVariable(
        name="GZ_PARTITION",
        value="fairino3_gazebo",
    )

    gazebo = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            os.path.join(
                ros_gz_sim_dir,
                "launch",
                "gz_sim.launch.py",
            )
        ),

        launch_arguments={
            "gz_args": LaunchConfiguration("gz_args"),
        }.items(),
    )


    # --------------------------------------------------
    # Spawn FR3 into Gazebo
    # --------------------------------------------------

    spawn_robot = Node(

        package="ros_gz_sim",

        executable="create",

        output="screen",

        arguments=[
            "-world",
            "default",

            "-name",
            "fairino3",

            "-topic",
            "robot_description",

            "-x",
            LaunchConfiguration("spawn_x"),

            "-y",
            LaunchConfiguration("spawn_y"),

            "-z",
            LaunchConfiguration("spawn_z"),

            "-R",
            LaunchConfiguration("spawn_roll"),

            "-P",
            LaunchConfiguration("spawn_pitch"),

            "-Y",
            LaunchConfiguration("spawn_yaw"),
        ],
    )


    # 等 Gazebo 启动后再 spawn
    delayed_spawn = TimerAction(

        period=3.0,

        actions=[
            spawn_robot
        ],
    )


    # --------------------------------------------------
    # ROS2 controllers
    # --------------------------------------------------

    joint_state_broadcaster = Node(

        package="controller_manager",

        executable="spawner",

        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],

        output="screen",
    )


    arm_controller = Node(

        package="controller_manager",

        executable="spawner",

        arguments=[
            "fairino3_controller",
            "--controller-manager",
            "/controller_manager",
        ],

        output="screen",
    )


    gripper_left_controller = Node(

        package="controller_manager",

        executable="spawner",

        arguments=[
            "gripper_left_controller",
            "--controller-manager",
            "/controller_manager",
        ],

        output="screen",
    )


    gripper_right_controller = Node(

        package="controller_manager",

        executable="spawner",

        arguments=[
            "gripper_right_controller",
            "--controller-manager",
            "/controller_manager",
        ],

        output="screen",
    )


    clock_bridge = Node(

        package="ros_gz_bridge",

        executable="parameter_bridge",

        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/fr3/grasp/attach@std_msgs/msg/Empty]ignition.msgs.Empty",
            "/fr3/grasp/detach@std_msgs/msg/Empty]ignition.msgs.Empty",
        ],

        output="screen",
    )


    grasp_reset = Node(
        package="fairino3_gazebo",
        executable="grasp_reset",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_grasp_weld")),
    )

    # Robot spawn 完成之后启动 controllers
    start_controllers = RegisterEventHandler(

        OnProcessExit(

            target_action=spawn_robot,

            on_exit=[
                joint_state_broadcaster,
                arm_controller,
                TimerAction(
                    period=1.5,
                    actions=[
                        gripper_left_controller,
                        gripper_right_controller,
                        grasp_reset,
                    ],
                ),
            ],
        )
    )


    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gz_args",
                default_value=f"-r {world_file}",
                description="传给 ign gazebo 的参数。无界面测试可用：-s -r WORLD",
            ),
            DeclareLaunchArgument(
                "spawn_x",
                default_value="0",
                description="FR3 底座在 Gazebo world 中的 x（米）",
            ),
            DeclareLaunchArgument(
                "spawn_y",
                default_value="0",
                description="FR3 底座在 Gazebo world 中的 y（米）",
            ),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="0",
                description="FR3 底座在 Gazebo world 中的 z（米）",
            ),
            DeclareLaunchArgument(
                "spawn_roll",
                default_value="0",
                description="FR3 底座 roll（弧度）",
            ),
            DeclareLaunchArgument(
                "spawn_pitch",
                default_value="0",
                description="FR3 底座 pitch（弧度）",
            ),
            DeclareLaunchArgument(
                "enable_grasp_weld",
                default_value="false",
                description="为 true 时加载 Gazebo DetachableJoint（阶段 4 翻面）",
            ),
            ign_resource_path,
            gz_resource_path,
            ign_system_plugin_path,
            gz_system_plugin_path,
            ign_partition,
            gz_partition,
            gazebo,
            clock_bridge,
            robot_state_publisher,
            delayed_spawn,
            start_controllers,
        ]
    )
