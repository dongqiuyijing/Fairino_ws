"""Mock dual-arm MoveIt bringup. Does not load FairinoHardwareInterface."""

import os

from launch import LaunchDescription
from launch.actions import (
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

from fr_control.grasp_poses import xyzw_to_rpy
from fr_control.inspection_poses import as_vec3
from fr_control.stage4_config import (
    as_xyzw,
    load_yaml,
    robot_base_rpy,
    robot_base_xyz,
    workcell_config_path,
)


def _require_mock_backends(backends):
    """Refuse any real backend so this launch cannot open a controller IP."""
    for arm in ("arm_a", "arm_b"):
        block = backends.get(arm)
        if not isinstance(block, dict):
            raise RuntimeError(f"dual_backends.yaml 缺少 {arm}")
        backend = str(block.get("backend", "")).lower()
        if backend != "mock":
            raise RuntimeError(
                f"dual_mock_bringup 只允许 mock，当前 {arm}.backend={backend}。"
                "第一阶段禁止连接真机。"
            )


def _arm_b_xyz_rpy(backends):
    pose = backends["arm_b"]["base_pose"]
    x, y, z = as_vec3(pose["position"])
    qx, qy, qz, qw = as_xyzw(pose["orientation_xyzw"])
    roll, pitch, yaw = xyzw_to_rpy(qx, qy, qz, qw)
    return (x, y, z), (roll, pitch, yaw), (qx, qy, qz, qw)


def generate_launch_description():
    pkg_share = get_package_share_directory("fairino3_dual_moveit_config")
    dual_xacro = os.path.join(pkg_share, "urdf", "dual_fr3.urdf.xacro")
    initial_positions = os.path.join(
        pkg_share, "config", "initial_positions.yaml"
    )
    ros2_controllers_yaml = os.path.join(
        pkg_share, "config", "ros2_controllers_mock.yaml"
    )
    backends_yaml = os.path.join(pkg_share, "config", "dual_backends.yaml")
    workcell_yaml = os.path.join(pkg_share, "config", "dual_workcell.yaml")
    rviz_config = os.path.join(pkg_share, "config", "dual.rviz")

    backends = load_yaml(backends_yaml)
    _require_mock_backends(backends)

    stage4 = load_yaml(workcell_config_path())
    ax, ay, az = robot_base_xyz(stage4)
    a_roll, a_pitch, a_yaw = robot_base_rpy(stage4)
    (bx, by, bz), (b_roll, b_pitch, b_yaw), b_xyzw = _arm_b_xyz_rpy(backends)

    xacro_mappings = {
        "initial_positions_file": initial_positions,
        "arm_a_x": str(ax),
        "arm_a_y": str(ay),
        "arm_a_z": str(az),
        "arm_a_roll": str(a_roll),
        "arm_a_pitch": str(a_pitch),
        "arm_a_yaw": str(a_yaw),
        "arm_b_x": str(bx),
        "arm_b_y": str(by),
        "arm_b_z": str(bz),
        "arm_b_roll": str(b_roll),
        "arm_b_pitch": str(b_pitch),
        "arm_b_yaw": str(b_yaw),
    }

    xacro_cmd = ["xacro ", dual_xacro]
    for key, value in xacro_mappings.items():
        xacro_cmd.extend([" ", f"{key}:={value}"])

    robot_description = ParameterValue(
        Command(xacro_cmd),
        value_type=str,
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "fairino3_dual_robot",
            package_name="fairino3_dual_moveit_config",
        )
        .robot_description(
            file_path="urdf/dual_fr3.urdf.xacro",
            mappings=xacro_mappings,
        )
        .robot_description_semantic(file_path="config/dual_fr3.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": False,
            }
        ],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            ros2_controllers_yaml,
            {"use_sim_time": False},
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    arm_a_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "arm_a_controller",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    arm_b_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "arm_b_controller",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": False,
                "allow_trajectory_execution": True,
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
        ],
    )

    workcell_scene_loader = Node(
        package="fr_control",
        executable="workcell_scene_loader",
        name="workcell_scene_loader",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "config_file": workcell_yaml,
                "include_object": False,
            }
        ],
    )

    def _fail(message):
        return [
            LogInfo(msg=f"[DUAL MOCK] {message}"),
            EmitEvent(event=Shutdown(reason=message)),
        ]

    def after_joint_state(event, context):
        if event.returncode != 0:
            return _fail("JointStateBroadcaster FAILED")
        return [
            LogInfo(msg="[DUAL MOCK] Starting arm_a_controller..."),
            arm_a_spawner,
        ]

    def after_arm_a(event, context):
        if event.returncode != 0:
            return _fail("arm_a_controller FAILED")
        return [
            LogInfo(msg="[DUAL MOCK] Starting arm_b_controller..."),
            arm_b_spawner,
        ]

    def after_arm_b(event, context):
        if event.returncode != 0:
            return _fail("arm_b_controller FAILED")
        return [
            LogInfo(msg="[DUAL MOCK] Starting MoveIt, RViz, and workcell scene."),
            move_group,
            rviz,
            workcell_scene_loader,
        ]

    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "[DUAL MOCK] Arm A T_world_base from stage4_config.yaml "
                    f"xyz=({ax:.3f}, {ay:.3f}, {az:.3f}) "
                    f"rpy=({a_roll:.6f}, {a_pitch:.6f}, {a_yaw:.6f})"
                )
            ),
            LogInfo(
                msg=(
                    "[DUAL MOCK] Arm B T_world_base NOMINAL from dual_backends.yaml "
                    f"xyz=({bx:.3f}, {by:.3f}, {bz:.3f}) "
                    f"xyzw=({b_xyzw[0]:.6f}, {b_xyzw[1]:.6f}, "
                    f"{b_xyzw[2]:.6f}, {b_xyzw[3]:.6f})"
                )
            ),
            LogInfo(
                msg="[DUAL MOCK] Both backends are mock. No controller IP will be opened."
            ),
            RegisterEventHandler(
                OnProcessStart(
                    target_action=controller_manager,
                    on_start=[
                        LogInfo(msg="[DUAL MOCK] Starting JointStateBroadcaster..."),
                        joint_state_spawner,
                    ],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_spawner,
                    on_exit=after_joint_state,
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=arm_a_spawner,
                    on_exit=after_arm_a,
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=arm_b_spawner,
                    on_exit=after_arm_b,
                )
            ),
            robot_state_publisher,
            controller_manager,
        ]
    )
