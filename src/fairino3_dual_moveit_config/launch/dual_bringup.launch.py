"""Dual-arm MoveIt bringup with per-arm mock|real backends.

Reads config/dual_backends.yaml. A mock arm never opens a controller IP.
A real arm loads fairino_hardware_dual/FairinoDualHardwareInterface.
"""

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


def _backend(block, arm):
    if not isinstance(block, dict):
        raise RuntimeError(f"dual_backends.yaml 缺少 {arm}")
    backend = str(block.get("backend", "mock")).lower()
    if backend not in ("mock", "real"):
        raise RuntimeError(f"{arm}.backend 只能是 mock 或 real，当前={backend}")
    return backend


def _hw_name(arm, backend):
    letter = "A" if arm == "arm_a" else "B"
    kind = "Real" if backend == "real" else "Mock"
    return f"Arm{letter}{kind}System"


def _arm_b_xyz_rpy(backends):
    pose = backends["arm_b"]["base_pose"]
    x, y, z = as_vec3(pose["position"])
    qx, qy, qz, qw = as_xyzw(pose["orientation_xyzw"])
    roll, pitch, yaw = xyzw_to_rpy(qx, qy, qz, qw)
    return (x, y, z), (roll, pitch, yaw), (qx, qy, qz, qw)


def _fail(message):
    return [
        LogInfo(msg=f"[DUAL] {message}"),
        EmitEvent(event=Shutdown(reason=message)),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("fairino3_dual_moveit_config")
    dual_xacro = os.path.join(pkg_share, "urdf", "dual_fr3.urdf.xacro")
    initial_positions = os.path.join(
        pkg_share, "config", "initial_positions.yaml"
    )
    backends_yaml = os.path.join(pkg_share, "config", "dual_backends.yaml")
    workcell_yaml = os.path.join(pkg_share, "config", "dual_workcell.yaml")
    rviz_config = os.path.join(pkg_share, "config", "dual.rviz")

    backends = load_yaml(backends_yaml)
    a_backend = _backend(backends.get("arm_a"), "arm_a")
    b_backend = _backend(backends.get("arm_b"), "arm_b")
    a_ip = str(backends["arm_a"].get("controller_ip", "192.168.58.2"))
    b_ip = str(backends["arm_b"].get("controller_ip", "192.168.58.5"))
    a_grip = str(
        backends["arm_a"].get("gripper_service", "/arm_a/fairino_gripper/command")
    )
    b_grip = str(
        backends["arm_b"].get("gripper_service", "/arm_b/fairino_gripper/command")
    )
    a_node = str(
        backends["arm_a"].get("gripper_node", "arm_a_fairino_gripper_bridge")
    )
    b_node = str(
        backends["arm_b"].get("gripper_node", "arm_b_fairino_gripper_bridge")
    )
    a_hw = _hw_name("arm_a", a_backend)
    b_hw = _hw_name("arm_b", b_backend)
    any_real = a_backend == "real" or b_backend == "real"

    controllers_yaml = os.path.join(
        pkg_share,
        "config",
        "ros2_controllers.yaml" if any_real else "ros2_controllers_mock.yaml",
    )

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
        "arm_a_backend": a_backend,
        "arm_a_ip": a_ip,
        "arm_a_gripper_service": a_grip,
        "arm_a_gripper_node": a_node,
        "arm_a_hw_name": a_hw,
        "arm_b_backend": b_backend,
        "arm_b_ip": b_ip,
        "arm_b_gripper_service": b_grip,
        "arm_b_gripper_node": b_node,
        "arm_b_hw_name": b_hw,
    }

    xacro_cmd = ["xacro ", dual_xacro]
    for key, value in xacro_mappings.items():
        xacro_cmd.extend([" ", f"{key}:={value}"])

    robot_description = ParameterValue(Command(xacro_cmd), value_type=str)

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
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
    )

    cm_extra = {"use_sim_time": False}
    real_names = [name for name, backend in ((a_hw, a_backend), (b_hw, b_backend))
                  if backend == "real"]
    if real_names:
        cm_extra["hardware_components_initial_state"] = {
            "unconfigured": list(real_names)
        }

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[controllers_yaml, cm_extra],
        remappings=[("~/robot_description", "/robot_description")],
    )

    def wait_hw(name, state, node_name):
        return Node(
            package="fairino3_v6_moveit2_config",
            executable="wait_hardware_component",
            name=node_name,
            output="screen",
            arguments=[
                "--component", name,
                "--state", state,
                "--timeout", "30",
                "--controller-manager", "/controller_manager",
            ],
        )

    def hw_spawner(name):
        return Node(
            package="controller_manager",
            executable="hardware_spawner",
            name=f"{name}_spawner",
            output="screen",
            arguments=[
                name, "--activate", "-c", "/controller_manager",
                "--controller-manager-timeout", "30",
            ],
        )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster", "-c", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
    )
    arm_a_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "arm_a_controller", "-c", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
    )
    arm_b_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "arm_b_controller", "-c", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False, "allow_trajectory_execution": True},
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[moveit_config.to_dict(), {"use_sim_time": False}],
    )
    workcell_scene_loader = Node(
        package="fr_control",
        executable="workcell_scene_loader",
        name="workcell_scene_loader",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "config_file": workcell_yaml,
            "include_object": False,
        }],
    )

    startup_nodes = []
    for name in real_names:
        safe = name.lower()
        startup_nodes.extend([
            wait_hw(name, "present", f"wait_{safe}_present"),
            hw_spawner(name),
            wait_hw(name, "active", f"wait_{safe}_active"),
        ])
    startup_nodes.extend([joint_state_spawner, arm_a_spawner, arm_b_spawner])

    labels = []
    for name in real_names:
        labels.extend([
            f"{name} present",
            f"{name} hardware_spawner",
            f"{name} active",
        ])
    labels.extend([
        "JointStateBroadcaster",
        "arm_a_controller",
        "arm_b_controller",
    ])

    def after_factory(index, label):
        def _cb(event, context):
            if event.returncode != 0:
                return _fail(f"{label} FAILED")
            if index + 1 < len(startup_nodes):
                return [
                    LogInfo(msg=f"[DUAL] Starting next: {labels[index + 1]}"),
                    startup_nodes[index + 1],
                ]
            return [
                LogInfo(msg="[DUAL] Starting MoveIt, RViz, and workcell scene."),
                move_group,
                rviz,
                workcell_scene_loader,
            ]
        return _cb

    handlers = [
        RegisterEventHandler(
            OnProcessStart(
                target_action=controller_manager,
                on_start=[
                    LogInfo(msg=f"[DUAL] Starting {labels[0]}..."),
                    startup_nodes[0],
                ],
            )
        )
    ]
    for i, node in enumerate(startup_nodes):
        handlers.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=node,
                    on_exit=after_factory(i, labels[i]),
                )
            )
        )

    connect_msg = []
    if a_backend == "real":
        connect_msg.append(f"Arm A REAL -> {a_ip}")
    else:
        connect_msg.append("Arm A mock (no IP)")
    if b_backend == "real":
        connect_msg.append(f"Arm B REAL -> {b_ip}")
    else:
        connect_msg.append("Arm B mock (no IP)")

    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "[DUAL] Arm A T_world_base from stage4_config.yaml "
                    f"xyz=({ax:.3f}, {ay:.3f}, {az:.3f})"
                )
            ),
            LogInfo(
                msg=(
                    "[DUAL] Arm B T_world_base NOMINAL from dual_backends.yaml "
                    f"xyz=({bx:.3f}, {by:.3f}, {bz:.3f}) "
                    f"xyzw=({b_xyzw[0]:.6f}, {b_xyzw[1]:.6f}, "
                    f"{b_xyzw[2]:.6f}, {b_xyzw[3]:.6f})"
                )
            ),
            LogInfo(msg="[DUAL] " + " | ".join(connect_msg)),
            *handlers,
            robot_state_publisher,
            controller_manager,
        ]
    )
