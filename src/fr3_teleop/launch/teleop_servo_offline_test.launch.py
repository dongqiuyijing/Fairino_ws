"""Fully isolated dual-arm Servo test: GenericSystem only, never FRRobot."""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from moveit_configs_utils import MoveItConfigsBuilder

from fr_control.stage4_config import robot_base_rpy, robot_base_xyz, load_yaml, workcell_config_path


def _yaml(path):
    with open(path, encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def generate_launch_description():
    namespace = "fr3_teleop_offline"
    dual = get_package_share_directory("fairino3_dual_moveit_config")
    teleop = get_package_share_directory("fr3_teleop")
    stage = load_yaml(workcell_config_path())
    ax, ay, az = robot_base_xyz(stage)
    ar, ap, ayaw = robot_base_rpy(stage)
    # Never read dual_backends.yaml here. Both xacro hardware systems are mock.
    mappings = {
        "initial_positions_file": os.path.join(teleop, "config", "offline_initial_positions.yaml"),
        "arm_a_x": str(ax), "arm_a_y": str(ay), "arm_a_z": str(az),
        "arm_a_roll": str(ar), "arm_a_pitch": str(ap), "arm_a_yaw": str(ayaw),
        "arm_a_backend": "mock", "arm_a_ip": "0.0.0.0",
        "arm_a_gripper_service": "/offline/arm_a/gripper", "arm_a_gripper_node": "offline_a_gripper",
        "arm_a_hw_name": "OfflineArmAMockSystem",
        "arm_b_backend": "mock", "arm_b_ip": "0.0.0.0",
        "arm_b_gripper_service": "/offline/arm_b/gripper", "arm_b_gripper_node": "offline_b_gripper",
        "arm_b_hw_name": "OfflineArmBMockSystem",
    }
    xacro = os.path.join(dual, "urdf", "dual_fr3.urdf.xacro")
    command = ["xacro ", xacro]
    for key, value in mappings.items():
        command.extend([" ", f"{key}:={value}"])
    robot_description = ParameterValue(Command(command), value_type=str)
    moveit = (MoveItConfigsBuilder("fairino3_dual_robot", package_name="fairino3_dual_moveit_config")
              .robot_description(file_path="urdf/dual_fr3.urdf.xacro", mappings=mappings)
              .robot_description_semantic(file_path="config/dual_fr3.srdf")
              .robot_description_kinematics(file_path="config/kinematics.yaml")
              .joint_limits(file_path="config/joint_limits.yaml")
              .planning_pipelines(pipelines=["ompl"])
              .planning_scene_monitor(
                  publish_robot_description=True,
                  publish_robot_description_semantic=True,
              )
              .to_moveit_configs())
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher", namespace=namespace, output="screen",
               parameters=[{"robot_description": robot_description, "use_sim_time": False}])
    cm = Node(package="controller_manager", executable="ros2_control_node", namespace=namespace, output="screen",
              parameters=[os.path.join(teleop, "config", "ros2_controllers_offline.yaml")],
              remappings=[("~/robot_description", f"/{namespace}/robot_description")])
    spawn_js = Node(package="controller_manager", executable="spawner", output="screen",
                    arguments=["joint_state_broadcaster", "-c", f"/{namespace}/controller_manager", "-t", "joint_state_broadcaster/JointStateBroadcaster"])
    spawn_a = Node(package="controller_manager", executable="spawner", output="screen",
                   arguments=["arm_a_teleop_controller", "-c", f"/{namespace}/controller_manager", "-t", "joint_trajectory_controller/JointTrajectoryController", "-p", os.path.join(teleop, "config", "teleop_controllers.yaml"), "--inactive"])
    spawn_b = Node(package="controller_manager", executable="spawner", output="screen",
                   arguments=["arm_b_teleop_controller", "-c", f"/{namespace}/controller_manager", "-t", "joint_trajectory_controller/JointTrajectoryController", "-p", os.path.join(teleop, "config", "teleop_controllers.yaml"), "--inactive"])
    common = [moveit.robot_description, moveit.robot_description_semantic,
              moveit.robot_description_kinematics, moveit.joint_limits]
    servo_a_cfg = _yaml(os.path.join(teleop, "config", "servo_arm_a.yaml"))["moveit_servo"]
    servo_b_cfg = _yaml(os.path.join(teleop, "config", "servo_arm_b.yaml"))["moveit_servo"]
    # There is no move_group in this isolated launch.  Arm A therefore owns
    # the namespaced planning scene; Arm B subscribes to that same scene.
    # The real launch continues to use move_group as the primary monitor.
    servo_a_cfg["is_primary_planning_scene_monitor"] = True
    servo_b_cfg["is_primary_planning_scene_monitor"] = False
    for arm, cfg in (("arm_a", servo_a_cfg), ("arm_b", servo_b_cfg)):
        cfg.update({"joint_topic": f"/{namespace}/joint_states", "cartesian_command_in_topic": f"/{namespace}/{arm}/delta_twist_cmds", "status_topic": f"/{namespace}/{arm}/servo_status", "command_out_topic": f"/{namespace}/{arm}_teleop_controller/joint_trajectory"})
    servo_a = Node(package="moveit_servo", executable="servo_node_main", namespace=namespace, name="offline_arm_a_servo", output="screen",
                   parameters=[{"moveit_servo": servo_a_cfg}, *common])
    servo_b = Node(package="moveit_servo", executable="servo_node_main", namespace=namespace, name="offline_arm_b_servo", output="screen",
                   parameters=[{"moveit_servo": servo_b_cfg}, *common])
    return LaunchDescription([rsp, cm, TimerAction(period=1.0, actions=[spawn_js]),
                              TimerAction(period=2.0, actions=[spawn_a, spawn_b]),
                              TimerAction(period=4.0, actions=[servo_a, servo_b])])
