"""Attach two isolated Servo instances to an already running dual bringup.

They publish only to inactive, separately named teleop-controller topics. This
launch never switches controllers and therefore cannot start physical motion.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
import yaml

from fr_control.grasp_poses import xyzw_to_rpy
from fr_control.inspection_poses import as_vec3
from fr_control.stage4_config import as_xyzw, load_yaml, robot_base_rpy, robot_base_xyz, workcell_config_path


def _load(path):
    with open(path, encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def generate_launch_description():
    dual_share = get_package_share_directory("fairino3_dual_moveit_config")
    teleop_share = get_package_share_directory("fr3_teleop")
    backends = load_yaml(os.path.join(dual_share, "config", "dual_backends.yaml"))
    stage = load_yaml(workcell_config_path())
    ax, ay, az = robot_base_xyz(stage)
    ar, ap, ayaw = robot_base_rpy(stage)
    bx, by, bz = as_vec3(backends["arm_b"]["base_pose"]["position"])
    qx, qy, qz, qw = as_xyzw(backends["arm_b"]["base_pose"]["orientation_xyzw"])
    br, bp, byaw = xyzw_to_rpy(qx, qy, qz, qw)
    mappings = {
        "initial_positions_file": os.path.join(dual_share, "config", "initial_positions.yaml"),
        "arm_a_x": str(ax), "arm_a_y": str(ay), "arm_a_z": str(az),
        "arm_a_roll": str(ar), "arm_a_pitch": str(ap), "arm_a_yaw": str(ayaw),
        "arm_b_x": str(bx), "arm_b_y": str(by), "arm_b_z": str(bz),
        "arm_b_roll": str(br), "arm_b_pitch": str(bp), "arm_b_yaw": str(byaw),
        "arm_a_backend": str(backends["arm_a"]["backend"]),
        "arm_a_ip": str(backends["arm_a"]["controller_ip"]),
        "arm_a_gripper_service": str(backends["arm_a"]["gripper_service"]),
        "arm_a_gripper_node": str(backends["arm_a"]["gripper_node"]),
        "arm_a_hw_name": "ArmARealSystem" if backends["arm_a"]["backend"] == "real" else "ArmAMockSystem",
        "arm_b_backend": str(backends["arm_b"]["backend"]),
        "arm_b_ip": str(backends["arm_b"]["controller_ip"]),
        "arm_b_gripper_service": str(backends["arm_b"]["gripper_service"]),
        "arm_b_gripper_node": str(backends["arm_b"]["gripper_node"]),
        "arm_b_hw_name": "ArmBRealSystem" if backends["arm_b"]["backend"] == "real" else "ArmBMockSystem",
    }
    moveit = (MoveItConfigsBuilder("fairino3_dual_robot", package_name="fairino3_dual_moveit_config")
              .robot_description(file_path="urdf/dual_fr3.urdf.xacro", mappings=mappings)
              .robot_description_semantic(file_path="config/dual_fr3.srdf")
              .robot_description_kinematics(file_path="config/kinematics.yaml")
              .joint_limits(file_path="config/joint_limits.yaml")
              .planning_scene_monitor(
                  publish_robot_description=True,
                  publish_robot_description_semantic=True,
              )
              .planning_pipelines(pipelines=["ompl"])
              .to_moveit_configs())
    common = [moveit.robot_description, moveit.robot_description_semantic,
              moveit.robot_description_kinematics, moveit.joint_limits]
    controller_params = os.path.join(teleop_share, "config", "teleop_controllers.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("allow_real_motion", default_value="false",
                              description="Explicit nonzero Servo output enable; default false"),
        # These controllers are only loaded/configured inactive.  This launch
        # does not request a controller switch or publish a nonzero command.
        TimerAction(period=1.0, actions=[
            Node(package="controller_manager", executable="spawner", output="screen",
                 arguments=["arm_a_teleop_controller", "-c", "/controller_manager",
                            "-t", "position_controllers/JointGroupPositionController",
                            "-p", controller_params, "--inactive"]),
            Node(package="controller_manager", executable="spawner", output="screen",
                 arguments=["arm_b_teleop_controller", "-c", "/controller_manager",
                            "-t", "position_controllers/JointGroupPositionController",
                            "-p", controller_params, "--inactive"]),
        ]),
        Node(package="moveit_servo", executable="servo_node_main", name="arm_a_servo", output="screen",
             parameters=[{"moveit_servo": _load(os.path.join(teleop_share, "config", "servo_arm_a.yaml"))["moveit_servo"]}, *common]),
        Node(package="moveit_servo", executable="servo_node_main", name="arm_b_servo", output="screen",
             parameters=[{"moveit_servo": _load(os.path.join(teleop_share, "config", "servo_arm_b.yaml"))["moveit_servo"]}, *common]),
        Node(package="fr3_teleop", executable="teleop_manager", output="screen",
             parameters=[{"backend": "servo", "allow_real_motion": LaunchConfiguration("allow_real_motion")}]),
    ])
