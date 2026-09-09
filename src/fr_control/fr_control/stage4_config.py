"""
Load and interpret stage4_config.yaml.

Task code should read values through these helpers so YAML shape stays
the single source of experiment numbers.
"""

from __future__ import annotations

from typing import Any

import yaml
from geometry_msgs.msg import Pose

from fr_control.constants import ARM_JOINTS
from fr_control.gazebo_world_pose import pose_from_rpy
from fr_control.inspection_poses import (
    InspectionError,
    as_rpy,
    as_vec3,
    check_not_collinear,
    frame_from_z_and_x,
)


def load_yaml(path: str) -> dict[str, Any]:
    """Load a YAML mapping from disk."""
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise InspectionError(f"配置格式错误：{path}")
    return data


def joint_positions(config: dict[str, Any]) -> list[float]:
    """Return Home joints in ARM_JOINTS order."""
    block = config["robot"]["initial_joint_positions"]
    missing = [name for name in ARM_JOINTS if name not in block]
    if missing:
        raise InspectionError(f"initial_joint_positions 缺少关节：{missing}")
    return [float(block[name]) for name in ARM_JOINTS]


def block_pose(block: dict[str, Any]) -> Pose:
    """Build a Pose from position / orientation_rpy nested mappings."""
    return pose_from_rpy(
        as_vec3(block["position"]),
        as_rpy(block["orientation_rpy"]),
    )


def robot_base_pose(config: dict[str, Any]) -> Pose:
    """Return T_world_base from YAML."""
    return block_pose(config["robot"]["base_pose"])


def grasp_tcp_xyzw(grasp_cfg: dict[str, Any]) -> tuple[float, float, float, float]:
    """TCP orientation: +Z = approach, +X = approach_up."""
    direction = as_vec3(grasp_cfg["approach_direction"])
    up = as_vec3(grasp_cfg["approach_up_direction"])
    check_not_collinear(
        direction, up, "grasp.approach_direction", "grasp.approach_up_direction"
    )
    return frame_from_z_and_x(direction, up)


def face_order(config: dict[str, Any]) -> list[str]:
    """Return face keys in Face1 / Face2 / Face3 order."""
    faces = config["inspection"]["faces"]
    ordered = []
    for key in ("face1", "face2", "face3"):
        if key not in faces:
            raise InspectionError(f"inspection.faces 缺少 {key}")
        ordered.append(key)
    return ordered


def validate_inspection_vectors(config: dict[str, Any]) -> None:
    """Fail fast if D1 / up or any face pair is collinear."""
    inspection = config["inspection"]
    direction = as_vec3(inspection["direction"])
    up = as_vec3(inspection["up_direction"])
    check_not_collinear(
        direction, up, "inspection.direction", "inspection.up_direction"
    )
    for name, face in inspection["faces"].items():
        check_not_collinear(
            as_vec3(face["normal_in_object"]),
            as_vec3(face["up_in_object"]),
            f"{name}.normal_in_object",
            f"{name}.up_in_object",
        )
