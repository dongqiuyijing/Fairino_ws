"""
Compute grasp / pre-grasp / lift poses from a known object pose.

Offsets come from the HKV gripper URDF, not from trial-and-error numbers.
TCP +z is the approach axis; pre-grasp retracts along -TCP +z.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

from geometry_msgs.msg import Pose, Quaternion, Transform

from fr_control.moveit_arm import make_pose


@dataclass(frozen=True)
class GraspPoses:
    """Object and end-effector poses in the MoveIt planning frame."""

    object_pose: Pose
    grasp_pose: Pose
    pre_grasp_pose: Pose
    lift_pose: Pose
    approach_vector: tuple[float, float, float]
    planning_frame: str


def rpy_to_xyzw(roll: float, pitch: float, yaw: float) -> tuple[float, ...]:
    """Convert RPY to the xyzw quaternion used by geometry_msgs."""
    cr = math.cos(float(roll) * 0.5)
    sr = math.sin(float(roll) * 0.5)
    cp = math.cos(float(pitch) * 0.5)
    sp = math.sin(float(pitch) * 0.5)
    cy = math.cos(float(yaw) * 0.5)
    sy = math.sin(float(yaw) * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def xyzw_to_rpy(
    x: float, y: float, z: float, w: float
) -> tuple[float, float, float]:
    """Convert xyzw quaternion to ROS / URDF RPY (intrinsic ZYX)."""
    xx = float(x)
    yy = float(y)
    zz = float(z)
    ww = float(w)
    sinr_cosp = 2.0 * (ww * xx + yy * zz)
    cosr_cosp = 1.0 - 2.0 * (xx * xx + yy * yy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (ww * yy - zz * xx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (ww * zz + xx * yy)
    cosy_cosp = 1.0 - 2.0 * (yy * yy + zz * zz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quat_xyzw(orientation: Quaternion) -> tuple[float, float, float, float]:
    """Return (x, y, z, w) from a Quaternion message."""
    return (
        float(orientation.x),
        float(orientation.y),
        float(orientation.z),
        float(orientation.w),
    )


def quat_to_matrix(
    xyzw: Sequence[float],
) -> tuple[tuple[float, float, float], ...]:
    """Convert xyzw quaternion to a 3x3 rotation matrix."""
    x, y, z, w = [float(value) for value in xyzw]
    return (
        (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ),
        (
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ),
        (
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ),
    )


def rotate_vec(
    matrix: Sequence[Sequence[float]],
    vector: Sequence[float],
) -> tuple[float, float, float]:
    """Rotate a 3-vector by a 3x3 matrix."""
    return (
        matrix[0][0] * vector[0]
        + matrix[0][1] * vector[1]
        + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0]
        + matrix[1][1] * vector[1]
        + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0]
        + matrix[2][1] * vector[1]
        + matrix[2][2] * vector[2],
    )


def pose_multiply(left: Pose, right: Pose) -> Pose:
    """Return left * right as homogeneous pose composition."""
    rotation = quat_to_matrix(quat_xyzw(left.orientation))
    offset = rotate_vec(
        rotation,
        (right.position.x, right.position.y, right.position.z),
    )
    result = Pose()
    result.position.x = left.position.x + offset[0]
    result.position.y = left.position.y + offset[1]
    result.position.z = left.position.z + offset[2]
    result.orientation = _quat_multiply(left.orientation, right.orientation)
    return result


def pose_inverse(pose: Pose) -> Pose:
    """Return the inverse of a Pose."""
    rotation = quat_to_matrix(quat_xyzw(pose.orientation))
    transposed = (
        (rotation[0][0], rotation[1][0], rotation[2][0]),
        (rotation[0][1], rotation[1][1], rotation[2][1]),
        (rotation[0][2], rotation[1][2], rotation[2][2]),
    )
    offset = rotate_vec(
        transposed,
        (-pose.position.x, -pose.position.y, -pose.position.z),
    )
    inverse = Pose()
    inverse.position.x = offset[0]
    inverse.position.y = offset[1]
    inverse.position.z = offset[2]
    x, y, z, w = quat_xyzw(pose.orientation)
    inverse.orientation = Quaternion(x=-x, y=-y, z=-z, w=w)
    return inverse


def transform_to_pose(transform: Transform) -> Pose:
    """Convert a geometry_msgs/Transform to Pose."""
    pose = Pose()
    pose.position.x = float(transform.translation.x)
    pose.position.y = float(transform.translation.y)
    pose.position.z = float(transform.translation.z)
    pose.orientation = transform.rotation
    return pose


def apply_transform(transform: Transform, pose: Pose) -> Pose:
    """Apply a TF transform to a Pose: T * pose."""
    return pose_multiply(transform_to_pose(transform), pose)


def translate_pose(
    pose: Pose,
    offset: Sequence[float],
    *,
    in_local_frame: bool,
) -> Pose:
    """Translate a Pose in its local frame or in the parent frame."""
    result = Pose()
    result.orientation = pose.orientation
    if in_local_frame:
        rotation = quat_to_matrix(quat_xyzw(pose.orientation))
        world_offset = rotate_vec(rotation, offset)
    else:
        world_offset = (float(offset[0]), float(offset[1]), float(offset[2]))
    result.position.x = pose.position.x + world_offset[0]
    result.position.y = pose.position.y + world_offset[1]
    result.position.z = pose.position.z + world_offset[2]
    return result


def interpolate_pose(start: Pose, goal: Pose, fraction: float) -> Pose:
    """Linearly interpolate position; keep goal orientation."""
    alpha = float(fraction)
    pose = Pose()
    pose.position.x = (
        start.position.x + (goal.position.x - start.position.x) * alpha
    )
    pose.position.y = (
        start.position.y + (goal.position.y - start.position.y) * alpha
    )
    pose.position.z = (
        start.position.z + (goal.position.z - start.position.z) * alpha
    )
    pose.orientation = goal.orientation
    return pose


def compute_grasp_poses(
    object_pose: Pose,
    *,
    approach_rpy: Sequence[float] | None = None,
    approach_xyzw: Sequence[float] | None = None,
    pre_grasp_offset: float,
    lift_height: float,
    table_top_z: float,
    fingertip_from_tcp: float,
    table_clearance: float,
    planning_frame: str,
) -> GraspPoses:
    """
    Build grasp poses for a top-down parallel-jaw grasp.

    Grasp orientation comes from approach_xyzw, or from approach_rpy.
    Grasp height keeps the fingertips above the table, using the URDF
    TCP-to-tip distance. Pre-grasp retracts along TCP -z. Lift rises
    along world +z.
    """
    if approach_xyzw is not None:
        orientation = tuple(float(value) for value in approach_xyzw)
    elif approach_rpy is not None:
        orientation = rpy_to_xyzw(*approach_rpy)
    else:
        raise ValueError("必须提供 approach_rpy 或 approach_xyzw")
    rotation = quat_to_matrix(orientation)
    approach = rotate_vec(rotation, (0.0, 0.0, 1.0))

    min_tcp_z = (
        float(table_top_z)
        + float(fingertip_from_tcp)
        + float(table_clearance)
    )
    grasp_z = max(float(object_pose.position.z), min_tcp_z)
    grasp_pose = make_pose(
        (object_pose.position.x, object_pose.position.y, grasp_z),
        orientation,
    )
    # Retract along -approach (TCP -z). For top-down that is world +z.
    pre_grasp_pose = translate_pose(
        grasp_pose,
        (0.0, 0.0, -abs(float(pre_grasp_offset))),
        in_local_frame=True,
    )
    lift_pose = translate_pose(
        grasp_pose,
        (0.0, 0.0, abs(float(lift_height))),
        in_local_frame=False,
    )
    return GraspPoses(
        object_pose=object_pose,
        grasp_pose=grasp_pose,
        pre_grasp_pose=pre_grasp_pose,
        lift_pose=lift_pose,
        approach_vector=approach,
        planning_frame=planning_frame,
    )


def pose_in_frame(parent: Pose, child: Pose) -> Pose:
    """Return child expressed in the parent pose frame."""
    return pose_multiply(pose_inverse(parent), child)


def format_pose(pose: Pose) -> str:
    """Format a Pose for log lines."""
    pos = pose.position
    ori = pose.orientation
    return (
        f"xyz=({pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f}) "
        f"xyzw=({ori.x:.4f}, {ori.y:.4f}, {ori.z:.4f}, {ori.w:.4f})"
    )


def _quat_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    """Hamilton product left * right."""
    lx, ly, lz, lw = quat_xyzw(left)
    rx, ry, rz, rw = quat_xyzw(right)
    return Quaternion(
        x=lw * rx + lx * rw + ly * rz - lz * ry,
        y=lw * ry - lx * rz + ly * rw + lz * rx,
        z=lw * rz + lx * ry - ly * rx + lz * rw,
        w=lw * rw - lx * rx - ly * ry - lz * rz,
    )
