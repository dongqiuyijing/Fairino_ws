"""
Stage 4 object / TCP pose math.

Object inspection poses are built from two direction vectors so a face
normal uniquely maps to D1 without a leftover twist about that normal.
TCP goals are then derived from the grasped T_tcp_object transform.
"""

from __future__ import annotations

import math
from typing import Sequence

from geometry_msgs.msg import Pose, Quaternion

from fr_control.grasp_poses import (
    pose_inverse,
    pose_multiply,
    quat_to_matrix,
    quat_xyzw,
    rotate_vec,
)
from fr_control.moveit_arm import make_pose


class InspectionError(ValueError):
    """Raised when inspection directions are invalid."""


def as_vec3(value: Sequence[float] | dict) -> tuple[float, float, float]:
    """Convert a YAML list or {x,y,z} mapping to a 3-tuple."""
    if isinstance(value, dict):
        return (float(value["x"]), float(value["y"]), float(value["z"]))
    if len(value) != 3:
        raise InspectionError(f"向量长度必须为 3：{value}")
    return (float(value[0]), float(value[1]), float(value[2]))


def as_rpy(value: Sequence[float] | dict) -> tuple[float, float, float]:
    """Convert a YAML list or {roll,pitch,yaw} mapping to a 3-tuple."""
    if isinstance(value, dict):
        return (
            float(value["roll"]),
            float(value["pitch"]),
            float(value["yaw"]),
        )
    return as_vec3(value)


def vec_norm(vector: Sequence[float]) -> float:
    """Return the Euclidean norm."""
    return math.sqrt(sum(float(item) * float(item) for item in vector))


def vec_normalize(
    vector: Sequence[float],
    name: str = "vector",
) -> tuple[float, float, float]:
    """Return a unit vector or raise if the input is too small."""
    norm = vec_norm(vector)
    if norm < 1e-9:
        raise InspectionError(f"{name} 长度过小，无法归一化")
    return (vector[0] / norm, vector[1] / norm, vector[2] / norm)


def vec_dot(left: Sequence[float], right: Sequence[float]) -> float:
    """Return the dot product."""
    return (
        float(left[0]) * float(right[0])
        + float(left[1]) * float(right[1])
        + float(left[2]) * float(right[2])
    )


def vec_cross(
    left: Sequence[float],
    right: Sequence[float],
) -> tuple[float, float, float]:
    """Return the cross product."""
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def vec_sub(
    left: Sequence[float],
    right: Sequence[float],
) -> tuple[float, float, float]:
    """Return left - right."""
    return (left[0] - right[0], left[1] - right[1], left[2] - right[2])


def angle_between_deg(
    left: Sequence[float],
    right: Sequence[float],
) -> float:
    """Return the angle in degrees between two vectors."""
    left_u = vec_normalize(left, "angle left")
    right_u = vec_normalize(right, "angle right")
    cosine = max(-1.0, min(1.0, vec_dot(left_u, right_u)))
    return math.degrees(math.acos(cosine))


def check_not_collinear(
    first: Sequence[float],
    second: Sequence[float],
    first_name: str,
    second_name: str,
) -> None:
    """Raise if two directions are parallel or anti-parallel."""
    a = vec_normalize(first, first_name)
    b = vec_normalize(second, second_name)
    if abs(vec_dot(a, b)) > 0.999:
        raise InspectionError(
            f"{first_name} 与 {second_name} 共线，无法唯一确定姿态"
        )


def matrix_to_xyzw(
    matrix: Sequence[Sequence[float]],
) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to an xyzw quaternion."""
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = 0.5 / math.sqrt(trace + 1.0)
        return (
            (matrix[2][1] - matrix[1][2]) * scale,
            (matrix[0][2] - matrix[2][0]) * scale,
            (matrix[1][0] - matrix[0][1]) * scale,
            0.25 / scale,
        )
    if matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = 2.0 * math.sqrt(
            1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]
        )
        return (
            0.25 * scale,
            (matrix[0][1] + matrix[1][0]) / scale,
            (matrix[0][2] + matrix[2][0]) / scale,
            (matrix[2][1] - matrix[1][2]) / scale,
        )
    if matrix[1][1] > matrix[2][2]:
        scale = 2.0 * math.sqrt(
            1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]
        )
        return (
            (matrix[0][1] + matrix[1][0]) / scale,
            0.25 * scale,
            (matrix[1][2] + matrix[2][1]) / scale,
            (matrix[0][2] - matrix[2][0]) / scale,
        )
    scale = 2.0 * math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1])
    return (
        (matrix[0][2] + matrix[2][0]) / scale,
        (matrix[1][2] + matrix[2][1]) / scale,
        0.25 * scale,
        (matrix[1][0] - matrix[0][1]) / scale,
    )


def quat_normalize(
    xyzw: Sequence[float],
) -> tuple[float, float, float, float]:
    """Return a unit quaternion, flipping so w >= 0."""
    norm = vec_norm(xyzw)
    if norm < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    values = tuple(float(item) / norm for item in xyzw)
    if values[3] < 0.0:
        values = tuple(-item for item in values)
    return values  # type: ignore[return-value]


def frame_from_z_and_x(
    z_axis: Sequence[float],
    x_axis: Sequence[float],
) -> tuple[float, float, float, float]:
    """Build a right-handed frame with given +Z and +X; return xyzw."""
    z_unit = vec_normalize(z_axis, "z_axis")
    check_not_collinear(z_axis, x_axis, "z_axis", "x_axis")
    x_proj = vec_sub(x_axis, _scale(z_unit, vec_dot(x_axis, z_unit)))
    x_unit = vec_normalize(x_proj, "x_axis projected")
    y_unit = vec_cross(z_unit, x_unit)
    matrix = (
        (x_unit[0], y_unit[0], z_unit[0]),
        (x_unit[1], y_unit[1], z_unit[1]),
        (x_unit[2], y_unit[2], z_unit[2]),
    )
    return quat_normalize(matrix_to_xyzw(matrix))


def rotation_mapping_axes(
    from_z: Sequence[float],
    from_up: Sequence[float],
    to_z: Sequence[float],
    to_up: Sequence[float],
) -> tuple[float, float, float, float]:
    """
    Return R mapping from_z -> to_z and from_up toward to_up.

    Both (from_z, from_up) and (to_z, to_up) must be non-collinear.
    """
    check_not_collinear(from_z, from_up, "from_z", "from_up")
    check_not_collinear(to_z, to_up, "to_z", "to_up")
    source = _basis_from_z_up(from_z, from_up)
    target = _basis_from_z_up(to_z, to_up)
    # R @ source = target  =>  R = target * source^T
    source_t = _transpose(source)
    matrix = _matmul(target, source_t)
    return quat_normalize(matrix_to_xyzw(matrix))


def object_pose_for_face(
    position: Sequence[float],
    *,
    face_normal: Sequence[float],
    face_up: Sequence[float],
    inspection_direction: Sequence[float],
    inspection_up: Sequence[float],
) -> Pose:
    """Build the object pose that aims one face at D1 with a fixed up."""
    xyzw = rotation_mapping_axes(
        face_normal,
        face_up,
        inspection_direction,
        inspection_up,
    )
    return make_pose(position, xyzw)


def tcp_pose_from_object(
    object_pose: Pose,
    tcp_t_object: Pose,
) -> Pose:
    """Return T_base_tcp = T_base_object * inverse(T_tcp_object)."""
    return pose_multiply(object_pose, pose_inverse(tcp_t_object))


def object_in_tcp(tcp_pose: Pose, object_pose: Pose) -> Pose:
    """Return T_tcp_object = inverse(T_base_tcp) * T_base_object."""
    return pose_multiply(pose_inverse(tcp_pose), object_pose)


def rotate_pose_vector(
    pose: Pose,
    vector: Sequence[float],
) -> tuple[float, float, float]:
    """Rotate a vector by a Pose orientation."""
    return rotate_vec(quat_to_matrix(quat_xyzw(pose.orientation)), vector)


def pose_position_error(actual: Pose, target: Pose) -> float:
    """Return the Cartesian distance between two pose origins."""
    dx = actual.position.x - target.position.x
    dy = actual.position.y - target.position.y
    dz = actual.position.z - target.position.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def quat_angle_deg(actual: Quaternion, target: Quaternion) -> float:
    """Return the rotation angle between two quaternions, in degrees."""
    dot = abs(
        actual.x * target.x
        + actual.y * target.y
        + actual.z * target.z
        + actual.w * target.w
    )
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def quat_slerp(
    start: Sequence[float],
    goal: Sequence[float],
    fraction: float,
) -> tuple[float, float, float, float]:
    """Spherical linear interpolation of xyzw quaternions."""
    x0, y0, z0, w0 = quat_normalize(start)
    x1, y1, z1, w1 = quat_normalize(goal)
    dot = x0 * x1 + y0 * y1 + z0 * z1 + w0 * w1
    if dot < 0.0:
        x1, y1, z1, w1, dot = -x1, -y1, -z1, -w1, -dot
    alpha = float(fraction)
    if dot > 0.9995:
        return quat_normalize(
            (
                x0 + alpha * (x1 - x0),
                y0 + alpha * (y1 - y0),
                z0 + alpha * (z1 - z0),
                w0 + alpha * (w1 - w0),
            )
        )
    theta_0 = math.acos(max(-1.0, min(1.0, dot)))
    sin_0 = math.sin(theta_0)
    theta = theta_0 * alpha
    sin_t = math.sin(theta)
    s0 = math.sin(theta_0 - theta) / sin_0
    s1 = sin_t / sin_0
    return quat_normalize(
        (
            s0 * x0 + s1 * x1,
            s0 * y0 + s1 * y1,
            s0 * z0 + s1 * z1,
            s0 * w0 + s1 * w1,
        )
    )


def interpolate_object_poses(
    start: Pose,
    goal: Pose,
    count: int,
) -> list[Pose]:
    """Interpolate object poses with a fixed origin and slerp orientation."""
    if count < 1:
        raise InspectionError("插值步数至少为 1")
    start_q = quat_xyzw(start.orientation)
    goal_q = quat_xyzw(goal.orientation)
    poses = []
    for index in range(1, count + 1):
        alpha = index / float(count)
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
        xyzw = quat_slerp(start_q, goal_q, alpha)
        pose.orientation = Quaternion(
            x=xyzw[0], y=xyzw[1], z=xyzw[2], w=xyzw[3]
        )
        poses.append(pose)
    return poses


def projected_up(
    up: Sequence[float],
    direction: Sequence[float],
) -> tuple[float, float, float]:
    """Project up onto the plane perpendicular to direction."""
    direction_u = vec_normalize(direction, "direction")
    check_not_collinear(up, direction, "up", "direction")
    projected = vec_sub(up, _scale(direction_u, vec_dot(up, direction_u)))
    return vec_normalize(projected, "projected up")


def _scale(
    vector: Sequence[float],
    scalar: float,
) -> tuple[float, float, float]:
    """Return scalar * vector."""
    return (vector[0] * scalar, vector[1] * scalar, vector[2] * scalar)


def _basis_from_z_up(
    z_axis: Sequence[float],
    up: Sequence[float],
) -> tuple[tuple[float, float, float], ...]:
    """Return rotation matrix columns [x y z] from a Z axis and up vector."""
    z_unit = vec_normalize(z_axis, "z")
    x_unit = vec_normalize(vec_cross(up, z_unit), "x = up × z")
    y_unit = vec_cross(z_unit, x_unit)
    return (
        (x_unit[0], y_unit[0], z_unit[0]),
        (x_unit[1], y_unit[1], z_unit[1]),
        (x_unit[2], y_unit[2], z_unit[2]),
    )


def _transpose(
    matrix: Sequence[Sequence[float]],
) -> tuple[tuple[float, float, float], ...]:
    """Transpose a 3x3 matrix."""
    return (
        (matrix[0][0], matrix[1][0], matrix[2][0]),
        (matrix[0][1], matrix[1][1], matrix[2][1]),
        (matrix[0][2], matrix[1][2], matrix[2][2]),
    )


def _matmul(
    left: Sequence[Sequence[float]],
    right: Sequence[Sequence[float]],
) -> tuple[tuple[float, float, float], ...]:
    """Multiply two 3x3 matrices."""
    return tuple(
        tuple(
            left[row][0] * right[0][col]
            + left[row][1] * right[1][col]
            + left[row][2] * right[2][col]
            for col in range(3)
        )
        for row in range(3)
    )
