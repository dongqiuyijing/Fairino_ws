"""
Read and write Gazebo / Ignition model poses.

Validation must use the physics-world pose of the part, not MoveIt
AttachedCollisionObject. This module talks to the running Ignition
instance used by fairino3_gazebo (partition fairino3_gazebo).
"""

from __future__ import annotations

import os
import re
import subprocess
from typing import Sequence

from geometry_msgs.msg import Pose, Quaternion

from fr_control.grasp_poses import rpy_to_xyzw
from fr_control.moveit_arm import make_pose


class GazeboPoseError(RuntimeError):
    """Raised when an Ignition pose query or set fails."""


def ensure_ign_partition() -> None:
    """Match the partition used by fairino3_gazebo/launch/sim.launch.py."""
    os.environ.setdefault("IGN_PARTITION", "fairino3_gazebo")
    os.environ.setdefault("GZ_PARTITION", "fairino3_gazebo")


def read_model_pose(
    name: str,
    *,
    timeout_sec: float = 8.0,
) -> Pose:
    """Return the named model pose from /world/default/pose/info."""
    ensure_ign_partition()
    command = [
        "ign",
        "topic",
        "-t",
        "/world/default/pose/info",
        "-e",
        "-n",
        "1",
    ]
    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=timeout_sec,
            check=False,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
        raise GazeboPoseError(f"读取 Gazebo 位姿失败：{exc}") from exc
    if result.returncode != 0:
        raise GazeboPoseError(
            result.stderr.strip() or result.stdout.strip() or "ign topic 失败"
        )
    pose = parse_named_pose(result.stdout, name)
    if pose is None:
        raise GazeboPoseError(
            f"在 /world/default/pose/info 中未找到模型 {name}"
        )
    return pose


def set_model_pose(name: str, pose: Pose, *, timeout_sec: float = 5.0) -> None:
    """Set a model pose through /world/default/set_pose."""
    ensure_ign_partition()
    request = (
        f'name: "{name}", '
        f"position: {{x: {pose.position.x}, y: {pose.position.y}, "
        f"z: {pose.position.z}}}, "
        f"orientation: {{x: {pose.orientation.x}, y: {pose.orientation.y}, "
        f"z: {pose.orientation.z}, w: {pose.orientation.w}}}"
    )
    command = [
        "ign",
        "service",
        "-s",
        "/world/default/set_pose",
        "--reqtype",
        "ignition.msgs.Pose",
        "--reptype",
        "ignition.msgs.Boolean",
        "--timeout",
        str(int(timeout_sec * 1000)),
        "--req",
        request,
    ]
    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=timeout_sec + 2.0,
            check=False,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
        raise GazeboPoseError(f"设置 Gazebo 位姿失败：{exc}") from exc
    if result.returncode != 0:
        raise GazeboPoseError(
            result.stderr.strip() or result.stdout.strip() or "set_pose 失败"
        )


def pose_from_rpy(
    position: Sequence[float],
    rpy: Sequence[float],
) -> Pose:
    """Build a Pose from XYZ metres and RPY radians."""
    return make_pose(position, rpy_to_xyzw(*rpy))


def parse_named_pose(text: str, name: str) -> Pose | None:
    """Extract a Pose from ign topic text output."""
    window = _window_for_name(text, name)
    if window is None:
        return None
    position = _xyz_block(window, "position")
    orientation = _xyzw_block(window, "orientation")
    if position is None or orientation is None:
        return None
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    pose.orientation = Quaternion(
        x=orientation[0],
        y=orientation[1],
        z=orientation[2],
        w=orientation[3],
    )
    return pose


def _window_for_name(text: str, name: str) -> str | None:
    """Return a text window around the first matching model name."""
    lines = text.splitlines()
    needle_dq = f'name: "{name}"'
    needle_sq = f"name: '{name}'"
    for index, line in enumerate(lines):
        if needle_dq in line or needle_sq in line:
            start = index
            return "\n".join(lines[start:index + 40])
    return None


def _xyz_block(
    text: str,
    key: str,
) -> tuple[float, float, float] | None:
    """Parse x/y/z from a named protobuf-text block."""
    match = re.search(rf"{key}\s*\{{([^}}]+)\}}", text)
    if match is None:
        return None
    values = _axis_values(match.group(1), ("x", "y", "z"))
    if values is None:
        return None
    return values[0], values[1], values[2]


def _xyzw_block(
    text: str,
    key: str,
) -> tuple[float, float, float, float] | None:
    """Parse x/y/z/w from a named protobuf-text block."""
    match = re.search(rf"{key}\s*\{{([^}}]+)\}}", text)
    if match is None:
        return None
    values = _axis_values(match.group(1), ("x", "y", "z", "w"))
    if values is None:
        return None
    return values[0], values[1], values[2], values[3]


def _axis_values(
    block: str,
    axes: Sequence[str],
) -> list[float] | None:
    """Read named floats from a protobuf-text fragment."""
    values: list[float] = []
    for axis in axes:
        pattern = rf"{axis}:\s*([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)"
        match = re.search(pattern, block)
        if match is None:
            return None
        values.append(float(match.group(1)))
    return values
