"""Trusted-feedback cache and TF pose formatting for the teleoperation UI."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import asin, atan2
from time import monotonic
from typing import Mapping


def quaternion_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """Return intrinsic XYZ roll/pitch/yaw, in radians, from a normalized quaternion."""
    roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


@dataclass
class ArmFeedback:
    joints: dict[str, float] = field(default_factory=dict)
    joint_stamp: float = 0.0
    tf_pose: dict | None = None
    tf_stamp: float = 0.0

    def joint_health(self, now: float, timeout_s: float) -> str:
        if not self.joints:
            return "UNKNOWN"
        return "ONLINE" if now - self.joint_stamp <= timeout_s else "STALE"

    def tf_health(self, now: float, timeout_s: float) -> str:
        if self.tf_pose is None:
            return "UNKNOWN"
        return "ONLINE" if now - self.tf_stamp <= timeout_s else "STALE"


class FeedbackCache:
    """Caches measured JointState data; it never stores command targets as feedback."""

    def __init__(self, robots: Mapping[str, Mapping[str, str]]) -> None:
        self._robots = robots
        self._arms = {arm: ArmFeedback() for arm in robots}

    def update_joints(self, names: list[str], positions: list[float], stamp: float | None = None) -> None:
        by_name = dict(zip(names, positions))
        stamp = monotonic() if stamp is None else stamp
        for arm, spec in self._robots.items():
            prefix = spec["joint_prefix"]
            joints = {name: value for name, value in by_name.items() if name.startswith(prefix) and name[len(prefix):] in {"j1", "j2", "j3", "j4", "j5", "j6"}}
            if len(joints) == 6:
                self._arms[arm].joints = joints
                self._arms[arm].joint_stamp = stamp

    def update_tf(self, arm: str, transform, reference_frame: str, stamp: float | None = None) -> None:
        """Store a TF TransformStamped for reference_frame <- tcp_frame."""
        value = transform.transform
        rpy = quaternion_to_rpy(value.rotation.x, value.rotation.y, value.rotation.z, value.rotation.w)
        self._arms[arm].tf_pose = {
            "frame": reference_frame,
            "position_m": {"x": value.translation.x, "y": value.translation.y, "z": value.translation.z},
            "orientation_rpy_rad": {"rx": rpy[0], "ry": rpy[1], "rz": rpy[2]},
            "orientation_xyzw": {"x": value.rotation.x, "y": value.rotation.y, "z": value.rotation.z, "w": value.rotation.w},
        }
        self._arms[arm].tf_stamp = monotonic() if stamp is None else stamp

    def status(self, arm: str, timeout_s: float) -> dict:
        now = monotonic()
        feedback = self._arms[arm]
        return {
            "joint_feedback": feedback.joint_health(now, timeout_s),
            "tcp_feedback": feedback.tf_health(now, timeout_s),
            "joint_positions_rad": feedback.joints,
            "tcp_pose": feedback.tf_pose,
            # The current overlay hardware plugin publishes no non-RT fault topic.
            "fault_state": "UNKNOWN",
            "fault_note": "fairino_hardware_dual does not publish RobotNonrtState",
        }
