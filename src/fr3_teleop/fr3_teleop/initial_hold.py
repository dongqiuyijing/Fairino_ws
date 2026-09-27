"""Measured-position hold gate used before allowing any nonzero Servo twist."""

from __future__ import annotations

from math import isfinite

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class InitialHoldGate:
    """Require JTC desired positions to converge to measured positions before motion."""

    def __init__(self, joint_names: list[str], max_error_rad: float, hold_s: float) -> None:
        self.joint_names, self.max_error_rad, self.hold_s = joint_names, max_error_rad, hold_s
        self.measured: list[float] | None = None
        self.verified = False

    def build_hold(self, measured_by_name: dict[str, float]) -> JointTrajectory:
        values = [float(measured_by_name[name]) for name in self.joint_names]
        if not all(isfinite(value) for value in values): raise ValueError("non-finite measured joint state")
        self.measured, self.verified = values, False
        point = JointTrajectoryPoint(positions=values, velocities=[0.0] * 6, time_from_start=Duration(sec=int(self.hold_s), nanosec=int((self.hold_s % 1.0) * 1e9)))
        return JointTrajectory(joint_names=self.joint_names, points=[point])

    def verify_desired(self, names: list[str], desired_positions: list[float]) -> bool:
        if self.measured is None or names != self.joint_names or len(desired_positions) != 6: return False
        self.verified = all(isfinite(value) and abs(value - measured) <= self.max_error_rad for value, measured in zip(desired_positions, self.measured))
        return self.verified
