"""Measured-position Float64 hold gate used before any nonzero Servo twist."""

from __future__ import annotations

from math import isfinite

from std_msgs.msg import Float64MultiArray


class InitialHoldGate:
    """Require a measured-position hold to remain close to measured feedback."""

    def __init__(self, joint_names: list[str], max_error_rad: float, hold_s: float) -> None:
        self.joint_names, self.max_error_rad, self.hold_s = joint_names, max_error_rad, hold_s
        self.measured: list[float] | None = None
        self.verified = False

    def build_hold(self, measured_by_name: dict[str, float]) -> Float64MultiArray:
        values = [float(measured_by_name[name]) for name in self.joint_names]
        if not all(isfinite(value) for value in values): raise ValueError("non-finite measured joint state")
        self.measured, self.verified = values, False
        return Float64MultiArray(data=values)

    def verify_measured(self, measured_by_name: dict[str, float]) -> bool:
        if self.measured is None: return False
        try:
            values = [float(measured_by_name[name]) for name in self.joint_names]
        except (KeyError, TypeError, ValueError):
            return False
        if len(values) != 6 or not all(isfinite(value) for value in values): return False
        self.verified = all(abs(value - held) <= self.max_error_rad for value, held in zip(values, self.measured))
        return self.verified
