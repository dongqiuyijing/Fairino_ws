"""Bounded 125 Hz position integration of MoveIt Servo joint velocities.

Despite the historical filename, this module contains no position-difference
accumulator.  MoveIt Servo publishes safe joint velocities; this class is the
only implementation that converts them to JGPC position commands.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Iterable


def _vector(values: Iterable[float], label: str) -> list[float]:
    result = [float(value) for value in values]
    if len(result) != 6:
        raise ValueError(f"expected exactly six {label}")
    if not all(isfinite(value) for value in result):
        raise ValueError(f"{label} must be finite")
    return result


@dataclass(frozen=True)
class VelocityIntegratorConfig:
    max_lead_rad: float
    max_raw_joint_velocity_rad_s: float
    raw_velocity_timeout_s: float
    max_timer_dt_s: float

    def __post_init__(self) -> None:
        if self.max_lead_rad <= 0 or self.max_raw_joint_velocity_rad_s <= 0:
            raise ValueError("velocity integrator limits must be positive")
        if self.raw_velocity_timeout_s <= 0 or self.max_timer_dt_s <= 0:
            raise ValueError("velocity integrator timeouts must be positive")


class ServoVelocityIntegrator:
    """Independent per-arm qdot-to-position integrator with a fixed stop hold."""

    def __init__(self, config: VelocityIntegratorConfig) -> None:
        self.config = config
        self._initialized = False
        self._target = [0.0] * 6
        self._published = [0.0] * 6
        self._stop_hold = [0.0] * 6
        self._velocity = [0.0] * 6
        self._last_velocity_rx: float | None = None
        self._last_output_time: float | None = None
        self.integrator_active = False
        self.hold_active = True
        self.fault_latched = False
        self._last_lead_scale = 1.0
        self._last_lead_reprojection_scale = 1.0
        self.last_event = "uninitialized"

    @property
    def target(self) -> tuple[float, ...]:
        return tuple(self._target)

    @property
    def published(self) -> tuple[float, ...]:
        return tuple(self._published)

    @property
    def stop_hold(self) -> tuple[float, ...]:
        return tuple(self._stop_hold)

    @property
    def velocity(self) -> tuple[float, ...]:
        return tuple(self._velocity)

    @property
    def last_lead_scale(self) -> float:
        """Common alpha applied to the most recent six-axis increment."""
        return self._last_lead_scale

    @property
    def last_lead_reprojection_scale(self) -> float:
        """Common scale used when the prior lead vector exceeded max_lead_rad.

        ``1.0`` means the latest integration step did not reproject.
        """
        return self._last_lead_reprojection_scale

    def reset_to_measured(self, measured: Iterable[float], now: float, event: str = "initial-hold") -> tuple[float, ...]:
        values = _vector(measured, "joint positions")
        self._initialized = True
        self._target = list(values)
        self._published = list(values)
        self._stop_hold = list(values)
        self._velocity = [0.0] * 6
        self._last_velocity_rx = None
        self._last_output_time = now
        self.integrator_active = False
        self.hold_active = True
        self.fault_latched = False
        self._last_lead_scale = 1.0
        self._last_lead_reprojection_scale = 1.0
        self.last_event = event
        return tuple(values)

    def begin_motion(self, now: float) -> bool:
        """Start integration only when no safety fault is latched.

        Repeated GUI motion messages are deliberately idempotent: they must
        not reset the output clock while motion is already active.
        """
        if not self._initialized:
            raise RuntimeError("velocity integrator is not initialized")
        if self.fault_latched:
            return False
        if not self.integrator_active:
            self.integrator_active = True
            self.hold_active = False
            self._velocity = [0.0] * 6
            # Give MoveIt Servo its raw-velocity watchdog interval before
            # declaring a timeout, rather than integrating an old velocity.
            self._last_velocity_rx = now
            self._last_output_time = now
            self.last_event = "motion-started"
        return True

    def start_new_motion(self, now: float) -> bool:
        """Explicit manager-authorized new motion start.

        ``tick()`` never clears a fault latch.  Only this manager-facing API
        (or a new AUTO -> MANUAL reset) may re-arm a recoverable held
        integrator after the operator has issued a new motion command.
        """
        self.fault_latched = False
        self._velocity = [0.0] * 6
        self._last_lead_scale = 0.0
        self._last_lead_reprojection_scale = 1.0
        return self.begin_motion(now)

    def accept_velocity(self, velocity: Iterable[float], now: float) -> str | None:
        try:
            values = _vector(velocity, "joint velocities")
        except ValueError as error:
            return str(error)
        maximum = max(abs(value) for value in values)
        if maximum > self.config.max_raw_joint_velocity_rad_s:
            return (
                "MoveIt Servo raw joint velocity exceeds max_raw_joint_velocity_rad_s "
                f"({maximum:.6g} rad/s)"
            )
        if self.integrator_active:
            self._velocity = values
            self._last_velocity_rx = now
            self.last_event = "running"
        return None

    def enter_fixed_hold(self, measured: Iterable[float], now: float, event: str) -> tuple[float, ...]:
        """Capture feedback once; later feedback noise must not move the hold target."""
        if not self._initialized or not self.hold_active:
            values = _vector(measured, "joint positions")
            self._initialized = True
            self._target = list(values)
            self._published = list(values)
            self._stop_hold = list(values)
        self._velocity = [0.0] * 6
        self._last_velocity_rx = None
        self._last_output_time = now
        self.integrator_active = False
        self.hold_active = True
        self.last_event = event
        return tuple(self._stop_hold)

    def enter_fault_hold(self, measured: Iterable[float], now: float, event: str) -> tuple[float, ...]:
        """Enter a latched safety hold that cannot be cleared by ``tick()``."""
        hold = self.enter_fixed_hold(measured, now, event)
        self.fault_latched = True
        return hold

    def _reproject_lead(self, measured: list[float]) -> float:
        """Scale the whole lead vector back inside max_lead_rad.

        Tracking lag can leave the previous target slightly ahead of measured
        joints.  One scale is applied to all six axes so the lead direction
        stays the same.  Per-joint clamps are intentionally not used.
        """
        limit = self.config.max_lead_rad
        lead = [target - actual for target, actual in zip(self._target, measured)]
        max_abs_lead = max(abs(value) for value in lead)
        if max_abs_lead <= limit:
            return 1.0
        scale = limit / max_abs_lead
        self._target = [actual + value * scale for actual, value in zip(measured, lead)]
        return scale

    def _lead_scale(self, measured: list[float], increments: list[float]) -> float:
        """Return a common alpha that keeps every target within the lead bound.

        A single joint nearing the bound slows the entire joint vector rather
        than independently clipping that joint and changing Servo's intended
        six-axis velocity direction.  Lead already outside the bound is
        reprojected by ``_reproject_lead`` before this runs.
        """
        limit = self.config.max_lead_rad
        alpha = 1.0
        for target, actual, increment in zip(self._target, measured, increments):
            lead = target - actual
            if increment > 0.0 and lead + increment > limit:
                alpha = min(alpha, (limit - lead) / increment)
            elif increment < 0.0 and lead + increment < -limit:
                alpha = min(alpha, (-limit - lead) / increment)
        return max(0.0, min(1.0, alpha))

    def tick(self, measured: Iterable[float], now: float, motion_active: bool) -> tuple[float, ...]:
        _vector(measured, "joint positions")  # Validate even while holding.
        if not self._initialized:
            return self.reset_to_measured(measured, now, "initialized")
        if self.fault_latched:
            return tuple(self._stop_hold)
        if not motion_active:
            return self.enter_fixed_hold(measured, now, "fixed-hold")
        if not self.begin_motion(now):
            return tuple(self._stop_hold)
        assert self._last_velocity_rx is not None
        if now - self._last_velocity_rx > self.config.raw_velocity_timeout_s:
            return self.enter_fault_hold(measured, now, "raw-velocity-timeout")
        assert self._last_output_time is not None
        dt = now - self._last_output_time
        self._last_output_time = now
        if dt <= 0:
            return tuple(self._published)
        if dt > self.config.max_timer_dt_s:
            return self.enter_fault_hold(measured, now, "timer-dt-out-of-range")
        measured_values = _vector(measured, "joint positions")
        increments = [qdot * dt for qdot in self._velocity]
        self._last_lead_reprojection_scale = self._reproject_lead(measured_values)
        alpha = self._lead_scale(measured_values, increments)
        self._last_lead_scale = alpha
        self._target = [target + alpha * increment for target, increment in zip(self._target, increments)]
        self._published = list(self._target)
        self.last_event = "running"
        return tuple(self._published)
