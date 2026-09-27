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
        self.last_event = event
        return tuple(values)

    def begin_motion(self, now: float) -> None:
        if not self._initialized:
            raise RuntimeError("velocity integrator is not initialized")
        if not self.integrator_active:
            self.integrator_active = True
            self.hold_active = False
            self._velocity = [0.0] * 6
            # Give MoveIt Servo its raw-velocity watchdog interval before
            # declaring a timeout, rather than integrating an old velocity.
            self._last_velocity_rx = now
            self._last_output_time = now
            self.last_event = "motion-started"

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
        if not self.hold_active:
            values = _vector(measured, "joint positions")
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

    def tick(self, measured: Iterable[float], now: float, motion_active: bool) -> tuple[float, ...]:
        _vector(measured, "joint positions")  # Validate even while holding.
        if not self._initialized:
            return self.reset_to_measured(measured, now, "initialized")
        if not motion_active:
            return self.enter_fixed_hold(measured, now, "fixed-hold")
        self.begin_motion(now)
        assert self._last_velocity_rx is not None
        if now - self._last_velocity_rx > self.config.raw_velocity_timeout_s:
            return self.enter_fixed_hold(measured, now, "raw-velocity-timeout")
        assert self._last_output_time is not None
        dt = now - self._last_output_time
        self._last_output_time = now
        if dt <= 0:
            return tuple(self._published)
        if dt > self.config.max_timer_dt_s:
            return self.enter_fixed_hold(measured, now, "timer-dt-out-of-range")
        measured_values = _vector(measured, "joint positions")
        self._target = [
            max(actual - self.config.max_lead_rad,
                min(actual + self.config.max_lead_rad, target + qdot * dt))
            for target, qdot, actual in zip(self._target, self._velocity, measured_values)
        ]
        self._published = list(self._target)
        self.last_event = "running"
        return tuple(self._published)
