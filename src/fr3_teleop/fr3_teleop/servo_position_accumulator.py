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
    # Command approaches measured at this joint speed after a normal stop.
    # 0.25 rad/s removes a 0.09 rad tracking lead in about 0.36 s, and one
    # 125 Hz step is only 0.002 rad, so the stop does not snap backward.
    stop_correction_velocity_rad_s: float = 0.25
    stop_hold_tolerance_rad: float = 0.002
    stop_settle_timeout_s: float = 1.0
    # Maximum joint motion between 125 Hz samples treated as "already stopped".
    stop_stationary_delta_rad: float = 0.0004

    def __post_init__(self) -> None:
        if self.max_lead_rad <= 0 or self.max_raw_joint_velocity_rad_s <= 0:
            raise ValueError("velocity integrator limits must be positive")
        if self.raw_velocity_timeout_s <= 0 or self.max_timer_dt_s <= 0:
            raise ValueError("velocity integrator timeouts must be positive")
        if (
            self.stop_correction_velocity_rad_s <= 0
            or self.stop_hold_tolerance_rad <= 0
            or self.stop_settle_timeout_s <= 0
            or self.stop_stationary_delta_rad <= 0
        ):
            raise ValueError("stop settling limits must be positive")


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
        self._settling = False
        self._settle_started: float | None = None
        self._settle_prev_measured: list[float] | None = None
        self.fault_latched = False
        self._last_lead_scale = 1.0
        self._last_lead_reprojection_scale = 1.0
        self._stop_settle_max_lead_rad = 0.0
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

    @property
    def settling(self) -> bool:
        return self._settling

    @property
    def stop_settle_max_lead_rad(self) -> float:
        """Largest |command - measured| joint lead during stop settling."""
        return self._stop_settle_max_lead_rad

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
        self._settling = False
        self._settle_started = None
        self._settle_prev_measured = None
        self.fault_latched = False
        self._last_lead_scale = 1.0
        self._last_lead_reprojection_scale = 1.0
        self._stop_settle_max_lead_rad = 0.0
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
        self._settling = False
        self._settle_started = None
        self._settle_prev_measured = None
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

    def begin_stop_settling(self, measured: Iterable[float], now: float, event: str = "stop") -> tuple[float, ...]:
        """Stop using qdot without jumping the position command to feedback.

        A normal button release keeps the last command, then later ticks pull
        that command toward measured.  A latched fault is left untouched.
        """
        values = _vector(measured, "joint positions")
        if self.fault_latched:
            return tuple(self._stop_hold)
        if not self._initialized or (self.hold_active and not self._settling):
            if not self._initialized:
                return self.enter_fixed_hold(values, now, event)
            return tuple(self._stop_hold)
        if not self._settling:
            self._velocity = [0.0] * 6
            self._last_velocity_rx = None
            self.integrator_active = False
            self.hold_active = False
            self._settling = True
            self._settle_started = now
            self._settle_prev_measured = None
            self.last_event = "stop-settling"
            lead = [target - actual for target, actual in zip(self._target, values)]
            self._stop_settle_max_lead_rad = max(abs(value) for value in lead)
        return tuple(self._published)

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
        self._settling = False
        self._settle_started = None
        self._settle_prev_measured = None
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

    def _advance_stop_settling(self, measured: list[float], now: float) -> tuple[float, ...]:
        """Shrink the whole lead vector toward the current feedback.

        One scale updates all six joints.  The command therefore stays
        continuous and does not remain parked on the pre-release target.
        """
        if self._last_output_time is None:
            dt = self.config.max_timer_dt_s
        else:
            dt = now - self._last_output_time
        if dt <= 0:
            return tuple(self._published)
        self._last_output_time = now
        # A stalled timer must not spend the entire lead in one sample.
        dt = min(dt, self.config.max_timer_dt_s)
        lead = [target - actual for target, actual in zip(self._target, measured)]
        max_error = max(abs(value) for value in lead)
        max_step = self.config.stop_correction_velocity_rad_s * dt
        if max_error <= max_step:
            new_target = list(measured)
        else:
            scale = (max_error - max_step) / max_error
            new_target = [actual + value * scale for actual, value in zip(measured, lead)]
        new_lead = [command - actual for command, actual in zip(new_target, measured)]
        new_max = max(abs(value) for value in new_lead)
        self._stop_settle_max_lead_rad = new_max
        stationary = False
        if self._settle_prev_measured is not None:
            delta = max(abs(current - previous) for current, previous in zip(measured, self._settle_prev_measured))
            stationary = delta <= self.config.stop_stationary_delta_rad
        self._settle_prev_measured = list(measured)
        timed_out = self._settle_started is not None and now - self._settle_started >= self.config.stop_settle_timeout_s
        if new_max <= self.config.stop_hold_tolerance_rad and (stationary or timed_out):
            self._stop_settle_max_lead_rad = 0.0
            return self.enter_fixed_hold(measured, now, "fixed-hold")
        self._target = new_target
        self._published = list(new_target)
        self.last_event = "stop-settling"
        return tuple(self._published)

    def tick(self, measured: Iterable[float], now: float, motion_active: bool) -> tuple[float, ...]:
        measured_values = _vector(measured, "joint positions")  # Validate even while holding.
        if not self._initialized:
            return self.reset_to_measured(measured_values, now, "initialized")
        if self.fault_latched:
            return tuple(self._stop_hold)
        if not motion_active:
            if self._settling or self.integrator_active:
                if not self._settling:
                    self.begin_stop_settling(measured_values, now, "stop")
                return self._advance_stop_settling(measured_values, now)
            return self.enter_fixed_hold(measured_values, now, "fixed-hold")
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
