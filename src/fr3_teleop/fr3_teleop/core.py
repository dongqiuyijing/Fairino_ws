"""Framework-independent ownership, validation, and watchdog logic."""

from __future__ import annotations

from dataclasses import dataclass
from time import monotonic
from typing import Any, Protocol


VALID_FRAMES = {"world", "base", "tool"}
VALID_AXES = {"x", "y", "z", "rx", "ry", "rz"}


@dataclass(frozen=True)
class Motion:
    arm: str
    frame: str
    axis: str
    sign: int
    linear_mm_s: float
    angular_deg_s: float
    source: str


class Backend(Protocol):
    """One backend for every input source (GUI and future joystick)."""

    name: str

    def start(self, motion: Motion) -> None: ...
    def stop(self, arm: str, reason: str) -> None: ...
    def gripper(self, arm: str, command: str) -> None: ...


class SourceRejected(ValueError):
    """Raised when a command arrives from an input that does not own the session."""


class MockBackend:
    """Non-moving backend used by tests and the default launch."""

    name = "mock"

    def __init__(self) -> None:
        self.history: list[tuple[str, Any]] = []
        self.active: dict[str, Motion] = {}

    def start(self, motion: Motion) -> None:
        self.active[motion.arm] = motion
        self.history.append(("start", motion))

    def stop(self, arm: str, reason: str) -> None:
        self.active.pop(arm, None)
        self.history.append(("stop", (arm, reason)))

    def gripper(self, arm: str, command: str) -> None:
        self.history.append(("gripper", (arm, command)))


class ControlCore:
    """Single-owner state machine. It does not speak ROS or the SDK."""

    def __init__(self, config: dict[str, Any], backend: Backend) -> None:
        self.config = config
        self.backend = backend
        robots = config["robots"]
        self.robots = {key for key, value in robots.items() if value["enabled"]}
        teleop = config["teleop"]
        self.arm = teleop["default_arm"]
        self.frame = teleop["default_frame"]
        self.linear = float(teleop["linear_speed_mm_s"])
        self.angular = float(teleop["angular_speed_deg_s"])
        self.enabled = not bool(teleop["start_with_control_disabled"])
        self.active: Motion | None = None
        self.last_input: float | None = None
        self.active_input_source: str | None = None
        self.claim_time: float | None = None
        self.fault = ""
        self.last_stop_reason = "startup"
        self._validate_config()

    def _reject_if_foreign(self, data: dict[str, Any]) -> str:
        source = str(data.get("source") or "")
        owner = self.active_input_source
        if owner and source and source != owner:
            raise SourceRejected(f"input source '{source}' is blocked while '{owner}' is active")
        return source

    def _claim(self, source: str, now: float) -> None:
        self.active_input_source = source
        self.claim_time = now

    def _validate_config(self) -> None:
        teleop = self.config["teleop"]
        if self.arm not in self.robots or self.frame not in VALID_FRAMES:
            raise ValueError("invalid default arm or frame")
        if not 1 <= int(teleop["input_rate_hz"]) <= 100:
            raise ValueError("input_rate_hz must be in [1, 100]")
        if not 50 <= int(teleop["input_timeout_ms"]) <= 2000:
            raise ValueError("input_timeout_ms must be in [50, 2000]")
        if float(teleop["max_linear_speed_mm_s"]) <= 0 or float(teleop["max_angular_speed_deg_s"]) <= 0:
            raise ValueError("speed limits must be positive")

    def _stop(self, reason: str) -> None:
        if self.active is not None:
            self.backend.stop(self.active.arm, reason)
        self.active = None
        self.last_input = None
        self.last_stop_reason = reason

    def command(self, data: dict[str, Any], now: float | None = None) -> None:
        now = monotonic() if now is None else now
        kind = data.get("type")
        if kind == "enable":
            self._reject_if_foreign(data)
            self._stop("manual-control-disabled")
            self.enabled = bool(data.get("value", False))
            return
        if kind == "stop":
            source = str(data.get("source") or "")
            reason = str(data.get("reason", "operator-stop"))
            foreign = bool(self.active_input_source and source and source != self.active_input_source)
            if foreign and reason != "operator-stop":
                return
            self._stop(reason)
            if not data.get("keep_claim") or reason == "operator-stop":
                self.active_input_source = None
                self.claim_time = None
            return
        if kind == "release":
            source = str(data.get("source") or "")
            if self.active_input_source and source and source != self.active_input_source:
                return
            self._stop(str(data.get("reason", "source-release")))
            self.active_input_source = None
            self.claim_time = None
            return
        if kind == "hold":
            source = self._reject_if_foreign(data)
            if not source:
                raise ValueError("hold requires a source")
            self._claim(source, now)
            return
        if kind == "select_arm":
            source = self._reject_if_foreign(data)
            arm = str(data.get("arm", ""))
            if arm not in self.robots:
                raise ValueError("unknown or disabled arm")
            self._stop("robot-switch")
            self.arm = arm
            if data.get("claim"):
                if not source:
                    raise ValueError("claim requires a source")
                self._claim(source, now)
            else:
                self.active_input_source = None
                self.claim_time = None
            return
        if kind == "set_frame":
            self._reject_if_foreign(data)
            frame = str(data.get("frame", ""))
            if frame not in VALID_FRAMES:
                raise ValueError("unknown frame")
            self._stop("frame-switch")
            self.frame = frame
            return
        if kind == "set_speed":
            self._reject_if_foreign(data)
            self._stop("speed-change")
            teleop = self.config["teleop"]
            linear = float(data.get("linear_mm_s", self.linear))
            angular = float(data.get("angular_deg_s", self.angular))
            self.linear = min(max(linear, 0.1), float(teleop["max_linear_speed_mm_s"]))
            self.angular = min(max(angular, 0.1), float(teleop["max_angular_speed_deg_s"]))
            return
        if kind == "gripper":
            self._reject_if_foreign(data)
            if not self.enabled:
                raise ValueError("manual control is disabled")
            command = str(data.get("command", ""))
            if command not in {"open", "close"}:
                raise ValueError("invalid gripper command")
            self._stop("gripper-command")
            self.backend.gripper(self.arm, command)
            return
        if kind != "motion":
            raise ValueError("unknown command")
        if not self.enabled:
            raise ValueError("manual control is disabled")
        source = self._reject_if_foreign(data) or "unknown"
        axis, sign = str(data.get("axis", "")), int(data.get("sign", 0))
        if axis not in VALID_AXES or sign not in {-1, 1}:
            raise ValueError("invalid motion axis or sign")
        scale = float(data.get("scale", 1.0))
        if not 0.0 < scale <= 1.0:
            raise ValueError("motion scale must be in (0, 1]")
        requested_arm = str(data.get("arm", self.arm))
        if requested_arm != self.arm:
            raise ValueError("motion targets a non-owned arm")
        teleop = self.config["teleop"]
        linear = min(self.linear * scale, float(teleop["max_linear_speed_mm_s"]))
        angular = min(self.angular * scale, float(teleop["max_angular_speed_deg_s"]))
        motion = Motion(self.arm, self.frame, axis, sign, linear, angular, source)
        self.backend.start(motion)
        self.active, self.last_input = motion, now
        self._claim(source, now)

    def tick(self, now: float | None = None) -> None:
        now = monotonic() if now is None else now
        timeout = float(self.config["teleop"]["input_timeout_ms"]) / 1000.0
        motion_expired = (
            self.active is not None and self.last_input is not None and now - self.last_input > timeout
        )
        claim_expired = (
            self.active_input_source is not None
            and self.claim_time is not None
            and now - self.claim_time > timeout
        )
        if motion_expired or claim_expired:
            self._stop("input-timeout")
            self.active_input_source = None
            self.claim_time = None

    def status(self) -> dict[str, Any]:
        return {
            "backend": self.backend.name,
            "selected_arm": self.arm,
            "frame": self.frame,
            "manual_enabled": self.enabled,
            "owner": self.active.source if self.active else "none",
            "active_input_source": self.active_input_source or "none",
            "motion": self.active.axis + ("+" if self.active.sign > 0 else "-") if self.active else "stopped",
            "linear_speed_mm_s": self.linear,
            "angular_speed_deg_s": self.angular,
            "command_linear_mm_s": self.active.linear_mm_s if self.active else 0.0,
            "command_angular_deg_s": self.active.angular_deg_s if self.active else 0.0,
            "fault": self.fault,
            "last_stop_reason": self.last_stop_reason,
        }
