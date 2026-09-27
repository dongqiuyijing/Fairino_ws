"""Physical joystick to the existing teleop command channel.

The adapter publishes the same JSON intents as the GUI. It never opens a
robot SDK, a Servo topic, or a controller manager.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from time import monotonic
from typing import Any

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import yaml

SOURCE = "joystick"
_AXIS_NAMES = {
    "linear_x": "x",
    "linear_y": "y",
    "linear_z": "z",
    "angular_x": "rx",
    "angular_y": "ry",
    "angular_z": "rz",
}
_PRIORITY = {"x": 0, "y": 1, "z": 2, "rx": 3, "ry": 4, "rz": 5}


def load_joystick_config(path: str = "") -> dict[str, Any]:
    filename = Path(path) if path else Path(get_package_share_directory("fr3_teleop")) / "config" / "joystick.yaml"
    with filename.open(encoding="utf-8") as handle:
        loaded = yaml.safe_load(handle)
    if not isinstance(loaded, dict) or not isinstance(loaded.get("joystick"), dict):
        raise ValueError("joystick configuration must contain a joystick mapping")
    config = loaded["joystick"]
    deadzone = float(config["deadzone"])
    if not 0.0 < deadzone < 1.0:
        raise ValueError("deadzone must be in (0, 1)")
    if int(config["joy_timeout_ms"]) <= 0 or int(config["publish_rate_hz"]) <= 0:
        raise ValueError("joy timing parameters must be positive")
    for name in _AXIS_NAMES:
        spec = config["axes"][name]
        if int(spec["index"]) < 0:
            raise ValueError(f"{name} axis index must be >= 0")
    for name in ("deadman", "rotation_mode", "arm_a", "arm_b", "stop"):
        if int(config["buttons"][name]) < 0:
            raise ValueError(f"{name} button index must be >= 0")
    return config


class JoystickMapper:
    """Pure mapping from one Joy sample to teleop command dicts."""

    def __init__(self, config: dict[str, Any]) -> None:
        self.deadzone = float(config["deadzone"])
        self.timeout_s = float(config["joy_timeout_ms"]) / 1000.0
        self.axes = {
            name: {"index": int(config["axes"][name]["index"]), "invert": bool(config["axes"][name]["invert"])}
            for name in _AXIS_NAMES
        }
        buttons = config["buttons"]
        self.deadman = int(buttons["deadman"])
        self.rotation_mode = int(buttons["rotation_mode"])
        self.arm_a = int(buttons["arm_a"])
        self.arm_b = int(buttons["arm_b"])
        self.stop_button = int(buttons["stop"])
        self.stick_indices = sorted({spec["index"] for spec in self.axes.values()})
        self._previous_buttons: list[int] | None = None
        self.last_joy_time: float | None = None
        self.timed_out = False
        self.latched = False
        self.claimed = False
        self.moving = False
        self._held_axis: str | None = None
        self._armed_enable = False

    def update(
        self,
        axes: list[float],
        buttons: list[int],
        now: float,
        blocked: bool = False,
        already_enabled: bool = False,
    ) -> list[dict[str, Any]]:
        axes = [float(value) for value in axes]
        buttons = [int(value) for value in buttons]
        self.last_joy_time = now
        self.timed_out = False
        if self._previous_buttons is None:
            self._previous_buttons = buttons
            return []
        if blocked:
            self._previous_buttons = buttons
            return []

        commands: list[dict[str, Any]] = []
        deadman = self._down(buttons, self.deadman)
        if deadman and not already_enabled and not self._armed_enable:
            commands.append({"type": "enable", "value": True, "source": SOURCE})
            self._armed_enable = True
        if not deadman:
            self._armed_enable = False
        if self._rising(buttons, self.stop_button):
            self.latched = True
            self.moving = False
            self._held_axis = None
            commands.append(self._stop("joystick-stop"))
            self.claimed = False
        if not self.latched and (self._rising(buttons, self.arm_a) or self._rising(buttons, self.arm_b)):
            arm = "arm_a" if self._rising(buttons, self.arm_a) else "arm_b"
            if deadman and self._deflected(axes):
                self.latched = True
                self.moving = False
                self._held_axis = None
                commands.append(self._stop("robot-switch", keep_claim=True))
                commands.append({"type": "select_arm", "arm": arm, "source": SOURCE, "claim": True})
                self.claimed = True
            else:
                commands.append({"type": "select_arm", "arm": arm, "source": SOURCE})
        if self.latched:
            commands.extend(self._while_latched(axes, deadman))
            self._previous_buttons = buttons
            return commands
        if not deadman:
            if self.claimed or self.moving:
                commands.append(self._stop("deadman-released"))
                self.claimed = False
                self.moving = False
                self._held_axis = None
            self._previous_buttons = buttons
            return commands

        picked = self._pick(self._mode_values(axes, buttons))
        if picked is None:
            if self.moving:
                commands.append(self._stop("axis-centered", keep_claim=True))
                self.moving = False
            commands.append({"type": "hold", "source": SOURCE})
            self.claimed = True
        else:
            axis, shaped = picked
            commands.append({
                "type": "motion",
                "axis": axis,
                "sign": 1 if shaped > 0.0 else -1,
                "scale": min(abs(shaped), 1.0),
                "source": SOURCE,
            })
            self.moving = True
            self.claimed = True
        self._previous_buttons = buttons
        return commands

    def timeout(self, now: float) -> list[dict[str, Any]]:
        if self.last_joy_time is None or self.timed_out:
            return []
        if now - self.last_joy_time <= self.timeout_s:
            return []
        self.timed_out = True
        self.latched = True
        self._held_axis = None
        commands: list[dict[str, Any]] = []
        if self.claimed or self.moving:
            commands.append(self._stop("joy-timeout"))
        self.claimed = False
        self.moving = False
        return commands

    def _while_latched(self, axes: list[float], deadman: bool) -> list[dict[str, Any]]:
        if not deadman and not self._deflected(axes):
            self.latched = False
            self._held_axis = None
            if self.claimed or self.moving:
                self.claimed = False
                self.moving = False
                return [self._stop("rearm")]
            return []
        if deadman:
            self.claimed = True
            return [{"type": "hold", "source": SOURCE}]
        if self.claimed or self.moving:
            self.claimed = False
            self.moving = False
            return [self._stop("deadman-released")]
        return []

    def _mode_values(self, axes: list[float], buttons: list[int]) -> dict[str, float]:
        if self._down(buttons, self.rotation_mode):
            names = ("angular_x", "angular_y", "angular_z")
        else:
            names = ("linear_x", "linear_y", "linear_z")
        return {_AXIS_NAMES[name]: self._shaped(axes, name) for name in names}

    def _shaped(self, axes: list[float], name: str) -> float:
        spec = self.axes[name]
        if spec["index"] >= len(axes):
            return 0.0
        value = -float(axes[spec["index"]]) if spec["invert"] else float(axes[spec["index"]])
        magnitude = abs(value)
        if magnitude < self.deadzone or math.isnan(magnitude):
            return 0.0
        magnitude = min(magnitude, 1.0)
        shaped = (magnitude - self.deadzone) / (1.0 - self.deadzone)
        return math.copysign(shaped, value)

    def _pick(self, values: dict[str, float]) -> tuple[str, float] | None:
        nonzero = {axis: value for axis, value in values.items() if value != 0.0}
        if not nonzero:
            self._held_axis = None
            return None
        best = max(nonzero, key=lambda axis: (abs(nonzero[axis]), -_PRIORITY[axis]))
        held = self._held_axis
        if held in nonzero and abs(nonzero[held]) >= 0.6 * abs(nonzero[best]):
            best = held
        self._held_axis = best
        return best, nonzero[best]

    def _deflected(self, axes: list[float]) -> bool:
        return any(index < len(axes) and abs(axes[index]) >= self.deadzone for index in self.stick_indices)

    def _down(self, buttons: list[int], index: int) -> bool:
        return 0 <= index < len(buttons) and buttons[index] != 0

    def _rising(self, buttons: list[int], index: int) -> bool:
        previous = self._previous_buttons or []
        return self._down(buttons, index) and not self._down(previous, index)

    @staticmethod
    def _stop(reason: str, keep_claim: bool = False) -> dict[str, Any]:
        command: dict[str, Any] = {"type": "stop", "reason": reason, "source": SOURCE}
        if keep_claim:
            command["keep_claim"] = True
        return command


class JoystickAdapter(Node):
    """ROS wrapper. Motion still goes through /fr3_teleop/command only."""

    def __init__(self) -> None:
        super().__init__("fr3_joystick_adapter")
        self.declare_parameter("config_file", "")
        config = load_joystick_config(str(self.get_parameter("config_file").value))
        self.mapper = JoystickMapper(config)
        self._blocked = False
        self._manual_enabled = False
        self._latest: Joy | None = None
        self._stamp: float | None = None
        self._logged: tuple | None = None
        joy_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Joy, str(config["joy_topic"]), self._on_joy, joy_qos)
        self.create_subscription(String, str(config["status_topic"]), self._on_status, 20)
        self._command_pub = self.create_publisher(String, str(config["command_topic"]), 30)
        self.create_timer(1.0 / float(config["publish_rate_hz"]), self._on_timer)
        self.get_logger().info(
            "joystick adapter publishing teleop intents only; "
            f"deadman=buttons[{self.mapper.deadman}] deadzone={self.mapper.deadzone}"
        )

    def _on_joy(self, msg: Joy) -> None:
        self._latest = msg
        self._stamp = monotonic()

    def _on_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(data, dict):
            return
        source = str(data.get("active_input_source") or "none")
        self._blocked = source not in {"none", SOURCE}
        self._manual_enabled = bool(data.get("manual_enabled", False))

    def _on_timer(self) -> None:
        if self._latest is None or self._stamp is None:
            return
        now = monotonic()
        if now - self._stamp > self.mapper.timeout_s:
            commands = self.mapper.timeout(now)
        else:
            commands = self.mapper.update(
                list(self._latest.axes),
                list(self._latest.buttons),
                now,
                blocked=self._blocked,
                already_enabled=self._manual_enabled,
            )
        for command in commands:
            signature = (command["type"], command.get("reason"), command.get("axis"), command.get("arm"))
            if command["type"] != "hold" and signature != self._logged:
                self._logged = signature
                self.get_logger().info(
                    f"joystick intent: {command['type']} {command.get('reason', command.get('axis', command.get('arm', '')))}"
                )
            message = String()
            message.data = json.dumps(command)
            self._command_pub.publish(message)


def main() -> None:
    rclpy.init()
    node = JoystickAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
