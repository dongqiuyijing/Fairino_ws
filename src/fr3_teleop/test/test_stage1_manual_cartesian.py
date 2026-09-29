"""Stage 1 ownership and command routing. No robot and no ROS graph."""

from types import SimpleNamespace
import json
from copy import deepcopy

import pytest

from std_msgs.msg import Float64, String

from fr3_teleop.manager import TeleopManager
from fr3_teleop.core import ControlCore, MockBackend
from fr3_teleop.stage1_cartesian import (
    controller_pair_state,
    is_stage1_motion,
    manual_namespace,
    stage1_velocity_mm_s,
    switch_lists,
)


def test_arm_a_manual_deactivates_only_the_trajectory_controller():
    activate, deactivate = switch_lists(
        "arm_a", True, True, "arm_a_controller", "arm_a_teleop_controller")
    assert activate == []
    assert deactivate == ["arm_a_controller"]
    assert "arm_a_teleop_controller" not in activate
    assert "arm_a_teleop_controller" not in deactivate


def test_arm_a_auto_activates_only_the_trajectory_controller():
    activate, deactivate = switch_lists(
        "arm_a", False, True, "arm_a_controller", "arm_a_teleop_controller")
    assert activate == ["arm_a_controller"]
    assert deactivate == []


def test_arm_b_switch_is_unchanged_while_stage1_is_enabled():
    assert switch_lists("arm_b", True, True, "arm_b_controller", "arm_b_teleop_controller") == (
        ["arm_b_teleop_controller"], ["arm_b_controller"])
    assert switch_lists("arm_b", False, True, "arm_b_controller", "arm_b_teleop_controller") == (
        ["arm_b_controller"], ["arm_b_teleop_controller"])


def test_arm_a_manual_means_both_controllers_inactive():
    assert controller_pair_state("arm_a", "inactive", "inactive", True) == "MANUAL"
    assert controller_pair_state("arm_a", "active", "inactive", True) == "AUTO"
    assert controller_pair_state("arm_a", "inactive", "active", True) == "FAULT"
    assert controller_pair_state("arm_b", "active", "inactive", True) == "AUTO"
    assert controller_pair_state("arm_b", "inactive", "active", True) == "MANUAL"


def test_stage1_velocity_is_fixed_base_x_positive():
    config = {"teleop": {"default_linear_speed_mm_s": 10.0}}
    assert stage1_velocity_mm_s(config) == 10.0
    assert is_stage1_motion("arm_a", "base", "x", 1)
    assert not is_stage1_motion("arm_a", "world", "x", 1)
    assert not is_stage1_motion("arm_a", "base", "y", 1)
    assert not is_stage1_motion("arm_a", "base", "x", -1)
    assert not is_stage1_motion("arm_b", "base", "x", 1)
    assert manual_namespace("/arm_a/fairino_gripper/command") == "/arm_a/fairino_manual"
    assert manual_namespace("/arm_b/fairino_gripper/command") == "/arm_b/fairino_manual"


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Core:
    def __init__(self):
        self.arm = "arm_a"
        self.frame = "base"
        self.commands = []
        self.fault = ""
        self.active = None
        self.enabled = False

    def command(self, command):
        self.commands.append(command)
        if command.get("type") == "motion":
            self.active = SimpleNamespace(arm="arm_a", frame="base", axis="x", sign=1, linear_mm_s=10.0)
        if command.get("type") == "stop":
            self.active = None
        if command.get("type") == "enable":
            self.enabled = bool(command.get("value", False))

    def _stop(self, reason):
        self.active = None
        self.commands.append({"type": "internal-stop", "reason": reason})

    def _reject_if_foreign(self, command):
        return str(command.get("source") or "")


class _Stage1CommandHarness:
    _on_command = TeleopManager._on_command
    _handle_stage1_motion = TeleopManager._handle_stage1_motion
    _publish_stage1_velocity = TeleopManager._publish_stage1_velocity

    def __init__(self):
        self.core = _Core()
        self._stage1 = True
        self._manual_ready = True
        self._manual_transition = "MANUAL_READY"
        self._servo_motion_allowed = True
        self._stage1_pub = _Publisher()
        self.config = {
            "teleop": {
                "backend": "servo",
                "default_linear_speed_mm_s": 10.0,
                "first_real_test_mode": False,
            }
        }
        self.warnings = []
        self.errors = []

    def get_logger(self):
        return SimpleNamespace(warning=self.warnings.append, error=self.errors.append)


def _send(manager, payload):
    message = String()
    message.data = json.dumps(payload)
    manager._on_command(message)


def test_stage1_motion_publishes_10_mm_s_and_does_not_touch_the_integrator():
    manager = _Stage1CommandHarness()
    manager._hold_integrator = lambda *args: (_ for _ in ()).throw(AssertionError("integrator used"))
    _send(manager, {"type": "motion", "axis": "x", "sign": 1, "source": "stage1"})
    assert manager._stage1_pub.messages
    assert isinstance(manager._stage1_pub.messages[-1], Float64)
    assert manager._stage1_pub.messages[-1].data == 10.0
    assert manager.core.commands[-1]["type"] == "motion"


def test_stage1_stop_publishes_zero_and_does_not_use_the_integrator():
    manager = _Stage1CommandHarness()
    manager._hold_integrator = lambda *args: (_ for _ in ()).throw(AssertionError("integrator used"))
    _send(manager, {"type": "stop", "reason": "button-released", "source": "stage1"})
    assert manager._stage1_pub.messages[-1].data == 0.0
    assert manager.core.commands[-1]["type"] == "stop"
    assert manager._manual_transition == "MANUAL_READY"


def test_stage1_rejects_non_base_x_without_leaving_manual():
    manager = _Stage1CommandHarness()
    _send(manager, {"type": "motion", "axis": "y", "sign": 1, "source": "stage1"})
    assert manager.core.fault == "Stage 1 only permits Arm A / base / X+"
    assert manager._stage1_pub.messages[-1].data == 0.0
    assert manager._manual_transition == "MANUAL_READY"


def test_stage1_enable_on_arm_b_does_not_request_a_controller_switch():
    manager = _Stage1CommandHarness()
    manager.core.arm = "arm_b"
    manager._handle_enable = TeleopManager._handle_enable.__get__(manager, TeleopManager)
    manager._handle_stage1_enable = TeleopManager._handle_stage1_enable.__get__(manager, TeleopManager)
    calls = []
    manager._controller_client = SimpleNamespace(request_manual=lambda *args: calls.append(args))
    _send(manager, {"type": "enable", "value": True})
    assert calls == []
    assert manager.core.fault == "Stage 1 manual is Arm A only"
    assert manager._manual_transition == "DENIED"


def _real_core_manager():
    manager = _Stage1CommandHarness()
    manager.config = deepcopy({
        "robots": {"arm_a": {"enabled": True}},
        "teleop": {
            "backend": "servo", "default_arm": "arm_a", "default_frame": "base",
            "linear_speed_mm_s": 5.0, "angular_speed_deg_s": 2.0,
            "default_linear_speed_mm_s": 10.0,
            "max_linear_speed_mm_s": 50.0, "max_angular_speed_deg_s": 20.0,
            "input_rate_hz": 20, "input_timeout_ms": 250,
            "start_with_control_disabled": True,
        },
    })
    manager.core = ControlCore(manager.config, MockBackend())
    manager.core.command({"type": "enable", "value": True})
    return manager


def _motion(manager, **extra):
    _send(manager, {"type": "motion", "axis": "x", "sign": 1, "source": "owner", **extra})


def test_stage1_uses_selected_speed_and_scale_instead_of_legacy_default():
    manager = _real_core_manager()
    _motion(manager)
    assert manager._stage1_pub.messages[-1].data == 5.0
    _send(manager, {"type": "set_speed", "linear_mm_s": 1.0, "source": "owner"})
    _motion(manager, scale=0.5)
    assert manager.core.active.linear_mm_s == 0.5
    assert manager._stage1_pub.messages[-1].data == 0.5


def test_stage1_keeps_core_speed_limit():
    manager = _real_core_manager()
    _send(manager, {"type": "set_speed", "linear_mm_s": 999.0})
    _motion(manager, scale=0.5)
    assert manager._stage1_pub.messages[-1].data == 25.0


def test_trace_records_actual_publication_and_foreign_rejection(tmp_path):
    from fr3_teleop.manual_trace import ManualTrace
    manager = _real_core_manager()
    manager._manual_trace = ManualTrace(tmp_path)
    _send(manager, {"type": "set_speed", "linear_mm_s": 10.0, "source": "owner"})
    _motion(manager, scale=1.0)
    _send(manager, {"type": "release", "source": "foreign"})
    _send(manager, {"type": "stop", "reason": "operator-stop"})
    manager._manual_trace.close()
    events = [json.loads(line) for line in manager._manual_trace.path.read_text().splitlines()]
    assert [e["velocity_mm_s"] for e in events if e["kind"] == "velocity_published"] == [10.0, 0.0]
    assert sum(e["kind"] == "command_received" for e in events) == 4
    assert sum(e["kind"] == "command_finished" for e in events) == 4
    assert sum(e["kind"] == "source_rejected" for e in events) == 1


@pytest.mark.parametrize("scale", [0.0, -1.0, 1.01, float("nan"), float("inf")])
def test_stage1_invalid_scale_stops_output(scale):
    manager = _real_core_manager()
    _motion(manager, scale=scale)
    assert manager._stage1_pub.messages[-1].data == 0.0
    assert manager.core.active is None


def test_stage1_nonfinite_selected_velocity_stops_output():
    manager = _real_core_manager()
    _send(manager, {"type": "set_speed", "linear_mm_s": float("nan")})
    _motion(manager)
    assert manager._stage1_pub.messages[-1].data == 0.0
    assert manager.core.active is None


@pytest.mark.parametrize("kind", ["stop", "release"])
def test_stage1_foreign_ordinary_stop_does_not_publish_or_change_motion(kind):
    manager = _real_core_manager()
    _motion(manager)
    before = list(manager._stage1_pub.messages)
    motion = manager.core.active
    _send(manager, {"type": kind, "source": "foreign", "reason": "button-released"})
    assert manager._stage1_pub.messages == before
    assert manager.core.active is motion
    assert manager.core.active_input_source == "owner"


@pytest.mark.parametrize("kind", ["stop", "release"])
def test_stage1_owner_stop_immediately_publishes_zero(kind):
    manager = _real_core_manager()
    _motion(manager)
    _send(manager, {"type": kind, "source": "owner", "reason": "button-released"})
    assert manager._stage1_pub.messages[-1].data == 0.0
    assert manager.core.active is None


@pytest.mark.parametrize("stop", [
    {"source": "foreign", "reason": "operator-stop"},
    {"source": "foreign"},
    {},
])
def test_stage1_global_operator_stop_remains_effective(stop):
    manager = _real_core_manager()
    _motion(manager)
    _send(manager, {"type": "stop", **stop})
    assert manager._stage1_pub.messages[-1].data == 0.0
    assert manager.core.active is None
    assert manager.core.active_input_source is None


def test_stage1_active_timer_does_not_refresh_or_zero_motion_and_timeout_stops():
    manager = _real_core_manager()
    _motion(manager)
    before = list(manager._stage1_pub.messages)
    TeleopManager._refresh_stage1_velocity(manager)
    assert manager._stage1_pub.messages == before
    manager.core.tick(manager.core.last_input + 0.251)
    TeleopManager._refresh_stage1_velocity(manager)
    assert manager._stage1_pub.messages[-1].data == 0.0
    assert manager.core.active is None


@pytest.mark.parametrize("guard", ["allow_real_motion", "manual_ready", "core_enabled"])
def test_stage1_velocity_fix_does_not_bypass_motion_guards(guard):
    manager = _real_core_manager()
    if guard == "allow_real_motion":
        manager._servo_motion_allowed = False
    elif guard == "manual_ready":
        manager._manual_ready = False
    else:
        manager.core.enabled = False
    _motion(manager)
    assert all(message.data == 0.0 for message in manager._stage1_pub.messages)
    assert manager.core.active is None
