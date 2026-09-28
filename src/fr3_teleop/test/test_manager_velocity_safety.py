"""Manager-only safety routing tests; no ROS graph or robot is used."""

from types import SimpleNamespace
import json

from std_msgs.msg import Float64MultiArray, Int8, String

from fr3_teleop.manager import TeleopManager


class _Core:
    def __init__(self):
        self.arm = "arm_a"
        self.commands = []
        self.fault = ""
        self.active = None

    def command(self, command):
        self.commands.append(command)


class _StatusHarness:
    def __init__(self):
        self._servo_status = {}
        self.core = _Core()
        self._manual_transition = "MANUAL_READY"
        self._manual_ready = True
        self.faults = []

    def _integrator_fault(self, arm, reason):
        self.faults.append((arm, reason))


def test_servo_invalid_status_is_a_fault_but_deceleration_warnings_are_not():
    manager = _StatusHarness()
    TeleopManager._on_servo_status(manager, "arm_a", Int8(data=-1))
    assert manager.faults == [("arm_a", "MoveIt Servo safety halt status=-1")]
    for warning in (1, 3, 6):
        TeleopManager._on_servo_status(manager, "arm_a", Int8(data=warning))
    assert len(manager.faults) == 1


class _MotionIgnoredHarness:
    _on_command = TeleopManager._on_command

    def __init__(self):
        self.core = _Core()
        self.core.fault = "raw-velocity-timeout"
        self._manual_ready = False
        self._manual_transition = "FAULT"
        self.config = {"teleop": {"backend": "servo", "first_real_test_mode": False}}
        self.warnings = []

    def get_logger(self):
        return SimpleNamespace(
            warning=self.warnings.append,
            error=lambda _message: None,
        )


def test_motion_while_manual_not_ready_preserves_existing_fault():
    manager = _MotionIgnoredHarness()
    message = String()
    message.data = json.dumps({"type": "motion", "axis": "x", "sign": 1, "source": "gui"})
    manager._on_command(message)
    assert manager.core.fault == "raw-velocity-timeout"
    assert manager._manual_transition == "FAULT"
    assert manager.core.commands == []
    assert manager.warnings == ["motion ignored: manual control is not ready"]


class _RecoveryClient:
    def __init__(self, events):
        self.events = events
        self.state_callback = None
        self.release_callback = None

    def get_controller_state(self, arm, callback):
        self.events.append(("get-state", arm))
        self.state_callback = callback
        return True

    def release_manual(self, arm, callback):
        self.events.append(("release", arm))
        self.release_callback = callback
        return True


class _FaultRecoveryHarness:
    _begin_fault_recovery = TeleopManager._begin_fault_recovery
    _on_fault_recovery_pair_state = TeleopManager._on_fault_recovery_pair_state
    _on_fault_recovery_auto = TeleopManager._on_fault_recovery_auto
    _fault_recovery_failed = TeleopManager._fault_recovery_failed

    def __init__(self):
        self.events = []
        self.core = _Core()
        self.core.fault = "raw-velocity-timeout"
        self._manual_ready = False
        self._manual_transition = "FAULT"
        self._controller_client = _RecoveryClient(self.events)
        self._on_fault_recovery_manual_switch = lambda *_args: None

    def _cancel_hold(self, arm):
        self.events.append(("cancel-hold", arm))

    def _hold_integrator(self, arm, event):
        self.events.append(("fixed-hold", arm, event))

    def _request_manual_from_auto(self, callback):
        self.events.append(("request-manual", callback))


def test_fault_retry_holds_then_restores_auto_before_requesting_manual_again():
    manager = _FaultRecoveryHarness()
    manager._begin_fault_recovery()
    assert manager._manual_transition == "RECOVERY_CHECKING_AUTO"
    assert manager.events[:2] == [("cancel-hold", "arm_a"), ("fixed-hold", "arm_a", "fault-recovery")]
    assert manager.core.commands == [{"type": "stop", "reason": "fault-recovery", "keep_claim": True}]
    manager._controller_client.state_callback("MANUAL")
    assert manager._manual_transition == "RECOVERY_RELEASING"
    assert manager.events[-1] == ("release", "arm_a")
    manager._controller_client.release_callback(True, "automatic restored")
    assert manager.events[-1][0] == "request-manual"
    # The original fault remains until strict MANUAL acquisition and Initial Hold pass.
    assert manager.core.fault == "raw-velocity-timeout"


def test_fault_retry_controller_restore_failure_remains_fault():
    manager = _FaultRecoveryHarness()
    manager._begin_fault_recovery()
    manager._controller_client.state_callback("MANUAL")
    manager._controller_client.release_callback(False, "switch rejected")
    assert manager._manual_transition == "FAULT"
    assert manager._manual_ready is False
    assert manager.core.fault == "controller AUTO restore failed: switch rejected"


class _Integrator:
    def __init__(self, events):
        self.events = events

    def enter_fixed_hold(self, measured, _now, _event):
        self.events.append("hold")
        return tuple(measured)


class _ControllerClient:
    def __init__(self, events):
        self.events = events

    def release_manual(self, arm, _callback):
        self.events.append(("release", arm))
        return True


class _ShutdownHarness:
    shutdown_safely = TeleopManager.shutdown_safely

    def __init__(self):
        self.events = []
        self._shutdown_started = False
        self.core = _Core()
        self.config = {"teleop": {"backend": "servo"}}
        self._integrators = {"arm_a": _Integrator(self.events)}
        self._manual_transition = "MANUAL_READY"
        self._manual_ready = True
        self._controller_client = _ControllerClient(self.events)

    def _cancel_hold(self, arm):
        self.events.append(("cancel", arm))

    def _measured_for_arm(self, _arm):
        return [1.0] * 6

    def _publish_command(self, arm, command):
        self.events.append(("publish", arm, tuple(command)))

    def _on_release_result(self, *_args):
        pass

    def get_logger(self):
        return SimpleNamespace(warning=lambda _message: None)


class _SettleIntegrator:
    def __init__(self, events):
        self.events = events
        self.fault_latched = False
        self.settling = False
        self.integrator_active = True

    def begin_stop_settling(self, measured, _now, event):
        self.events.append(("settle", event, tuple(measured)))
        self.settling = True
        self.integrator_active = False
        return (0.2,) * 6

    def enter_fixed_hold(self, measured, _now, event):
        self.events.append(("fixed", event, tuple(measured)))
        return tuple(measured)

    def accept_velocity(self, velocity, _now):
        self.events.append(("accept", tuple(velocity)))
        return None


class _StopOrderHarness:
    _on_command = TeleopManager._on_command
    _hold_integrator = TeleopManager._hold_integrator
    _on_servo_raw = TeleopManager._on_servo_raw

    def __init__(self):
        self.events = []
        self.published = []
        self.core = _Core()
        self.core.active = object()
        self.core.events = self.events
        self._manual_ready = True
        self._manual_transition = "MANUAL_READY"
        self._servo_motion_allowed = True
        self.config = {"teleop": {"backend": "servo", "first_real_test_mode": False}}
        self._integrators = {"arm_a": _SettleIntegrator(self.events)}

    def _measured_for_arm(self, _arm):
        return [0.0] * 6

    def _publish_command(self, arm, command):
        self.published.append((arm, tuple(command)))


def _command(core, command):
    core.events.append(("twist-stop", command))
    core.commands.append(command)
    if command.get("type") == "stop":
        core.active = None


def test_button_release_settles_before_zeroing_twist_and_does_not_jump():
    manager = _StopOrderHarness()
    manager.core.command = lambda command: _command(manager.core, command)
    message = String()
    message.data = json.dumps({"type": "stop", "reason": "button-released"})
    manager._on_command(message)
    assert manager.events[0][0] == "settle"
    assert manager.events[0][1] == "stop"
    assert manager.events[1][0] == "twist-stop"
    assert manager.events[1][1]["type"] == "stop"
    assert manager.published == [("arm_a", (0.2,) * 6)]
    assert manager._manual_transition == "MANUAL_READY"
    assert manager._manual_ready is True
    assert manager.core.fault == ""
    raw = Float64MultiArray()
    raw.data = [0.4] * 6
    manager._on_servo_raw("arm_a", raw)
    assert not any(event[0] == "accept" for event in manager.events)


def test_fault_stop_still_captures_feedback_instead_of_settling():
    manager = _StopOrderHarness()
    manager._integrators["arm_a"].fault_latched = True
    manager._hold_integrator("arm_a", "stop")
    assert manager.events == [("fixed", "stop", (0.0,) * 6)]
    assert manager.published == [("arm_a", (0.0,) * 6)]


def test_shutdown_holds_before_attempting_manual_controller_release():
    manager = _ShutdownHarness()
    manager.shutdown_safely()
    assert manager.core.commands == [{"type": "stop", "reason": "manager-shutdown"}]
    hold_index = manager.events.index("hold")
    release_index = manager.events.index(("release", "arm_a"))
    assert hold_index < release_index
    assert sum(event[0] == "publish" for event in manager.events if isinstance(event, tuple)) == 3
    manager.shutdown_safely()
    assert manager.events.count(("release", "arm_a")) == 1
