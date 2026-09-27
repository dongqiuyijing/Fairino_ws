"""Manager-only safety routing tests; no ROS graph or robot is used."""

from types import SimpleNamespace
import json

from std_msgs.msg import Int8, String

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
