from copy import deepcopy

from fr3_teleop.core import ControlCore, MockBackend, SourceRejected


CONFIG = {
    "robots": {"arm_a": {"enabled": True}, "arm_b": {"enabled": True}},
    "teleop": {
        "default_arm": "arm_a", "default_frame": "world",
        "linear_speed_mm_s": 5.0, "angular_speed_deg_s": 2.0,
        "max_linear_speed_mm_s": 20.0, "max_angular_speed_deg_s": 10.0,
        "input_rate_hz": 20, "input_timeout_ms": 250,
        "start_with_control_disabled": True,
    },
}


def ready_core():
    backend = MockBackend()
    core = ControlCore(deepcopy(CONFIG), backend)
    core.command({"type": "enable", "value": True}, now=0.0)
    return core, backend


def test_hold_then_release_stops():
    core, backend = ready_core()
    core.command({"type": "motion", "axis": "x", "sign": 1, "source": "gui"}, now=1.0)
    assert backend.active["arm_a"].axis == "x"
    core.command({"type": "stop", "reason": "button-released"}, now=1.1)
    assert not backend.active


def test_switch_stops_old_arm_and_does_not_carry_motion():
    core, backend = ready_core()
    core.command({"type": "motion", "axis": "z", "sign": 1, "source": "gui"}, now=1.0)
    core.command({"type": "select_arm", "arm": "arm_b"}, now=1.1)
    assert core.arm == "arm_b"
    assert core.active is None
    assert backend.history[-1] == ("stop", ("arm_a", "robot-switch"))


def test_input_timeout_stops():
    core, backend = ready_core()
    core.command({"type": "motion", "axis": "rz", "sign": -1, "source": "joystick"}, now=1.0)
    core.tick(now=1.251)
    assert core.active is None
    assert backend.history[-1] == ("stop", ("arm_a", "input-timeout"))


def test_non_owned_arm_is_rejected():
    core, _ = ready_core()
    try:
        core.command({"type": "motion", "arm": "arm_b", "axis": "x", "sign": 1}, now=1.0)
    except ValueError as exc:
        assert "non-owned" in str(exc)
    else:
        raise AssertionError("wrong arm accepted")


def test_frame_switch_stops_and_speed_is_limited():
    core, backend = ready_core()
    core.command({"type": "motion", "axis": "y", "sign": 1, "source": "gui"}, now=1.0)
    core.command({"type": "set_frame", "frame": "tool"}, now=1.1)
    core.command({"type": "set_speed", "linear_mm_s": 999, "angular_deg_s": 999}, now=1.2)
    assert core.frame == "tool" and core.linear == 20.0 and core.angular == 10.0
    assert backend.history[-1][0] == "stop"


def test_motion_scale_cannot_exceed_manager_limit():
    core, backend = ready_core()
    core.command({"type": "motion", "axis": "x", "sign": 1, "scale": 0.5, "source": "joystick"}, now=1.0)
    assert backend.active["arm_a"].linear_mm_s == 2.5
    assert backend.active["arm_a"].angular_deg_s == 1.0
    try:
        core.command({"type": "motion", "axis": "x", "sign": 1, "scale": 2.0, "source": "joystick"}, now=1.1)
    except ValueError as exc:
        assert "scale" in str(exc)
    else:
        raise AssertionError("scale above 1 accepted")
    core.command({"type": "set_speed", "linear_mm_s": 999, "angular_deg_s": 999, "source": "joystick"}, now=1.2)
    core.command({"type": "motion", "axis": "z", "sign": -1, "scale": 1.0, "source": "joystick"}, now=1.3)
    assert backend.active["arm_a"].linear_mm_s == 20.0
    assert backend.active["arm_a"].angular_deg_s == 10.0


def test_foreign_source_cannot_steal_or_disable():
    core, backend = ready_core()
    core.command({"type": "motion", "axis": "x", "sign": 1, "source": "gui"}, now=1.0)
    try:
        core.command({"type": "motion", "axis": "z", "sign": 1, "source": "joystick"}, now=1.1)
    except SourceRejected:
        pass
    else:
        raise AssertionError("foreign motion accepted")
    try:
        core.command({"type": "enable", "value": False, "source": "joystick"}, now=1.2)
    except SourceRejected:
        pass
    else:
        raise AssertionError("foreign disable accepted")
    assert core.enabled and backend.active["arm_a"].axis == "x"
    core.command({"type": "stop", "reason": "button-released", "source": "joystick"}, now=1.3)
    assert backend.active["arm_a"].axis == "x"
    core.command({"type": "stop", "reason": "operator-stop", "source": "joystick"}, now=1.4)
    assert core.active is None and core.active_input_source is None


def test_switch_claim_blocks_new_motion_until_release():
    core, backend = ready_core()
    core.command({"type": "motion", "axis": "x", "sign": 1, "source": "joystick"}, now=1.0)
    core.command({"type": "stop", "reason": "robot-switch", "source": "joystick", "keep_claim": True}, now=1.05)
    core.command({"type": "select_arm", "arm": "arm_b", "source": "joystick", "claim": True}, now=1.06)
    assert core.arm == "arm_b" and core.active is None
    assert backend.history[-1] == ("stop", ("arm_a", "robot-switch"))
    try:
        core.command({"type": "motion", "axis": "x", "sign": 1, "source": "gui"}, now=1.1)
    except SourceRejected:
        pass
    else:
        raise AssertionError("gui inherited the switch")
    core.command({"type": "hold", "source": "joystick"}, now=1.2)
    core.tick(now=1.4)
    assert core.active_input_source == "joystick"
    core.tick(now=1.46)
    assert core.active_input_source is None
