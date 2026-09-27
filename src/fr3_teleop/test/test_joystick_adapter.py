from copy import deepcopy
from pathlib import Path

from fr3_teleop.core import ControlCore, MockBackend, SourceRejected
from fr3_teleop.joystick_adapter import JoystickMapper, load_joystick_config


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


YAML = Path(__file__).resolve().parents[1] / "config" / "joystick.yaml"
# Live /joy rest sample: triggers on 2 and 5 sit at +1, sticks are ~0.
IDLE = [0.0, 3.0518509447574615e-05, 1.0, 0.0, 3.0518509447574615e-05, 1.0, 0.0, 0.0]


def config():
    return load_joystick_config(str(YAML))


def primed():
    mapper = JoystickMapper(config())
    assert mapper.update(IDLE, [0] * 11, 0.0) == []
    return mapper


def buttons(**down):
    values = [0] * 11
    for index in down.values():
        values[index] = 1
    return values


def test_yaml_matches_measured_pad():
    cfg = config()
    assert cfg["buttons"] == {
        "deadman": 5, "rotation_mode": 4, "arm_a": 0, "arm_b": 1, "stop": 6,
    }
    assert cfg["axes"]["linear_x"]["index"] == 0
    assert cfg["axes"]["linear_y"]["index"] == 1
    assert cfg["axes"]["linear_z"]["index"] == 4
    assert cfg["axes"]["angular_z"]["index"] == 3
    assert cfg["axes"]["linear_x"]["invert"] is True
    assert cfg["axes"]["linear_y"]["invert"] is False
    assert cfg["axes"]["linear_z"]["invert"] is False
    assert cfg["axes"]["angular_y"]["invert"] is False
    assert cfg["deadzone"] == 0.02
    used = {spec["index"] for spec in cfg["axes"].values()}
    assert used.isdisjoint({2, 5, 6, 7})


def test_idle_triggers_and_deadzone_produce_no_motion():
    mapper = primed()
    held = buttons(deadman=5)
    commands = mapper.update(IDLE, held, 0.05)
    assert [item["type"] for item in commands] == ["enable", "hold"]
    noisy = IDLE.copy()
    noisy[0] = -0.01
    commands = mapper.update(noisy, held, 0.10, already_enabled=True)
    assert commands == [{"type": "hold", "source": "joystick"}]


def test_deadman_scales_speed_and_release_stops():
    mapper = primed()
    axes = IDLE.copy()
    axes[0] = -1.0
    held = buttons(deadman=5)
    commands = mapper.update(axes, held, 0.05)
    motion = commands[-1]
    assert motion == {
        "type": "motion", "axis": "x", "sign": 1, "scale": 1.0, "source": "joystick",
    }
    axes[0] = -0.5
    motion = mapper.update(axes, held, 0.10, already_enabled=True)[-1]
    assert motion["axis"] == "x" and motion["sign"] == 1
    assert abs(motion["scale"] - (0.48 / 0.98)) < 1e-9
    released = mapper.update(axes, [0] * 11, 0.15, already_enabled=True)
    assert released == [{"type": "stop", "reason": "deadman-released", "source": "joystick"}]
    assert mapper.update(axes, [0] * 11, 0.20, already_enabled=True) == []


def test_rotation_mode_and_dominant_axis():
    mapper = primed()
    axes = IDLE.copy()
    axes[0] = -0.4
    axes[1] = 1.0
    held = buttons(deadman=5)
    motion = mapper.update(axes, held, 0.05, already_enabled=True)[-1]
    assert motion["axis"] == "y" and motion["sign"] == 1
    axes = IDLE.copy()
    axes[3] = -0.8
    rotating = buttons(deadman=5, mode=4)
    motion = mapper.update(axes, rotating, 0.10, already_enabled=True)[-1]
    assert motion["axis"] == "rz" and motion["sign"] == 1


def test_arm_switch_while_deflected_does_not_carry_motion():
    mapper = primed()
    axes = IDLE.copy()
    axes[0] = -1.0
    held = buttons(deadman=5)
    assert mapper.update(axes, held, 0.05, already_enabled=True)[-1]["type"] == "motion"
    switched = buttons(deadman=5, arm_b=1)
    commands = mapper.update(axes, switched, 0.10, already_enabled=True)
    assert commands[0]["reason"] == "robot-switch" and commands[0]["keep_claim"] is True
    assert commands[1] == {"type": "select_arm", "arm": "arm_b", "source": "joystick", "claim": True}
    assert all(item["type"] != "motion" for item in commands)
    still = mapper.update(axes, switched, 0.15, already_enabled=True)
    assert still == [{"type": "hold", "source": "joystick"}]
    released = mapper.update(axes, [0] * 11, 0.20, already_enabled=True)
    assert released[0]["reason"] == "deadman-released"
    centered = mapper.update(IDLE, [0] * 11, 0.25, already_enabled=True)
    assert centered == []
    resumed = mapper.update(axes, held, 0.30, already_enabled=True)
    assert resumed[-1]["type"] == "motion"


def test_joy_timeout_stops_and_requires_rearm():
    mapper = primed()
    axes = IDLE.copy()
    axes[0] = -1.0
    held = buttons(deadman=5)
    mapper.update(axes, held, 1.0, already_enabled=True)
    assert mapper.timeout(1.1) == []
    stopped = mapper.timeout(1.21)
    assert stopped == [{"type": "stop", "reason": "joy-timeout", "source": "joystick"}]
    assert mapper.timeout(2.0) == []
    again = mapper.update(axes, held, 2.1, already_enabled=True)
    assert all(item["type"] != "motion" for item in again)
    mapper.update(IDLE, [0] * 11, 2.2, already_enabled=True)
    resumed = mapper.update(axes, held, 2.3, already_enabled=True)
    assert resumed[-1]["type"] == "motion"


def test_mock_backend_receives_scaled_command_and_blocks_gui():
    mapper = primed()
    axes = IDLE.copy()
    axes[0] = -0.5
    commands = mapper.update(axes, buttons(deadman=5), 0.05)
    backend = MockBackend()
    core = ControlCore(deepcopy(CONFIG), backend)
    for index, command in enumerate(commands):
        core.command(command, now=1.0 + index * 0.01)
    motion = backend.active["arm_a"]
    assert motion.source == "joystick"
    assert motion.axis == "x" and motion.sign == 1
    assert abs(motion.linear_mm_s - 5.0 * (0.48 / 0.98)) < 1e-9
    assert motion.linear_mm_s <= 20.0
    try:
        core.command({"type": "motion", "axis": "z", "sign": 1, "source": "gui"}, now=1.2)
    except SourceRejected:
        pass
    else:
        raise AssertionError("gui moved during joystick ownership")
    assert backend.active["arm_a"].source == "joystick"
