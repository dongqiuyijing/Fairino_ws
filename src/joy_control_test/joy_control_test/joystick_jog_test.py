"""Arm A base-frame translation jog from /joy.

Default mode only listens to /joy and prints simulated events.
--precheck connects and reads the arm, then disconnects.
--execute-once is a limited real jog. Continuous jog is not enabled:
this SDK documents max_dis as one finite distance, not an open-ended jog.
"""

import argparse
import math
import os
import sys
import threading
import time
from datetime import datetime
from enum import Enum

from joy_control_test.sdk_jog_x_test import (
    JOG_REF_BASE,
    STOP_REF_BASE,
    competing_controllers,
    existing_arm_sockets,
    read_six,
    settled,
    state_snapshot,
    wait_for_state,
)
from joy_control_test.sdk_read_test import ARM_A_IP, import_robot, sdk_code

# vel and acc are percentages. A real jog also needs an explicit finite max_dis.
# This SDK does not document max_dis=0 or any other open-ended jog mode.
VEL_PERCENT = 2
ACC_PERCENT = 10.0
CONTINUOUS_JOG_SUPPORTED = False
JOY_TIMEOUT_S = 0.15
JOY_LOG_INTERVAL_S = 1.0
START_THRESHOLD = 0.30
STOP_THRESHOLD = 0.20
OTHER_AXIS_LIMIT = 0.20
NORMAL_REARM_REASONS = ("stick_center", "rb_released", "axis_conflict")

LEFT_X = 0
LEFT_Y = 1
RIGHT_X = 3
RIGHT_Y = 4
BUTTON_RB = 5
BUTTON_LB = 4
BUTTON_BACK = 6
MIN_AXES = 5
MIN_BUTTONS = 7
# Right stick X is not a translation command. It must stay centered.
KNOWN_BUTTONS = {
    0: "A",
    1: "B",
    4: "LB",
    5: "RB",
    6: "Back",
}

ESTOP_WARNING = (
    "SOFTWARE STOP DID NOT CONFIRM THE ARM IS STOPPED. "
    "Use the physical emergency stop. "
    "Software stop and the Back button are not a certified e-stop."
)


class State(Enum):
    """Jog gesture state. COMPLETE and FAULT refuse another start."""

    READY = "READY"
    STARTING = "STARTING"
    JOGGING = "JOGGING"
    STOPPING = "STOPPING"
    COMPLETE = "COMPLETE"
    FAULT = "FAULT"


TERMINAL = (State.COMPLETE, State.FAULT)
ACTIVE = (State.STARTING, State.JOGGING)


def wall_time():
    """Wall-clock stamp for logs. Safety timeouts do not use this clock."""
    return datetime.now().isoformat(timespec="milliseconds")


def log_event(kind, **fields):
    """Print one diagnostic record and return its kind."""
    body = " ".join(f"{key}={value}" for key, value in fields.items())
    print(f"{wall_time()} {kind} {body}".rstrip(), flush=True)
    return kind


class TranslationJog:
    """One base-frame translation command. nb and direction match StartJOG."""

    def __init__(self, name, nb, direction):
        self.name = name
        self.nb = int(nb)
        self.direction = int(direction)

    def same_as(self, other):
        if other is None:
            return False
        return self.nb == other.nb and self.direction == other.direction


DIRECTION_LOCKS = {
    "x_pos": TranslationJog("X+", 1, 1),
    "x_neg": TranslationJog("X-", 1, 0),
    "y_pos": TranslationJog("Y+", 2, 1),
    "y_neg": TranslationJog("Y-", 2, 0),
    "z_pos": TranslationJog("Z+", 3, 1),
    "z_neg": TranslationJog("Z-", 3, 0),
}


def format_max_dis(max_dis_mm):
    """Render one finite distance. None stays unset; it is not replaced by 10."""
    if max_dis_mm is None:
        return "UNCONFIGURED"
    return str(float(max_dis_mm))


def start_jog_call(jog, max_dis_mm):
    """Python StartJOG arguments for one base translation."""
    return (
        f"StartJOG({JOG_REF_BASE}, {jog.nb}, {jog.direction}, "
        f"{format_max_dis(max_dis_mm)}, vel={VEL_PERCENT}, acc={ACC_PERCENT})"
    )


class ParsedJoy:
    """One /joy sample reduced to the controls this stage uses."""

    def __init__(self, axes, buttons):
        self.valid = False
        self.reason = "incomplete_joy"
        self.axes0 = None
        self.stick_x = None
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.right_x = 0.0
        self.rb = 0
        self.back = 0
        self.lb = 0
        self.a = 0
        self.b = 0
        self.extra_buttons = False
        self.right_x_neutral = False
        self.engaged = []
        self.axes = []
        self.buttons = []
        self._parse(axes, buttons)

    def _parse(self, axes, buttons):
        if axes is None or buttons is None:
            return
        if len(axes) < MIN_AXES or len(buttons) < MIN_BUTTONS:
            return
        try:
            values = [float(axes[index]) for index in (LEFT_X, LEFT_Y, RIGHT_X, RIGHT_Y)]
            pressed = [int(buttons[index]) != 0 for index in range(len(buttons))]
        except (TypeError, ValueError):
            self.reason = "invalid_joy"
            return
        if not all(math.isfinite(value) for value in values):
            self.reason = "invalid_joy"
            return
        self.valid = True
        self.reason = ""
        checked_axes = {LEFT_X, LEFT_Y, RIGHT_X, RIGHT_Y}
        self.axes = [
            float(item) if index in checked_axes else item
            for index, item in enumerate(axes)
        ]
        self.buttons = [int(item) for item in buttons]
        self.axes0 = values[0]
        self.x = -values[0]
        self.y = float(axes[LEFT_Y])
        self.z = float(axes[RIGHT_Y])
        self.right_x = float(axes[RIGHT_X])
        self.stick_x = self.x
        self.rb = 1 if pressed[BUTTON_RB] else 0
        self.lb = 1 if pressed[BUTTON_LB] else 0
        self.back = 1 if pressed[BUTTON_BACK] else 0
        self.a = 1 if pressed[0] else 0
        self.b = 1 if pressed[1] else 0
        self.extra_buttons = any(
            pressed[index]
            for index in range(len(pressed))
            if index not in (BUTTON_RB, BUTTON_BACK)
        )
        self.right_x_neutral = abs(self.right_x) <= STOP_THRESHOLD
        self.engaged = self._engaged_axes()

    def _command_values(self):
        return (("X", 1, self.x), ("Y", 2, self.y), ("Z", 3, self.z))

    def _engaged_axes(self):
        found = []
        for name, nb, value in self._command_values():
            if abs(value) < START_THRESHOLD:
                continue
            direction = 1 if value > 0.0 else 0
            sign = "+" if direction == 1 else "-"
            found.append(TranslationJog("{0}{1}".format(name, sign), nb, direction))
        return found

    def command_value(self, nb):
        for _name, axis_nb, value in self._command_values():
            if axis_nb == nb:
                return value
        return 0.0

    def _other_axes_neutral(self, selected_nb):
        for _name, nb, value in self._command_values():
            if nb != selected_nb and abs(value) > STOP_THRESHOLD:
                return False
        return True

    @property
    def neutral(self):
        quiet = all(abs(value) <= STOP_THRESHOLD for _name, _nb, value in self._command_values())
        return (
            self.valid
            and self.rb == 0
            and quiet
            and self.right_x_neutral
            and self.back == 0
            and not self.extra_buttons
        )

    def start_command(self):
        """Return one translation, or None when the gesture is not a single axis."""
        if not self.valid or self.rb != 1 or self.back or self.extra_buttons:
            return None
        if not self.right_x_neutral or len(self.engaged) != 1:
            return None
        selected = self.engaged[0]
        if not self._other_axes_neutral(selected.nb):
            return None
        return selected

    def violations(self):
        """List button and non-command-axis violations. Command axes are separate."""
        found = []
        if not self.valid:
            return found
        if abs(self.right_x) > OTHER_AXIS_LIMIT:
            found.append({
                "kind": "axis",
                "axis_index": RIGHT_X,
                "name": "right_x",
                "actual_value": self.right_x,
                "expected": "neutral",
                "threshold": OTHER_AXIS_LIMIT,
            })
        for index, value in enumerate(self.buttons):
            if index in (BUTTON_RB, BUTTON_BACK) or int(value) == 0:
                continue
            found.append({
                "kind": "button",
                "button_index": index,
                "name": KNOWN_BUTTONS.get(index, "button_{0}".format(index)),
                "actual_value": int(value),
                "expected": 0,
            })
        return found


class JogController:
    """Joy state machine. It never calls the robot SDK itself."""

    def __init__(self, rearm_after_normal_stop=True, joy_debug=False, direction_lock=None):
        self.lock = threading.RLock()
        self.wake = threading.Event()
        self.rearm_after_normal_stop = bool(rearm_after_normal_stop)
        self.joy_debug = bool(joy_debug)
        self.direction_lock = direction_lock
        self._rejected_direction = None
        self._last_joy_log_mono = None
        self._last_joy_signature = None
        self.state = State.READY
        self.neutral_seen = False
        self.last_valid_joy = None
        self.start_intents = 0
        self.start_sent = False
        self.motion_anchor = None
        self.active_jog = None
        self.stop_reason = ""
        self.fault_on_stop = False
        self.watchdog_fired = False
        self.time_limit_hit = False
        self.start_ret = None
        self.stop_ret = None
        self.confirmed = None
        self._command = None
        self.events = []
        self.last_unsupported = None

    def on_joy(self, axes, buttons, now):
        """Update the gesture from one /joy sample. now is monotonic time."""
        with self.lock:
            sample = ParsedJoy(axes, buttons)
            engaged = ",".join(item.name for item in sample.engaged) or "none"
            self._log_joy(sample, engaged, now)
            if self.state in TERMINAL:
                return
            if sample.valid:
                self.last_valid_joy = now
            if sample.back == 1:
                self.fault_on_stop = False
                self._request_stop("back")
                return
            if self.state in ACTIVE and not sample.valid:
                self._request_stop(sample.reason or "incomplete_joy")
                return
            if self.state in ACTIVE:
                reason = self._active_stop_reason(sample)
                if reason is not None:
                    if reason in ("unsupported_button", "unsupported_axis"):
                        self._record_unsupported(sample, now, reason)
                        self.fault_on_stop = True
                    self._request_stop(reason)
                    return
            if sample.neutral:
                self.neutral_seen = True
            command = sample.start_command()
            if command is not None and not self._direction_allowed(command):
                command = None
            if (
                self.state == State.READY
                and self.neutral_seen
                and command is not None
                and self.start_intents == 0
            ):
                self.start_intents = 1
                self.active_jog = command
                self._command = "start"
                self._transition(State.STARTING, command.name)
                self._record(
                    "START",
                    direction=command.name,
                    nb=command.nb,
                    dir=command.direction,
                    mono=f"{now:.3f}",
                )
                self.wake.set()

    def poll_safety(self, now):
        """Request a stop when /joy goes stale. This thread is not the joy callback."""
        with self.lock:
            if self.state not in ACTIVE:
                return
            if self.last_valid_joy is None or (now - self.last_valid_joy) > JOY_TIMEOUT_S:
                self.watchdog_fired = True
                self.fault_on_stop = True
                self._request_stop("joy_timeout")

    def current_jog(self):
        """Return the translation selected for the current start."""
        with self.lock:
            return self.active_jog

    def _direction_allowed(self, command):
        """True when this gesture matches the real-test lock, if one is set."""
        lock = self.direction_lock
        if lock is None or command.same_as(lock):
            self._rejected_direction = None
            return True
        if self._rejected_direction != command.name:
            self._rejected_direction = command.name
            self._record(
                "DIRECTION_REJECTED",
                requested=command.name,
                locked=lock.name,
                nb=command.nb,
                dir=command.direction,
            )
        return False

    def _active_stop_reason(self, sample):
        """Return a specific stop reason, or None while the same axis may continue."""
        if sample.extra_buttons:
            return "unsupported_button"
        if not sample.right_x_neutral:
            return "unsupported_axis"
        if sample.rb == 0:
            return "rb_released"
        if len(sample.engaged) >= 2:
            return "axis_conflict"
        jog = self.active_jog
        if jog is None:
            return "stick_center"
        value = sample.command_value(jog.nb)
        if jog.direction == 1 and value <= STOP_THRESHOLD:
            return "stick_center"
        if jog.direction == 0 and value >= -STOP_THRESHOLD:
            return "stick_center"
        for _name, nb, other in (("X", 1, sample.x), ("Y", 2, sample.y), ("Z", 3, sample.z)):
            if nb != jog.nb and abs(other) > STOP_THRESHOLD:
                return "axis_conflict"
        return None

    def pop_command(self):
        """Return the one pending SDK command, or None. Stop replaces start."""
        with self.lock:
            command = self._command
            self._command = None
            return command

    def begin_rpc(self, now):
        """Commit to one StartJOG call. A waiting stop cancels it."""
        with self.lock:
            if self.state != State.STARTING or self.start_sent or self._command == "stop":
                return False
            self.start_sent = True
            self.motion_anchor = now
            return True

    def note_start_result(self, ret):
        """Enter JOGGING only when StartJOG succeeded and stop is not waiting."""
        with self.lock:
            self.start_ret = ret
            if ret != 0:
                self.fault_on_stop = True
                self.stop_reason = "startjog_error"
                self._command = "stop"
                if self.state == State.STARTING:
                    self._transition(State.STOPPING, "startjog_error")
                self.wake.set()
                return
            if self.state == State.STOPPING or self._command == "stop":
                return
            if self.state == State.STARTING:
                self._transition(State.JOGGING, "startjog_ok")

    def fail_before_motion(self, reason):
        """Robot state blocked the jog. StartJOG was not called."""
        with self.lock:
            self.start_sent = False
            self.fault_on_stop = True
            self._command = None
            self._transition(State.FAULT, reason)
            self._record("STARTJOG_NOT_CALLED", reason=reason)

    def finish_cancelled(self):
        """Stop won before StartJOG. Do not call StopJOG for a jog that never started."""
        with self.lock:
            if self.state in TERMINAL:
                return
            target = self._state_after_stop(stop_ret=0, confirmed=True)
            self._transition(target, self.stop_reason or "cancelled_before_start")
            self._record("STOPJOG_NOT_CALLED", reason="startjog_was_not_sent")
            if target == State.READY:
                self._reset_for_rearm()

    def finish_motion(self, target, stop_ret, confirmed):
        """Record the stop result. A normal release can return to READY."""
        with self.lock:
            self.stop_ret = stop_ret
            self.confirmed = confirmed
            if self.state in TERMINAL:
                return
            if target == State.COMPLETE:
                target = self._state_after_stop(stop_ret, confirmed)
            self._transition(target, self.stop_reason or target.value)
            if target == State.READY:
                self._reset_for_rearm()

    def _state_after_stop(self, stop_ret, confirmed):
        if self.fault_on_stop or stop_ret != 0 or not confirmed:
            return State.FAULT
        if self.stop_reason == "back":
            return State.COMPLETE
        if self.rearm_after_normal_stop and self.stop_reason in NORMAL_REARM_REASONS:
            return State.READY
        return State.COMPLETE

    def _reset_for_rearm(self):
        """Allow one later StartJOG after a confirmed normal stop."""
        self.start_intents = 0
        self.start_sent = False
        self.motion_anchor = None
        self.active_jog = None
        self.fault_on_stop = False

    def request_exit(self):
        """Ask the SDK thread to stop an active jog before the process exits."""
        with self.lock:
            if self.state in ACTIVE:
                self.fault_on_stop = True
                self._request_stop("node_exit")
            self.wake.set()

    def _request_stop(self, reason):
        if self.state in TERMINAL or self.state == State.STOPPING:
            return
        self.stop_reason = reason
        self._record("STOP_REQUESTED", reason=reason)
        if self.state == State.READY:
            target = State.FAULT if self.fault_on_stop else State.COMPLETE
            self._command = None
            self._transition(target, reason)
            return
        self._command = "stop"
        self._transition(State.STOPPING, reason)
        self.wake.set()

    def _transition(self, target, reason):
        previous = self.state
        self.state = target
        self._record("STATE", previous=previous.value, next=target.value, reason=reason)

    def _record_unsupported(self, sample, now, reason):
        """Save and print the Joy frame behind an unsupported button or axis."""
        violations = sample.violations()
        self.last_unsupported = {
            "monotonic_time": now,
            "state": self.state.value,
            "axes": list(sample.axes),
            "buttons": list(sample.buttons),
            "stick_x_raw": sample.axes0,
            "stick_x_normalized": sample.stick_x,
            "x": sample.x,
            "y": sample.y,
            "z": sample.z,
            "violations": violations,
            "stop_reason": reason,
        }
        lines = [
            "UNSUPPORTED_INPUT_DIAG",
            "timestamp: {0}".format(wall_time()),
            "monotonic_time: {0:.6f}".format(now),
            "state: {0}".format(self.state.value),
            "axes: {0}".format(sample.axes),
            "buttons: {0}".format(sample.buttons),
            "stick_x_raw: {0}".format(sample.axes0),
            "stick_x_normalized: {0}".format(sample.stick_x),
            "RB: {0}".format(sample.rb),
            "LB: {0}".format(sample.lb),
            "A: {0}".format(sample.a),
            "B: {0}".format(sample.b),
            "Back: {0}".format(sample.back),
            "x: {0}".format(sample.x),
            "y: {0}".format(sample.y),
            "z: {0}".format(sample.z),
            "start_threshold: {0}".format(START_THRESHOLD),
            "stop_threshold: {0}".format(STOP_THRESHOLD),
            "neutral_threshold: {0}".format(OTHER_AXIS_LIMIT),
            "violated_conditions:",
        ]
        if not violations:
            lines.append("  - none_recorded")
        for item in violations:
            if item["kind"] == "axis":
                lines.append(
                    "  - axis_index: {0}".format(item["axis_index"])
                )
                lines.append("    name: {0}".format(item["name"]))
                lines.append("    actual_value: {0}".format(item["actual_value"]))
                lines.append("    expected: {0}".format(item["expected"]))
                lines.append("    threshold: {0}".format(item["threshold"]))
            else:
                lines.append(
                    "  - button_index: {0}".format(item["button_index"])
                )
                lines.append("    name: {0}".format(item["name"]))
                lines.append("    actual_value: {0}".format(item["actual_value"]))
                lines.append("    expected: {0}".format(item["expected"]))
        lines.append("stop_reason: {0}".format(reason))
        print("\n".join(lines), flush=True)
        self.events.append("UNSUPPORTED_INPUT_DIAG")

    def _log_joy(self, sample, engaged, now):
        """Record every sample. Printing is summarized unless joy_debug is set."""
        self.events.append("JOY")
        signature = (
            round(sample.x, 3),
            round(sample.y, 3),
            round(sample.z, 3),
            round(sample.right_x, 3),
            sample.rb,
            sample.back,
            sample.valid,
            engaged,
        )
        centered = (
            abs(sample.x) <= STOP_THRESHOLD
            and abs(sample.y) <= STOP_THRESHOLD
            and abs(sample.z) <= STOP_THRESHOLD
            and abs(sample.right_x) <= STOP_THRESHOLD
        )
        unchanged = signature == self._last_joy_signature
        self._last_joy_signature = signature
        if not self.joy_debug:
            if centered and unchanged:
                return
            if (
                self._last_joy_log_mono is not None
                and (now - self._last_joy_log_mono) < JOY_LOG_INTERVAL_S
            ):
                return
        self._last_joy_log_mono = now
        log_event(
            "JOY",
            mono=f"{now:.3f}",
            x=sample.x,
            y=sample.y,
            z=sample.z,
            right_x=sample.right_x,
            rb=sample.rb,
            back=sample.back,
            valid=sample.valid,
            engaged=engaged,
        )

    def _record(self, kind, **fields):
        self.events.append(kind)
        log_event(kind, **fields)


class SimAdapter:
    """Dry-run SDK. It records calls and never opens a robot connection."""

    def __init__(self, max_dis_mm=None):
        self.max_dis_mm = max_dis_mm
        self.start_calls = 0
        self.stop_calls = 0
        self.imm_calls = 0
        self.start_ret = 0
        self.stop_ret = 0
        self.confirm_ok = True
        self.ready = True
        self.tcp_before = None
        self.tcp_after = None
        self.last_jog = None
        self.last_call = ""

    def robot_ready(self):
        return self.ready, "sim"

    def read_tcp(self):
        return self.tcp_before

    def start_jog(self, jog):
        self.start_calls += 1
        self.last_jog = jog
        self.last_call = start_jog_call(jog, self.max_dis_mm)
        log_event("SIMULATED", call=self.last_call)
        return self.start_ret

    def stop_jog(self):
        self.stop_calls += 1
        log_event("SIMULATED", call=f"StopJOG({STOP_REF_BASE})")
        return self.stop_ret

    def imm_stop(self):
        self.imm_calls += 1
        log_event("SIMULATED", call="ImmStopJOG()")
        return 0

    def confirm_stopped(self):
        log_event("SIMULATED", stop_confirm=self.confirm_ok)
        state = {"robot_state": 1 if self.confirm_ok else 2}
        return self.confirm_ok, self.tcp_after, state


class RealAdapter:
    """One FRRobot used only by the SDK thread."""

    def __init__(self, robot, max_dis_mm):
        if max_dis_mm is None or not math.isfinite(max_dis_mm) or max_dis_mm <= 0:
            raise ValueError("real jog requires an explicit positive max_dis")
        self.robot = robot
        self.max_dis_mm = float(max_dis_mm)
        self.tcp_before = None
        self.imm_calls = 0

    def robot_ready(self):
        if not wait_for_state(self.robot, timeout=1.0):
            return False, "cnde_state_missing"
        state = state_snapshot(self.robot)
        log_event("ROBOT_STATE", **state)
        if state["safety_code"] != 0 or state["main_code"] != 0:
            return False, "robot_fault_or_safety_stop"
        if state["robot_state"] != 1:
            return False, "robot_not_stopped"
        return True, "ok"

    def read_tcp(self):
        self.tcp_before = read_six(self.robot, "GetActualTCPPose")
        log_event("TCP_BEFORE", pose=self.tcp_before)
        return self.tcp_before

    def start_jog(self, jog):
        call = start_jog_call(jog, self.max_dis_mm)
        log_event("STARTJOG_CALL", call=call, direction=jog.name)
        return self.robot.StartJOG(
            JOG_REF_BASE,
            jog.nb,
            jog.direction,
            self.max_dis_mm,
            vel=VEL_PERCENT,
            acc=ACC_PERCENT,
        )

    def stop_jog(self):
        log_event("STOPJOG_CALL", call=f"StopJOG({STOP_REF_BASE})")
        return self.robot.StopJOG(STOP_REF_BASE)

    def imm_stop(self):
        self.imm_calls += 1
        log_event("IMMSTOPJOG_CALL", call="ImmStopJOG()")
        return self.robot.ImmStopJOG()

    def confirm_stopped(self):
        tcp, stable = settled(self.robot)
        state = state_snapshot(self.robot)
        confirmed = bool(stable and state["robot_state"] == 1)
        log_event(
            "STOP_CONFIRM",
            tcp_stable=stable,
            robot_state=state["robot_state"],
            confirmed=confirmed,
        )
        return confirmed, tcp, state


class SdkWorker:
    """Serialize every SDK call. Stop preempts a start that has not been sent."""

    def __init__(self, controller, adapter, clock=time.monotonic):
        self.controller = controller
        self.adapter = adapter
        self.clock = clock
        self.tcp_before = None
        self.tcp_after = None

    def process_once(self):
        command = self.controller.pop_command()
        if command is None:
            return False
        if command == "start":
            self._run_start()
            if self.controller.pop_command() == "stop" or self._stop_waiting():
                self._run_stop()
            return True
        if command == "stop":
            self._run_stop()
            return True
        return False

    def _stop_waiting(self):
        return self.controller.state == State.STOPPING

    def _run_start(self):
        ready, detail = self.adapter.robot_ready()
        if not ready:
            self.controller.fail_before_motion(detail)
            return
        try:
            self.tcp_before = self.adapter.read_tcp()
        except Exception as exc:
            log_event("TCP_BEFORE_FAILED", error=f"{type(exc).__name__}: {exc}")
            self.controller.fail_before_motion("tcp_read_failed")
            return
        jog = self.controller.current_jog()
        if jog is None:
            self.controller.fail_before_motion("missing_jog_command")
            return
        if not self.controller.begin_rpc(self.clock()):
            self.controller.finish_cancelled()
            return
        log_event(
            "STARTJOG_CALLED",
            mono=f"{self.clock():.3f}",
            ref=JOG_REF_BASE,
            nb=jog.nb,
            direction=jog.direction,
            name=jog.name,
            max_dis_mm=format_max_dis(self.adapter.max_dis_mm),
            vel_percent=VEL_PERCENT,
            acc_percent=ACC_PERCENT,
        )
        try:
            ret = self.adapter.start_jog(jog)
        except Exception as exc:
            log_event("STARTJOG_EXCEPTION", error=f"{type(exc).__name__}: {exc}")
            ret = -1
        log_event("STARTJOG_RET", ret=ret)
        self.controller.note_start_result(ret)

    def _run_stop(self):
        if not self.controller.start_sent:
            self.controller.finish_cancelled()
            return
        log_event(
            "STOPJOG_CALLED",
            mono=f"{self.clock():.3f}",
            ref=STOP_REF_BASE,
            reason=self.controller.stop_reason,
        )
        try:
            stop_ret = self.adapter.stop_jog()
        except Exception as exc:
            log_event("STOPJOG_EXCEPTION", error=f"{type(exc).__name__}: {exc}")
            stop_ret = -1
        log_event("STOPJOG_RET", ret=stop_ret)
        confirmed = False
        tcp_after = None
        robot_state = None
        if stop_ret != 0:
            log_event("NORMAL_JOG_STOP_FAILED", ret=stop_ret)
            self._imm_stop()
            print(ESTOP_WARNING, flush=True)
        else:
            try:
                confirmed, tcp_after, state = self.adapter.confirm_stopped()
                robot_state = None if state is None else state.get("robot_state")
            except Exception as exc:
                log_event("STOP_CONFIRM_FAILED", error=f"{type(exc).__name__}: {exc}")
                confirmed = False
            self.tcp_after = tcp_after
            if not confirmed:
                log_event("NORMAL_JOG_STOP_FAILED", reason="motion_not_confirmed")
                self._imm_stop()
                print(ESTOP_WARNING, flush=True)
        self._log_motion(tcp_after, robot_state)
        target = State.COMPLETE
        if stop_ret != 0 or not confirmed or self.controller.fault_on_stop:
            target = State.FAULT
        self.controller.finish_motion(target, stop_ret, confirmed)

    def _imm_stop(self):
        try:
            ret = self.adapter.imm_stop()
            log_event("IMMSTOPJOG_RET", ret=ret)
        except Exception as exc:
            log_event("IMMSTOPJOG_EXCEPTION", error=f"{type(exc).__name__}: {exc}")

    def _log_motion(self, tcp_after, robot_state):
        log_event(
            "MOTION_RESULT",
            tcp_before=self.tcp_before,
            tcp_after=tcp_after,
            robot_state=robot_state,
            watchdog=self.controller.watchdog_fired,
            time_limit=self.controller.time_limit_hit,
            max_dis_mm=format_max_dis(self.adapter.max_dis_mm),
        )
        if self.tcp_before is None or tcp_after is None:
            return
        dx = tcp_after[0] - self.tcp_before[0]
        dy = tcp_after[1] - self.tcp_before[1]
        dz = tcp_after[2] - self.tcp_before[2]
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        log_event("TCP_DELTA", dx=dx, dy=dy, dz=dz, distance=distance)
        limit = self.adapter.max_dis_mm
        if limit is not None and distance >= limit:
            log_event(
                "MAX_DIS_REACHED",
                distance=distance,
                note="early_stop_not_verified",
            )


WEBAPP_PORTS = (80, 9999)
BROWSER_NAMES = ("firefox", "chrome", "chromium", "chromium-browser")


def webapp_view_socket(line):
    """True when a browser is only viewing the WebApp on port 80 or 9999."""
    lowered = line.lower()
    if not any(f'"{name}"' in lowered for name in BROWSER_NAMES):
        return False
    for port in WEBAPP_PORTS:
        token = f"{ARM_A_IP}:{port}"
        start = 0
        while True:
            index = line.find(token, start)
            if index < 0:
                break
            end = index + len(token)
            if end >= len(line) or not line[end].isdigit():
                return True
            start = end
    return False


def blocking_arm_sockets(lines):
    """Drop an idle WebApp browser connection. Keep SDK and other control sockets."""
    return [line for line in lines if not webapp_view_socket(line)]


def other_controllers():
    """Return control processes, including another joystick_jog_test instance."""
    found = list(competing_controllers())
    own_pid = os.getpid()
    for pid_text in os.listdir("/proc"):
        if not pid_text.isdigit() or int(pid_text) == own_pid:
            continue
        try:
            command = open(f"/proc/{pid_text}/cmdline", "rb").read().replace(
                b"\x00", b" "
            ).decode(errors="replace")
        except OSError:
            continue
        if "cursorsandbox" in command or command.startswith("/bin/bash"):
            continue
        argv0 = command.split(" ", 1)[0]
        if argv0.endswith("joystick_jog_test"):
            found.append((int(pid_text), "joystick_jog_test", command[:180]))
    sockets = existing_arm_sockets()
    ignored = [line for line in sockets if webapp_view_socket(line)]
    for line in ignored:
        print(f"WebApp view connection ignored: {line}", flush=True)
    return found, blocking_arm_sockets(sockets)


def run_precheck():
    """Connect, read state, print the jog plan, and disconnect. No motion."""
    print("=== joystick_jog_test SDK PRECHECK ===", flush=True)
    print("MODE: PRECHECK", flush=True)
    print("motion commands: none", flush=True)
    found, sockets = other_controllers()
    if found or sockets:
        print("BLOCKED: another FR3 control process is running", flush=True)
        for item in found:
            print(f"process: {item}", flush=True)
        for line in sockets:
            print(f"socket: {line}", flush=True)
        return 2
    print("No other controller: PASS", flush=True)
    robot = None
    try:
        robot_module = import_robot()
        print(f"SDK module: {getattr(robot_module, '__file__', robot_module)}", flush=True)
        robot = robot_module.RPC(ARM_A_IP)
        if not bool(getattr(robot, "is_connect", False)):
            print("PRECHECK FAIL: SDK did not connect", flush=True)
            return 1
        version = robot.GetSoftwareVersion()
        print(f"GetSoftwareVersion return: {version!r}", flush=True)
        if sdk_code(version) != 0:
            print("PRECHECK FAIL: GetSoftwareVersion", flush=True)
            return 1
        if not wait_for_state(robot):
            print("PRECHECK FAIL: CNDE state was not received", flush=True)
            return 1
        joints = read_six(robot, "GetActualJointPosDegree")
        tcp = read_six(robot, "GetActualTCPPose")
        state = state_snapshot(robot)
        print(f"joint_before: {joints}", flush=True)
        print(f"tcp_before: {tcp}", flush=True)
        print(
            "robot state: "
            f"mode={state['robot_mode']} (1 manual, 0 auto), "
            f"motion_state={state['robot_state']}, "
            f"enable={state['rbt_enable_state']}, "
            f"main_code={state['main_code']}, sub_code={state['sub_code']}, "
            f"safety_code={state['safety_code']}",
            flush=True,
        )
        print(
            "Planned StartJOG, one finite segment, continuous jog BLOCKED: "
            f"ref={JOG_REF_BASE}, max_dis=operator --max-dis, "
            f"vel={VEL_PERCENT}, acc={ACC_PERCENT}",
            flush=True,
        )
        print("X+ nb=1 dir=1; X- nb=1 dir=0", flush=True)
        print("Y+ nb=2 dir=1; Y- nb=2 dir=0", flush=True)
        print("Z+ nb=3 dir=1; Z- nb=3 dir=0", flush=True)
        print(f"Planned StopJOG: StopJOG({STOP_REF_BASE})", flush=True)
        print("software duration cutoff: none", flush=True)
        print(f"joy timeout: {JOY_TIMEOUT_S} s", flush=True)
        print("continuous jog: BLOCKED", flush=True)
        print("NO MOTION COMMAND SENT", flush=True)
        if state["safety_code"] != 0 or state["main_code"] != 0:
            print("PRECHECK FAIL: robot fault or safety stop", flush=True)
            return 1
        print("PRECHECK PASS", flush=True)
        return 0
    except Exception as exc:
        print(f"PRECHECK FAIL: {type(exc).__name__}: {exc}", flush=True)
        return 1
    finally:
        if robot is not None:
            try:
                robot.CloseRPC()
                print("CloseRPC finished", flush=True)
            except Exception as exc:
                print(f"CloseRPC failed: {type(exc).__name__}: {exc}", flush=True)


def _joy_qos():
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    # Best effort can receive both reliable and best-effort /joy publishers.
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.VOLATILE,
    )


def run_ros(execute, joy_debug=False, max_dis_mm=None, direction_lock=None):
    """Spin the joy subscriber. execute=True is the only mode that can move."""
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from sensor_msgs.msg import Joy

    print("=== joystick_jog_test ===", flush=True)
    if joy_debug:
        print("joy log: every /joy sample", flush=True)
    else:
        print(
            "joy log: at most 1 Hz; centered unchanged input is not repeated; "
            "--joy-debug prints every sample",
            flush=True,
        )
    if execute:
        print("CONTINUOUS JOG: BLOCKED", flush=True)
        print(
            "WARNING: real base translation is armed. "
            f"max_dis={format_max_dis(max_dis_mm)} mm, "
            f"vel={VEL_PERCENT}, acc={ACC_PERCENT}.",
            flush=True,
        )
        if direction_lock is None:
            print("MODE: ALL SIX DIRECTIONS", flush=True)
            print("X+ X- Y+ Y- Z+ Z- are armed, one axis at a time.", flush=True)
            print(
                "Release the stick or RB, wait until the arm stops, "
                "then select the next direction.",
                flush=True,
            )
        else:
            print("MODE: ONE DIRECTION", flush=True)
            print(
                "direction lock: "
                f"{direction_lock.name} nb={direction_lock.nb} dir={direction_lock.direction}",
                flush=True,
            )
        print("It starts only after neutral, with RB and exactly one axis.", flush=True)
        print("A second StartJOG is not sent while that axis stays held.", flush=True)
        print(ESTOP_WARNING, flush=True)
        found, sockets = other_controllers()
        if found or sockets:
            print("BLOCKED: another FR3 control process is running", flush=True)
            for item in found:
                print(f"process: {item}", flush=True)
            for line in sockets:
                print(f"socket: {line}", flush=True)
            return 2
    else:
        print("MODE: DRY RUN", flush=True)
        print("SDK connection: none", flush=True)
        print("motion commands: none", flush=True)
        print("continuous jog: BLOCKED on the real controller", flush=True)
        locked = "all six" if direction_lock is None else direction_lock.name
        print(
            f"simulated directions: {locked}, "
            f"StartJOG ref={JOG_REF_BASE}, max_dis={format_max_dis(max_dis_mm)}, "
            f"vel={VEL_PERCENT}, acc={ACC_PERCENT}, StopJOG({STOP_REF_BASE})",
            flush=True,
        )
        if direction_lock is not None:
            print(
                "direction lock: "
                f"{direction_lock.name} nb={direction_lock.nb} "
                f"dir={direction_lock.direction}",
                flush=True,
            )

    robot = None
    if execute:
        robot_module = import_robot()
        robot = robot_module.RPC(ARM_A_IP)
        if not bool(getattr(robot, "is_connect", False)):
            print("FAIL: SDK did not connect", flush=True)
            try:
                robot.CloseRPC()
            except Exception:
                pass
            return 1
        adapter = RealAdapter(robot, max_dis_mm)
    else:
        adapter = SimAdapter(max_dis_mm=max_dis_mm)

    # Dry run can always try another direction. A locked real test stays one segment.
    # An unlocked real test can change direction after each confirmed stop.
    allow_next_direction = (not execute) or direction_lock is None
    controller = JogController(
        rearm_after_normal_stop=allow_next_direction,
        joy_debug=joy_debug,
        direction_lock=direction_lock,
    )
    worker = SdkWorker(controller, adapter)
    stop_threads = threading.Event()

    def sdk_loop():
        while not stop_threads.is_set():
            worked = worker.process_once()
            if not worked:
                controller.wake.wait(0.02)
                controller.wake.clear()

    def watchdog_loop():
        while not stop_threads.is_set():
            controller.poll_safety(time.monotonic())
            time.sleep(0.02)

    sdk_thread = threading.Thread(target=sdk_loop, name="sdk_jog_worker", daemon=True)
    watchdog_thread = threading.Thread(
        target=watchdog_loop, name="joy_watchdog", daemon=True
    )
    sdk_thread.start()
    watchdog_thread.start()

    rclpy.init()
    node = Node("joystick_jog_test")

    def on_joy(message):
        # Reception time is monotonic. Do not use the message stamp or sim time.
        controller.on_joy(message.axes, message.buttons, time.monotonic())

    node.create_subscription(Joy, "/joy", on_joy, _joy_qos())
    print("subscribed: /joy", flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print("shutdown requested", flush=True)
    finally:
        controller.request_exit()
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and controller.state in ACTIVE + (State.STOPPING,):
            worker.process_once()
            time.sleep(0.01)
        stop_threads.set()
        controller.wake.set()
        sdk_thread.join(timeout=2.0)
        watchdog_thread.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if robot is not None:
            try:
                robot.CloseRPC()
                print("CloseRPC finished", flush=True)
            except Exception as exc:
                print(f"CloseRPC failed: {type(exc).__name__}: {exc}", flush=True)
    return 0 if controller.state != State.FAULT else 1


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Arm A base translation joystick jog. Default is a /joy dry run."
    )
    parser.add_argument(
        "--precheck",
        action="store_true",
        help="Read Arm A state and print the jog plan. Do not move.",
    )
    parser.add_argument(
        "--execute-once",
        action="store_true",
        help="Arm real finite jogs. Omit --test-direction to allow all six, one at a time.",
    )
    parser.add_argument(
        "--joy-debug",
        action="store_true",
        help="Print every /joy sample. The default summarizes unchanged input.",
    )
    parser.add_argument(
        "--test-direction",
        choices=tuple(DIRECTION_LOCKS),
        help="Optional lock to one direction. Omit it to test all six in one run.",
    )
    parser.add_argument(
        "--max-dis",
        type=float,
        default=None,
        help="Finite jog distance in mm. Required with --execute-once. There is no default.",
    )
    args, unknown = parser.parse_known_args(argv)
    if unknown:
        print(f"ignored arguments: {unknown}", flush=True)
    return args


def execute_configuration_error(args):
    """Refuse a real jog until the operator gives a positive finite distance."""
    if not args.execute_once:
        return None
    if args.max_dis is None:
        return (
            "REFUSED: --execute-once requires an explicit positive --max-dis in mm. "
            "CONTINUOUS JOG: BLOCKED"
        )
    if not math.isfinite(args.max_dis) or args.max_dis <= 0:
        return "REFUSED: --max-dis must be a positive finite distance in mm"
    return None


def main(argv=None):
    args = parse_args(argv)
    if args.precheck and args.execute_once:
        print("Use only one of --precheck or --execute-once", flush=True)
        return 2
    error = execute_configuration_error(args)
    if error:
        print(error, flush=True)
        print("CONTINUOUS JOG: BLOCKED", flush=True)
        print("No motion command sent", flush=True)
        return 2
    if args.precheck:
        return run_precheck()
    lock = None
    if args.test_direction is not None:
        lock = DIRECTION_LOCKS[args.test_direction]
    return run_ros(
        execute=args.execute_once,
        joy_debug=args.joy_debug,
        max_dis_mm=args.max_dis,
        direction_lock=lock,
    )


if __name__ == "__main__":
    sys.exit(main())
