"""State-machine tests for six-direction joystick jog. No robot connection."""

from joy_control_test.joystick_jog_test import (
    DIRECTION_LOCKS,
    JOY_TIMEOUT_S,
    SdkWorker,
    SimAdapter,
    State,
    JogController,
    blocking_arm_sockets,
    execute_configuration_error,
    parse_args,
    start_jog_call,
    webapp_view_socket,
)

TEST_MAX_DIS_MM = 3.0


def pad(ax0=0.0, ax1=0.0, ax3=0.0, ax4=0.0, rb=0, lb=0, back=0, a=0, b=0):
    axes = [ax0, ax1, 0.0, ax3, ax4, 0.0]
    buttons = [a, b, 0, 0, lb, rb, back, 0]
    return axes, buttons


def machine(adapter=None, now=None, direction_lock=None, rearm=True):
    clock_now = {"t": 0.0 if now is None else now}
    controller = JogController(
        rearm_after_normal_stop=rearm,
        direction_lock=direction_lock,
    )
    worker = SdkWorker(
        controller,
        adapter or SimAdapter(max_dis_mm=TEST_MAX_DIS_MM),
        clock=lambda: clock_now["t"],
    )
    return controller, worker, clock_now


def send(controller, worker, now, **joy):
    clock = joy.pop("clock", None)
    if clock is not None:
        clock["t"] = now
    controller.on_joy(*pad(**joy), now)
    worker.process_once()


def test_no_rb_cannot_start():
    controller, worker, clock = machine()
    send(controller, worker, 0.0, clock=clock)
    send(controller, worker, 0.1, clock=clock, ax0=-1.0, rb=0)
    assert controller.start_intents == 0
    assert worker.adapter.start_calls == 0
    assert controller.state == State.READY


def test_right_stick_starts_once():
    controller, worker, clock = machine()
    send(controller, worker, 0.0, clock=clock)
    send(controller, worker, 0.1, clock=clock, ax0=-1.0, rb=1)
    send(controller, worker, 0.2, clock=clock, ax0=-1.0, rb=1)
    send(controller, worker, 0.3, clock=clock, ax0=-1.0, rb=1)
    assert controller.start_intents == 1
    assert worker.adapter.start_calls == 1
    assert controller.state == State.JOGGING
    assert controller.events.count("START") == 1


def test_stick_center_requests_stop():
    controller, worker, clock = machine()
    send(controller, worker, 0.0, clock=clock)
    send(controller, worker, 0.1, clock=clock, ax0=-1.0, rb=1)
    send(controller, worker, 0.2, clock=clock, ax0=-0.10, rb=1)
    assert controller.stop_reason == "stick_center"
    assert "STOP_REQUESTED" in controller.events
    assert worker.adapter.stop_calls == 1
    assert worker.adapter.start_calls == 1
    assert controller.state == State.READY


def test_rb_release_requests_stop():
    controller, worker, clock = machine()
    send(controller, worker, 0.0, clock=clock)
    send(controller, worker, 0.1, clock=clock, ax0=-1.0, rb=1)
    send(controller, worker, 0.2, clock=clock, ax0=-1.0, rb=0)
    assert controller.stop_reason == "rb_released"
    assert worker.adapter.stop_calls == 1
    assert worker.adapter.start_calls == 1
    assert controller.state == State.READY


def test_back_locks_out_another_start():
    controller, worker, clock = machine()
    send(controller, worker, 0.0, clock=clock)
    send(controller, worker, 0.1, clock=clock, ax0=-1.0, rb=1)
    send(controller, worker, 0.2, clock=clock, back=1)
    assert controller.stop_reason == "back"
    assert controller.state == State.COMPLETE
    send(controller, worker, 0.3, clock=clock)
    send(controller, worker, 0.4, clock=clock, ax0=-1.0, rb=1)
    assert worker.adapter.start_calls == 1


def test_joy_timeout_locks_and_faults():
    controller, worker, clock = machine()
    send(controller, worker, 0.0, clock=clock)
    send(controller, worker, 0.10, clock=clock, ax0=-1.0, rb=1)
    controller.poll_safety(0.10 + JOY_TIMEOUT_S + 0.01)
    worker.process_once()
    assert controller.watchdog_fired is True
    assert controller.stop_reason == "joy_timeout"
    assert controller.state == State.FAULT
    send(controller, worker, 1.0, clock=clock)
    send(controller, worker, 1.1, clock=clock, ax0=-1.0, rb=1)
    assert worker.adapter.start_calls == 1


def test_illegal_input_cannot_start():
    controller, worker, clock = machine()
    send(controller, worker, 0.0, clock=clock)
    send(controller, worker, 0.1, clock=clock, ax0=-1.0, ax1=1.0, rb=1)
    send(controller, worker, 0.2, clock=clock, ax0=-1.0, ax3=-1.0, rb=1)
    send(controller, worker, 0.3, clock=clock, ax0=-1.0, rb=1, lb=1)
    send(controller, worker, 0.4, clock=clock, ax0=-1.0, rb=1, a=1)
    send(controller, worker, 0.5, clock=clock, ax0=-1.0, rb=1, b=1)
    assert controller.start_intents == 0
    assert worker.adapter.start_calls == 0
    assert controller.state == State.READY


def test_repeated_joy_does_not_repeat_startjog():
    controller, worker, clock = machine()
    send(controller, worker, 0.0, clock=clock)
    for index in range(6):
        send(controller, worker, 0.1 + index * 0.02, clock=clock, ax0=-1.0, rb=1)
    assert worker.adapter.start_calls == 1


def test_stopjog_error_does_not_retry_motion():
    adapter = SimAdapter()
    adapter.stop_ret = 7
    controller, worker, clock = machine(adapter)
    send(controller, worker, 0.0, clock=clock)
    send(controller, worker, 0.1, clock=clock, ax0=-1.0, rb=1)
    send(controller, worker, 0.2, clock=clock, ax0=0.0, rb=0)
    assert adapter.start_calls == 1
    assert adapter.stop_calls == 1
    assert adapter.imm_calls == 1
    assert controller.state == State.FAULT
    send(controller, worker, 0.4, clock=clock)
    send(controller, worker, 0.5, clock=clock, ax0=-1.0, rb=1)
    assert adapter.start_calls == 1
    assert adapter.imm_calls == 1


def test_normal_release_can_start_again_and_fault_cannot():
    controller, worker, clock = machine()
    send(controller, worker, 0.0, clock=clock)
    send(controller, worker, 0.1, clock=clock, ax0=-1.0, rb=1)
    send(controller, worker, 0.2, clock=clock, ax0=0.0, rb=0)
    assert controller.state == State.READY
    send(controller, worker, 0.3, clock=clock, ax4=1.0, rb=1)
    assert worker.adapter.start_calls == 2
    assert worker.adapter.last_jog.nb == 3
    assert worker.adapter.last_jog.direction == 1

    failed = SimAdapter()
    failed.stop_ret = 3
    faulted, fault_worker, fault_clock = machine(failed)
    send(faulted, fault_worker, 0.0, clock=fault_clock)
    send(faulted, fault_worker, 0.1, clock=fault_clock, ax0=-1.0, rb=1)
    send(faulted, fault_worker, 0.2, clock=fault_clock, ax0=0.0, rb=1)
    assert faulted.state == State.FAULT
    send(faulted, fault_worker, 0.3, clock=fault_clock)
    send(faulted, fault_worker, 0.4, clock=fault_clock, ax0=-1.0, rb=1)
    assert failed.start_calls == 1


def test_startup_hold_cannot_start_until_neutral():
    controller, worker, clock = machine()
    send(controller, worker, 0.0, clock=clock, ax0=-1.0, rb=1)
    assert controller.neutral_seen is False
    assert worker.adapter.start_calls == 0
    send(controller, worker, 0.2, clock=clock, ax0=-1.0, rb=1)
    assert worker.adapter.start_calls == 0
    send(controller, worker, 0.4, clock=clock)
    assert controller.neutral_seen is True
    send(controller, worker, 0.6, clock=clock, ax0=-1.0, rb=1)
    assert worker.adapter.start_calls == 1
    assert controller.state == State.JOGGING


def test_stop_preempts_start_that_was_not_sent():
    controller, worker, clock = machine()
    controller.on_joy(*pad(), 0.0)
    controller.on_joy(*pad(ax0=-1.0, rb=1), 0.1)
    controller.on_joy(*pad(ax0=0.0, rb=1), 0.15)
    worker.process_once()
    assert worker.adapter.start_calls == 0
    assert worker.adapter.stop_calls == 0
    assert controller.state == State.READY


def test_hysteresis_keeps_jogging_until_stop_threshold():
    controller, worker, clock = machine()
    send(controller, worker, 0.0, clock=clock)
    send(controller, worker, 0.1, clock=clock, ax0=-1.0, rb=1)
    send(controller, worker, 0.2, clock=clock, ax0=-0.25, rb=1)
    assert controller.state == State.JOGGING
    assert worker.adapter.stop_calls == 0
    send(controller, worker, 0.3, clock=clock, ax0=-0.20, rb=1)
    assert controller.stop_reason == "stick_center"
    assert controller.state == State.READY


def test_fresh_hold_is_not_cut_off_at_half_second():
    controller, worker, clock = machine()
    send(controller, worker, 1.0, clock=clock)
    send(controller, worker, 1.1, clock=clock, ax0=-1.0, rb=1)
    assert controller.motion_anchor == 1.1
    held_until = 1.1 + 0.50
    controller.on_joy(*pad(ax0=-1.0, rb=1), held_until)
    controller.poll_safety(held_until)
    worker.process_once()
    assert controller.time_limit_hit is False
    assert controller.stop_reason == ""
    assert worker.adapter.stop_calls == 0
    assert worker.adapter.start_calls == 1
    assert controller.state == State.JOGGING


def send_raw(controller, worker, axes, buttons, now, clock):
    clock["t"] = now
    controller.on_joy(axes, buttons, now)
    worker.process_once()


def begin_jog(clock=None):
    controller, worker, clock = machine(now=0.0 if clock is None else None)
    send(controller, worker, 0.0, clock=clock)
    send(controller, worker, 0.1, clock=clock, ax0=-1.0, rb=1)
    assert controller.state == State.JOGGING
    assert worker.adapter.start_calls == 1
    assert controller.last_unsupported is None
    return controller, worker, clock


def test_legal_hold_does_not_raise_unsupported():
    controller, worker, clock = begin_jog()
    send(controller, worker, 0.2, clock=clock, ax0=-1.0, rb=1)
    send(controller, worker, 0.3, clock=clock, ax0=-1.0, rb=1)
    assert controller.state == State.JOGGING
    assert controller.stop_reason == ""
    assert controller.last_unsupported is None
    assert worker.adapter.start_calls == 1
    assert worker.adapter.stop_calls == 0


def test_center_and_rb_release_are_not_unsupported():
    centered, center_worker, center_clock = begin_jog()
    send(centered, center_worker, 0.2, clock=center_clock, ax0=0.0, rb=1)
    assert centered.stop_reason == "stick_center"
    assert centered.last_unsupported is None

    released, release_worker, release_clock = begin_jog()
    send(released, release_worker, 0.2, clock=release_clock, ax0=-1.0, rb=0)
    assert released.stop_reason == "rb_released"
    assert released.last_unsupported is None


def test_lb_records_button_violation():
    controller, worker, clock = begin_jog()
    send(controller, worker, 0.2, clock=clock, ax0=-1.0, rb=1, lb=1)
    assert controller.stop_reason == "unsupported_button"
    assert controller.state == State.FAULT
    assert controller.last_unsupported["axes"][0] == -1.0
    assert controller.last_unsupported["buttons"][5] == 1
    assert controller.last_unsupported["buttons"][4] == 1
    assert controller.last_unsupported["violations"] == [{
        "kind": "button",
        "button_index": 4,
        "name": "LB",
        "actual_value": 1,
        "expected": 0,
    }]
    assert worker.adapter.start_calls == 1
    assert worker.adapter.stop_calls == 1


def test_a_and_b_record_separate_button_violations():
    for name, joy in (("A", {"a": 1}), ("B", {"b": 1})):
        controller, worker, clock = begin_jog()
        send(controller, worker, 0.2, clock=clock, ax0=-1.0, rb=1, **joy)
        assert controller.stop_reason == "unsupported_button"
        assert controller.last_unsupported["stop_reason"] == "unsupported_button"
        assert controller.last_unsupported["violations"][0]["name"] == name
        assert worker.adapter.start_calls == 1


def test_command_axis_conflict_is_not_labeled_unsupported():
    cases = ({"ax1": -1.0}, {"ax4": 1.0})
    for joy in cases:
        controller, worker, clock = begin_jog()
        send(controller, worker, 0.2, clock=clock, ax0=-1.0, rb=1, **joy)
        assert controller.stop_reason == "axis_conflict"
        assert controller.last_unsupported is None
        assert worker.adapter.start_calls == 1
        assert worker.adapter.stop_calls == 1
        assert controller.state == State.READY


def test_right_stick_x_records_unsupported_axis():
    controller, worker, clock = begin_jog()
    send(controller, worker, 0.2, clock=clock, ax0=-1.0, ax3=-1.0, rb=1)
    violation = controller.last_unsupported["violations"][0]
    assert violation["axis_index"] == 3
    assert violation["name"] == "right_x"
    assert violation["actual_value"] == -1.0
    assert violation["threshold"] == 0.20
    assert controller.stop_reason == "unsupported_axis"
    assert controller.last_unsupported["stop_reason"] == "unsupported_axis"
    assert worker.adapter.start_calls == 1
    assert worker.adapter.stop_calls == 1
    assert controller.state == State.FAULT


def test_unchecked_axes_at_rest_extremes_do_not_stop():
    controller, worker, clock = begin_jog()
    axes = [-1.0, 0.0, -1.0, 0.0, 0.0, 1.0]
    buttons = [0, 0, 0, 0, 0, 1, 0, 0]
    send_raw(controller, worker, axes, buttons, 0.2, clock)
    assert controller.state == State.JOGGING
    assert controller.last_unsupported is None
    assert worker.adapter.stop_calls == 0
    send_raw(controller, worker, [-1.0, 0.0, float("nan"), 0.0, 0.0, 1.0], buttons, 0.3, clock)
    assert controller.state == State.JOGGING
    assert worker.adapter.start_calls == 1


def test_short_or_nonfinite_joy_is_not_labeled_unsupported():
    short, short_worker, short_clock = begin_jog()
    send_raw(short, short_worker, [-1.0, 0.0], [0, 0, 0, 0, 0, 1], 0.2, short_clock)
    assert short.stop_reason == "incomplete_joy"
    assert short.last_unsupported is None

    bad, bad_worker, bad_clock = begin_jog()
    send_raw(
        bad,
        bad_worker,
        [-1.0, float("nan"), 0.0, 0.0, 0.0],
        [0, 0, 0, 0, 0, 1, 0],
        0.2,
        bad_clock,
    )
    assert bad.stop_reason == "invalid_joy"
    assert bad.last_unsupported is None
    assert bad_worker.adapter.start_calls == 1


DIRECTIONS = (
    ("X+", {"ax0": -1.0}, 1, 1, "StartJOG(2, 1, 1, 3.0, vel=5.0, acc=20.0)"),
    ("X-", {"ax0": 1.0}, 1, 0, "StartJOG(2, 1, 0, 3.0, vel=5.0, acc=20.0)"),
    ("Y+", {"ax1": 1.0}, 2, 1, "StartJOG(2, 2, 1, 3.0, vel=5.0, acc=20.0)"),
    ("Y-", {"ax1": -1.0}, 2, 0, "StartJOG(2, 2, 0, 3.0, vel=5.0, acc=20.0)"),
    ("Z+", {"ax4": 1.0}, 3, 1, "StartJOG(2, 3, 1, 3.0, vel=5.0, acc=20.0)"),
    ("Z-", {"ax4": -1.0}, 3, 0, "StartJOG(2, 3, 0, 3.0, vel=5.0, acc=20.0)"),
)


def test_each_direction_starts_once_with_sdk_parameters():
    for name, joy, nb, direction, call in DIRECTIONS:
        controller, worker, clock = machine()
        send(controller, worker, 0.0, clock=clock)
        send(controller, worker, 0.1, clock=clock, rb=1, **joy)
        send(controller, worker, 0.2, clock=clock, rb=1, **joy)
        assert controller.state == State.JOGGING, name
        assert worker.adapter.start_calls == 1, name
        assert worker.adapter.last_jog.name == name
        assert worker.adapter.last_jog.nb == nb
        assert worker.adapter.last_jog.direction == direction
        assert worker.adapter.last_call == call
        send(controller, worker, 0.3, clock=clock, rb=1)
        assert controller.stop_reason == "stick_center", name
        assert worker.adapter.stop_calls == 1, name
        assert controller.state == State.READY, name
        released, release_worker, release_clock = machine()
        send(released, release_worker, 0.0, clock=release_clock)
        send(released, release_worker, 0.1, clock=release_clock, rb=1, **joy)
        send(released, release_worker, 0.2, clock=release_clock, rb=0, **joy)
        assert released.stop_reason == "rb_released", name
        assert release_worker.adapter.start_calls == 1, name
        assert release_worker.adapter.stop_calls == 1, name


def test_direction_change_does_not_start_before_stop():
    controller, worker, clock = machine()
    send(controller, worker, 0.0, clock=clock)
    send(controller, worker, 0.1, clock=clock, ax0=-1.0, rb=1)
    send(controller, worker, 0.2, clock=clock, ax1=1.0, rb=1)
    assert worker.adapter.start_calls == 1
    assert worker.adapter.last_jog.name == "X+"
    assert worker.adapter.stop_calls == 1
    assert controller.stop_reason == "stick_center"
    assert controller.state == State.READY
    send(controller, worker, 0.3, clock=clock, ax1=1.0, rb=1)
    assert worker.adapter.start_calls == 2
    assert worker.adapter.last_jog.nb == 2
    assert worker.adapter.last_jog.direction == 1
    stop_at = controller.events.index("STOP_REQUESTED")
    second_start = controller.events.index("START", stop_at)
    assert stop_at < second_start


def test_opposite_direction_waits_for_stop():
    controller, worker, clock = machine()
    send(controller, worker, 0.0, clock=clock)
    send(controller, worker, 0.1, clock=clock, ax0=-1.0, rb=1)
    send(controller, worker, 0.2, clock=clock, ax0=1.0, rb=1)
    assert worker.adapter.start_calls == 1
    assert worker.adapter.last_jog.direction == 1
    assert worker.adapter.stop_calls == 1
    send(controller, worker, 0.3, clock=clock, ax0=1.0, rb=1)
    assert worker.adapter.start_calls == 2
    assert worker.adapter.last_jog.direction == 0
    assert worker.adapter.last_call == "StartJOG(2, 1, 0, 3.0, vel=5.0, acc=20.0)"


def test_multi_axis_conflict_stops_and_does_not_pick_one():
    idle, idle_worker, idle_clock = machine()
    send(idle, idle_worker, 0.0, clock=idle_clock)
    send(idle, idle_worker, 0.1, clock=idle_clock, ax0=-1.0, ax1=1.0, ax4=-1.0, rb=1)
    assert idle_worker.adapter.start_calls == 0
    assert idle.state == State.READY

    controller, worker, clock = machine()
    send(controller, worker, 0.0, clock=clock)
    send(controller, worker, 0.1, clock=clock, ax0=-1.0, rb=1)
    send(controller, worker, 0.2, clock=clock, ax0=-1.0, ax1=1.0, rb=1)
    assert controller.stop_reason == "axis_conflict"
    assert controller.last_unsupported is None
    assert worker.adapter.start_calls == 1
    assert worker.adapter.stop_calls == 1
    send(controller, worker, 0.3, clock=clock, ax0=-1.0, ax4=1.0, rb=1)
    assert worker.adapter.start_calls == 1
    assert controller.state == State.READY


def test_back_and_watchdog_still_lock():
    controller, worker, clock = machine()
    send(controller, worker, 0.0, clock=clock)
    send(controller, worker, 0.1, clock=clock, ax4=-1.0, rb=1)
    send(controller, worker, 0.2, clock=clock, back=1)
    assert controller.stop_reason == "back"
    assert controller.state == State.COMPLETE
    assert worker.adapter.start_calls == 1
    send(controller, worker, 0.4, clock=clock, ax4=-1.0, rb=1)
    assert worker.adapter.start_calls == 1

    timed, timed_worker, timed_clock = machine()
    send(timed, timed_worker, 0.0, clock=timed_clock)
    send(timed, timed_worker, 0.1, clock=timed_clock, ax1=-1.0, rb=1)
    timed.poll_safety(0.1 + JOY_TIMEOUT_S + 0.01)
    timed_worker.process_once()
    assert timed.stop_reason == "joy_timeout"
    assert timed.state == State.FAULT
    assert timed.watchdog_fired is True
    assert timed_worker.adapter.stop_calls == 1
    send(timed, timed_worker, 1.0, clock=timed_clock, ax1=-1.0, rb=1)
    assert timed_worker.adapter.start_calls == 1


def _joy_lines(text):
    return [line for line in text.splitlines() if " JOY " in line]


def test_centered_unchanged_joy_prints_once(capsys):
    controller = JogController()
    for index in range(40):
        controller.on_joy(*pad(), index * 0.25)
    assert len(_joy_lines(capsys.readouterr().out)) == 1


def test_held_axis_joy_prints_at_most_once_per_second(capsys):
    controller = JogController()
    for index in range(61):
        controller.on_joy(*pad(ax0=-1.0, rb=1), index * 0.05)
    lines = _joy_lines(capsys.readouterr().out)
    assert len(lines) == 4


def test_joy_debug_prints_every_sample(capsys):
    controller = JogController(joy_debug=True)
    for now in (0.0, 0.01, 0.02, 0.03):
        controller.on_joy(*pad(), now)
    assert len(_joy_lines(capsys.readouterr().out)) == 4


def test_motion_logs_stay_immediate(capsys):
    controller, worker, clock = machine()
    send(controller, worker, 0.0, clock=clock)
    send(controller, worker, 0.05, clock=clock, ax0=-1.0, rb=1)
    send(controller, worker, 0.10, clock=clock, rb=0)
    text = capsys.readouterr().out
    for kind in (
        "STARTJOG_CALLED",
        "STARTJOG_RET",
        "STOP_REQUESTED",
        "STOPJOG_CALLED",
        "STOPJOG_RET",
        "stop_confirm=True",
        "STATE",
    ):
        assert kind in text
    assert len(_joy_lines(text)) == 1


def test_execute_once_does_not_rearm_after_release():
    controller = JogController(rearm_after_normal_stop=False)
    worker = SdkWorker(controller, SimAdapter(), clock=lambda: 0.0)
    controller.on_joy(*pad(), 0.0)
    controller.on_joy(*pad(ax0=-1.0, rb=1), 0.1)
    worker.process_once()
    controller.on_joy(*pad(rb=0), 0.2)
    worker.process_once()
    assert controller.state == State.COMPLETE
    controller.on_joy(*pad(), 0.3)
    controller.on_joy(*pad(ax1=1.0, rb=1), 0.4)
    worker.process_once()
    assert worker.adapter.start_calls == 1


GESTURES = {
    "x_pos": {"ax0": -1.0},
    "x_neg": {"ax0": 1.0},
    "y_pos": {"ax1": 1.0},
    "y_neg": {"ax1": -1.0},
    "z_pos": {"ax4": 1.0},
    "z_neg": {"ax4": -1.0},
}


def test_direction_lock_accepts_only_the_selected_axis():
    for locked_name, joy in GESTURES.items():
        lock = DIRECTION_LOCKS[locked_name]
        controller, worker, clock = machine(direction_lock=lock, rearm=False)
        send(controller, worker, 0.0, clock=clock)
        stamp = 0.1
        for other_name, other_joy in GESTURES.items():
            if other_name == locked_name:
                continue
            send(controller, worker, stamp, clock=clock, rb=1, **other_joy)
            stamp += 0.1
            assert worker.adapter.start_calls == 0, locked_name
        send(controller, worker, stamp, clock=clock, rb=1, **joy)
        assert worker.adapter.start_calls == 1, locked_name
        assert worker.adapter.last_jog.same_as(lock)
        assert worker.adapter.last_call == start_jog_call(lock, TEST_MAX_DIS_MM)
        send(controller, worker, stamp + 0.1, clock=clock, rb=1)
        assert controller.state == State.COMPLETE, locked_name
        send(controller, worker, stamp + 0.2, clock=clock, rb=1, **joy)
        assert worker.adapter.start_calls == 1, locked_name


def test_unconfigured_distance_is_not_replaced():
    adapter = SimAdapter()
    controller = JogController()
    worker = SdkWorker(controller, adapter, clock=lambda: 0.0)
    controller.on_joy(*pad(), 0.0)
    controller.on_joy(*pad(ax0=-1.0, rb=1), 0.1)
    worker.process_once()
    assert adapter.start_calls == 1
    assert "10.0" not in adapter.last_call
    assert "UNCONFIGURED" in adapter.last_call


def test_all_six_directions_can_run_in_one_session():
    controller, worker, clock = machine(rearm=True)
    send(controller, worker, 0.0, clock=clock)
    stamp = 0.1
    for name, joy in GESTURES.items():
        send(controller, worker, stamp, clock=clock, rb=1, **joy)
        stamp += 0.1
        assert worker.adapter.last_jog.same_as(DIRECTION_LOCKS[name])
        send(controller, worker, stamp, clock=clock, rb=0)
        stamp += 0.1
        assert controller.state == State.READY
    assert worker.adapter.start_calls == 6
    assert worker.adapter.stop_calls == 6


def test_firefox_webapp_view_does_not_block_motion_sockets():
    webapp_page = (
        'ESTAB 0 0 192.168.58.11:34112 192.168.58.2:9999 '
        'users:(("firefox",pid=20242,fd=198))'
    )
    webapp_http = (
        'ESTAB 0 0 192.168.58.11:50000 192.168.58.2:80 '
        'users:(("firefox",pid=20242,fd=96))'
    )
    sdk = (
        'ESTAB 0 0 192.168.58.11:40000 192.168.58.2:20003 '
        'users:(("python3",pid=9,fd=4))'
    )
    assert webapp_view_socket(webapp_page) is True
    assert webapp_view_socket(webapp_http) is True
    assert webapp_view_socket(sdk) is False
    assert blocking_arm_sockets([webapp_page, webapp_http, sdk]) == [sdk]
    assert blocking_arm_sockets([webapp_page, webapp_http]) == []


def test_execute_once_requires_positive_distance():
    refused = execute_configuration_error(parse_args(["--execute-once"]))
    assert refused is not None
    assert "max-dis" in refused
    missing_distance = execute_configuration_error(parse_args([
        "--execute-once",
        "--test-direction",
        "x_neg",
    ]))
    assert missing_distance is not None
    assert "BLOCKED" in missing_distance
    all_directions = parse_args(["--execute-once", "--max-dis", "4"])
    assert execute_configuration_error(all_directions) is None
    assert all_directions.test_direction is None
    for bad_distance in ("0", "-3", "nan"):
        error = execute_configuration_error(parse_args([
            "--execute-once",
            "--test-direction",
            "y_pos",
            "--max-dis",
            bad_distance,
        ]))
        assert error is not None, bad_distance
    ready = parse_args([
        "--execute-once",
        "--test-direction",
        "z_neg",
        "--max-dis",
        "4",
    ])
    assert execute_configuration_error(ready) is None
    assert execute_configuration_error(parse_args([])) is None
