import pytest

from fr3_teleop.servo_position_accumulator import ServoVelocityIntegrator, VelocityIntegratorConfig


def make_integrator():
    return ServoVelocityIntegrator(VelocityIntegratorConfig(0.02, 3.0, 0.12, 0.02))


def test_reset_uses_current_measured_position_and_zero_velocity():
    integrator = make_integrator(); actual = [0.1 * index for index in range(6)]
    assert integrator.reset_to_measured(actual, 1.0) == pytest.approx(actual)
    assert integrator.target == pytest.approx(actual)
    assert integrator.published == pytest.approx(actual)
    assert integrator.velocity == pytest.approx([0.0] * 6)


def test_constant_velocity_integrates_actual_timer_dt_without_position_difference():
    integrator = make_integrator(); actual = [0.0] * 6; integrator.reset_to_measured(actual, 0.0)
    integrator.begin_motion(0.0); assert integrator.accept_velocity([0.1] * 6, 0.0) is None
    assert integrator.tick(actual, 0.008, True) == pytest.approx([0.0008] * 6)
    assert integrator.tick(actual, 0.015, True) == pytest.approx([0.0015] * 6)
    assert integrator.tick(actual, 0.024, True) == pytest.approx([0.0024] * 6)
    assert integrator.last_lead_scale == pytest.approx(1.0)


def test_large_timer_gap_is_safe_fixed_hold_not_position_jump():
    integrator = make_integrator(); actual = [1.0] * 6; integrator.reset_to_measured(actual, 0.0)
    integrator.begin_motion(0.0); integrator.accept_velocity([0.5] * 6, 0.0)
    assert integrator.tick(actual, 0.1, True) == pytest.approx(actual)
    assert integrator.last_event == "timer-dt-out-of-range"
    assert integrator.fault_latched
    # A held button must not make tick() silently restart after a timer fault.
    assert integrator.tick([1.001] * 6, 0.108, True) == pytest.approx(actual)


def test_lead_is_bounded_against_current_feedback_during_motion():
    integrator = make_integrator(); actual = [0.0] * 6; integrator.reset_to_measured(actual, 0.0)
    integrator.begin_motion(0.0); integrator.accept_velocity([1.0] * 6, 0.0)
    for index in range(1, 30):
        now = index * 0.008
        if index % 5 == 0: integrator.accept_velocity([1.0] * 6, now)
        integrator.tick(actual, now, True)
    assert integrator.target == pytest.approx([0.02] * 6)


def test_lead_limit_scales_all_six_joint_increments_by_one_alpha():
    integrator = make_integrator(); actual = [0.0] * 6; integrator.reset_to_measured(actual, 0.0)
    integrator.begin_motion(0.0)
    # Place only joint 1 close to the positive lead bound.
    integrator.accept_velocity([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0)
    assert integrator.tick(actual, 0.019, True) == pytest.approx([0.019, 0.0, 0.0, 0.0, 0.0, 0.0])
    qdot = [1.0, 0.5, 0.25, -0.5, 0.1, -0.2]
    integrator.accept_velocity(qdot, 0.019)
    before = integrator.target
    after = integrator.tick(actual, 0.027, True)
    increments = [new - old for new, old in zip(after, before)]
    expected_alpha = 0.001 / 0.008
    assert integrator.last_lead_scale == pytest.approx(expected_alpha)
    assert increments == pytest.approx([expected_alpha * velocity * 0.008 for velocity in qdot])
    assert all(abs(target - measured) <= 0.02 + 1e-12 for target, measured in zip(after, actual))
    # All nonzero joints preserve the original qdot proportions.
    assert increments[1] / increments[0] == pytest.approx(qdot[1] / qdot[0])
    assert increments[3] / increments[0] == pytest.approx(qdot[3] / qdot[0])


def test_invalid_velocity_and_timeout_stop_integration():
    integrator = make_integrator(); actual = [0.5] * 6; integrator.reset_to_measured(actual, 0.0); integrator.begin_motion(0.0)
    assert "finite" in integrator.accept_velocity([float("nan")] * 6, 0.0)
    assert "finite" in integrator.accept_velocity([float("inf")] * 6, 0.0)
    assert integrator.accept_velocity([0.0] * 5, 0.0) == "expected exactly six joint velocities"
    assert "exceeds" in integrator.accept_velocity([4.0] * 6, 0.0)
    assert integrator.tick(actual, 0.13, True) == pytest.approx(actual)
    assert integrator.last_event == "raw-velocity-timeout"
    assert integrator.fault_latched
    assert integrator.tick([0.501] * 6, 0.138, True) == pytest.approx(actual)


def test_explicit_new_motion_start_is_required_to_clear_fault_latch():
    integrator = make_integrator(); actual = [0.0] * 6; integrator.reset_to_measured(actual, 0.0)
    integrator.begin_motion(0.0)
    assert integrator.tick(actual, 0.13, True) == pytest.approx(actual)
    assert integrator.fault_latched
    # begin_motion is intentionally insufficient; only the manager-facing
    # new-motion API may re-arm the integrator.
    assert not integrator.begin_motion(0.14)
    assert integrator.start_new_motion(0.14)
    assert not integrator.fault_latched
    assert integrator.accept_velocity([0.1] * 6, 0.14) is None
    assert integrator.tick(actual, 0.148, True) == pytest.approx([0.0008] * 6)


def test_uninitialized_fixed_hold_captures_measured_position_never_zero():
    integrator = make_integrator()
    measured = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert integrator.enter_fixed_hold(measured, 1.0, "shutdown") == pytest.approx(measured)
    assert integrator.target == pytest.approx(measured)
    assert integrator.stop_hold == pytest.approx(measured)


def test_stop_captures_feedback_once_and_never_tracks_later_noise():
    integrator = make_integrator(); integrator.reset_to_measured([0.0] * 6, 0.0); integrator.begin_motion(0.0)
    integrator.accept_velocity([0.1] * 6, 0.0); integrator.tick([0.0] * 6, 0.008, True)
    assert integrator.enter_fixed_hold([1.0] * 6, 0.01, "stop") == pytest.approx([1.0] * 6)
    assert integrator.tick([1.001] * 6, 0.018, False) == pytest.approx([1.0] * 6)
    assert integrator.tick([0.999] * 6, 0.026, False) == pytest.approx([1.0] * 6)
    assert integrator.tick([1.002] * 6, 0.034, False) == pytest.approx([1.0] * 6)


def test_stop_ignores_later_velocity_until_new_motion_starts():
    integrator = make_integrator(); actual = [0.0] * 6; integrator.reset_to_measured(actual, 0.0)
    integrator.begin_motion(0.0); integrator.accept_velocity([0.1] * 6, 0.0); integrator.enter_fixed_hold(actual, 0.01, "stop")
    assert integrator.accept_velocity([0.1] * 6, 0.02) is None
    assert integrator.tick(actual, 0.03, False) == pytest.approx(actual)
    integrator.begin_motion(0.04); integrator.accept_velocity([0.1] * 6, 0.04)
    assert integrator.tick(actual, 0.048, True) == pytest.approx([0.0008] * 6)


def test_arm_integrators_have_no_shared_target_or_stop_hold_state():
    arm_a, arm_b = make_integrator(), make_integrator()
    arm_a.reset_to_measured([0.0] * 6, 0.0)
    arm_b.reset_to_measured([1.0] * 6, 0.0)
    arm_a.begin_motion(0.0)
    arm_a.accept_velocity([0.1] * 6, 0.0)
    arm_a.tick([0.0] * 6, 0.008, True)
    arm_a.enter_fixed_hold([0.5] * 6, 0.01, "stop")
    assert arm_a.stop_hold == pytest.approx([0.5] * 6)
    assert arm_b.stop_hold == pytest.approx([1.0] * 6)
