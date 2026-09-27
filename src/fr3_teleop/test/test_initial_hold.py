from fr3_teleop.initial_hold import InitialHoldGate


NAMES = [f"arm_a_j{i}" for i in range(1, 7)]
MEASURED = {name: index * 0.1 for index, name in enumerate(NAMES)}


def test_hold_is_measured_not_zero_and_verifies_within_limit():
    gate = InitialHoldGate(NAMES, 0.02, 0.2)
    msg = gate.build_hold(MEASURED)
    assert list(msg.data) == [MEASURED[name] for name in NAMES]
    assert gate.verify_measured(MEASURED)


def test_hold_rejects_stale_or_wrong_desired_position():
    gate = InitialHoldGate(NAMES, 0.02, 0.2); gate.build_hold(MEASURED)
    assert not gate.verify_measured({name: 0.0 for name in NAMES})


def test_hold_rejects_missing_or_non_finite_feedback():
    gate = InitialHoldGate(NAMES, 0.02, 0.2); gate.build_hold(MEASURED)
    assert not gate.verify_measured({name: value for name, value in MEASURED.items() if name != "arm_a_j6"})
    non_finite = dict(MEASURED); non_finite["arm_a_j3"] = float("nan")
    assert not gate.verify_measured(non_finite)
