from fr3_teleop.initial_hold import InitialHoldGate


NAMES = [f"arm_a_j{i}" for i in range(1, 7)]
MEASURED = {name: index * 0.1 for index, name in enumerate(NAMES)}


def test_hold_is_measured_not_zero_and_verifies_within_limit():
    gate = InitialHoldGate(NAMES, 0.02, 0.2)
    msg = gate.build_hold(MEASURED)
    assert list(msg.points[0].positions) == [MEASURED[name] for name in NAMES]
    assert gate.verify_desired(NAMES, msg.points[0].positions)


def test_hold_rejects_stale_or_wrong_desired_position():
    gate = InitialHoldGate(NAMES, 0.02, 0.2); gate.build_hold(MEASURED)
    assert not gate.verify_desired(NAMES, [0.0] * 6)
