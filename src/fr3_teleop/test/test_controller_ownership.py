from fr3_teleop.controller_ownership import ControllerLease, ControllerPair, Owner


def test_manual_is_not_granted_while_auto_goal_is_active():
    lease = ControllerLease({"arm_a": ControllerPair("auto", "manual")}, lambda *_: True, lambda _: True)
    assert not lease.acquire_manual("arm_a")
    assert lease.owner["arm_a"] == Owner.AUTO


def test_failed_switch_never_grants_manual_control():
    lease = ControllerLease({"arm_a": ControllerPair("auto", "manual")}, lambda *_: False, lambda _: False)
    assert not lease.acquire_manual("arm_a")
    assert lease.owner["arm_a"] == Owner.AUTO


def test_switch_and_restore_are_strictly_ordered():
    calls = []
    lease = ControllerLease({"arm_a": ControllerPair("auto", "manual")}, lambda start, stop: calls.append((start, stop)) is None, lambda _: False)
    assert lease.acquire_manual("arm_a")
    assert lease.release_manual("arm_a")
    assert calls == [(["manual"], ["auto"]), (["auto"], ["manual"])]
