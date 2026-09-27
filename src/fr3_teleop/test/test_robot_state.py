from math import pi

from fr3_teleop.robot_state import FeedbackCache, quaternion_to_rpy


ROBOTS = {
    "arm_a": {"joint_prefix": "arm_a_"},
    "arm_b": {"joint_prefix": "arm_b_"},
}


def test_only_six_measured_arm_joints_are_accepted():
    cache = FeedbackCache(ROBOTS)
    names = [f"arm_a_j{i}" for i in range(1, 7)] + ["arm_a_gripper_joint"]
    cache.update_joints(names, [1., 2., 3., 4., 5., 6., .1], stamp=10.0)
    state = cache.status("arm_a", 1.0)
    assert state["joint_positions_rad"] == {f"arm_a_j{i}": float(i) for i in range(1, 7)}


def test_quaternion_rpy_identity_and_yaw():
    assert quaternion_to_rpy(0., 0., 0., 1.) == (0., 0., 0.)
    _, _, yaw = quaternion_to_rpy(0., 0., 2 ** -0.5, 2 ** -0.5)
    assert abs(yaw - pi / 2) < 1e-12
