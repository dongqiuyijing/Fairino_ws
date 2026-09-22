#!/usr/bin/env python3
"""Plan two opposed FR3 gripper TCPs to a 30 mm X-axis separation.

At the shared midpoint TARGET in world coordinates:
  Arm A TCP = TARGET + (GAP/2, 0, 0), finger forward (TCP +Z) = world -X.
  Arm B TCP = TARGET - (GAP/2, 0, 0), finger forward (TCP +Z) = world +X.
  Both TCP +X (the jaw opening/closing direction) = world +Z.

--arm both solves each TCP via collision-aware single-arm IK, validates the
combined 12-joint target, then plans ONE dual_arms joint trajectory. It
executes ONE combined RobotTrajectory through
the existing dual-arm MoveIt controllers, NOT two uncoordinated actions.
30 mm is TCP-to-TCP distance, NOT guaranteed clearance between grippers.
Default: PLAN ONLY. This test never controls the grippers or starts SDK RPC.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
import math
import os
import sys
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK, GetPositionFK, GetStateValidity
import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_ros import TransformException
import yaml

from fr_control.moveit_arm import MoveItArm, MoveItError
from go_world_tcp import WorldTcpGo, distance, fmt, position, read_real_backend


# Edit the shared midpoint here (world coordinates, METRES).
TARGET = (0.0, 0.3, 1.1)
# 30 mm between TCP origins along world X; not the free gap between fingers.
GAP = 0.030
ROOT_HALF = math.sqrt(0.5)
QUATERNION = {
    "a": (0.0, -ROOT_HALF, 0.0, ROOT_HALF),
    "b": (ROOT_HALF, 0.0, ROOT_HALF, 0.0),
}
TCP = {"a": "arm_a_gripper_tcp", "b": "arm_b_gripper_tcp"}
ARM_JOINTS = {arm: tuple(f"arm_{arm}_j{i}" for i in range(1, 7)) for arm in ("a", "b")}


def target_xyz(arm: str) -> tuple[float, float, float]:
    return (
        TARGET[0] + (GAP / 2.0 if arm == "a" else -GAP / 2.0),
        TARGET[1],
        TARGET[2],
    )


def target_pose(arm: str) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = target_xyz(arm)
    q = QUATERNION[arm]
    pose.orientation.x, pose.orientation.y = q[0], q[1]
    pose.orientation.z, pose.orientation.w = q[2], q[3]
    return pose


def column_guard(xyz: tuple[float, float, float]) -> tuple[bool, str]:
    """Check the TCP point against the configured column + 20 mm margin.

    MoveIt must separately collision-check the WHOLE robots and joint paths.
    """
    path = os.path.join(
        get_package_share_directory("fairino3_dual_moveit_config"),
        "config", "dual_workcell.yaml",
    )
    with open(path, encoding="utf-8") as stream:
        cfg = yaml.safe_load(stream)
    col = cfg["column"]
    center = col["initial_pose"]["position"]
    size = col["dimensions"]
    margin = 0.020
    hit = all(
        abs(xyz[i] - float(center[axis])) <=
        float(size[axis]) / 2.0 + margin
        for i, axis in enumerate(("x", "y", "z"))
    )
    return hit, f"column guard at {fmt(xyz)}, config={path}"


def fresh_tcp(node: WorldTcpGo, arm: str, timeout: float = 8.0):
    """Fetch a fresh dynamic TF for either arm, without driving hardware."""
    deadline = time.monotonic() + timeout
    reason = ""
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            transform = node.tf.lookup_transform("world", TCP[arm], Time())
        except TransformException as exc:
            reason = str(exc)
            continue
        stamp = Time.from_msg(transform.header.stamp).nanoseconds
        age = (node.get_clock().now().nanoseconds - stamp) / 1e9
        if stamp and -0.5 <= age <= node.max_tf_age:
            return transform
        reason = f"stale TCP TF: {age:.3f}s"
    raise RuntimeError(f"No fresh world <- {TCP[arm]} TF: {reason}")


def quaternion_delta_deg(t1, t2) -> float:
    a = t1.transform.rotation
    b = t2.transform.rotation
    dot = abs(a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w)
    return math.degrees(2.0 * math.acos(max(0.0, min(1.0, dot))))


def await_service(node, srv_type, name):
    """Only connect to existing MoveIt, never open a second FAIRINO RPC."""
    client = node.create_client(srv_type, name)
    if not client.wait_for_service(timeout_sec=3.0):
        raise RuntimeError(
            f"MoveIt service {name} unavailable. Check dual_bringup; "
            "no motion sent."
        )
    return client


def service_call(node, client, request, label, timeout=10.0):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        client.remove_pending_request(future)
        raise RuntimeError(f"{label} timeout; no motion sent")
    result = future.result()
    if result is None:
        raise RuntimeError(f"{label} response missing: {future.exception()}")
    return result


def current_robot_state(node, timeout=5.0):
    """Require fresh joint state with both arms; preserve gripper joints."""
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        msg = node._both_joint_msg
        stamp = node._both_joint_arrived
        if msg is None or stamp is None:
            continue
        if time.monotonic() - stamp > node.max_tf_age:
            continue
        names = dict(zip(msg.name, msg.position))
        if all(set(ARM_JOINTS[a]).issubset(names) for a in ("a", "b")):
            if not all(
                math.isfinite(names[name]) for arm in ("a", "b")
                for name in ARM_JOINTS[arm]
            ):
                raise RuntimeError("Nonfinite joint state; refusing")
            state = RobotState()
            state.joint_state = deepcopy(msg)
            state.is_diff = False
            return state
    raise RuntimeError("Missing fresh /joint_states for BOTH arms")


def merge_joint_positions(state, values):
    """Only change named joints, retaining the other arm and fingers."""
    out = deepcopy(state)
    names = list(out.joint_state.name)
    positions = list(out.joint_state.position)
    for joint, value in values.items():
        if joint not in names or not math.isfinite(value):
            raise RuntimeError(f"Missing/invalid joint {joint}")
        positions[names.index(joint)] = float(value)
    out.joint_state.position = positions
    return out


def solve_arm_ik(node, client, arm, seed):
    req = GetPositionIK.Request()
    req.ik_request.group_name = f"arm_{arm}"
    req.ik_request.ik_link_name = TCP[arm]
    req.ik_request.robot_state = seed
    req.ik_request.avoid_collisions = True
    req.ik_request.timeout = Duration(seconds=3.0).to_msg()
    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose = target_pose(arm)
    req.ik_request.pose_stamped = pose
    result = service_call(node, client, req, f"Arm {arm.upper()} IK")
    if result.error_code.val != MoveItErrorCodes.SUCCESS:
        raise RuntimeError(
            f"Arm {arm.upper()} IK failed code {result.error_code.val}; "
            "check pose and collision scene"
        )
    names = result.solution.joint_state.name
    angles = result.solution.joint_state.position
    if len(names) != len(angles) or len(names) != len(set(names)):
        raise RuntimeError("IK returned inconsistent joint names/positions")
    data = dict(zip(names, angles))
    if not set(ARM_JOINTS[arm]).issubset(data):
        raise RuntimeError(f"Arm {arm.upper()} IK missing joints")
    return merge_joint_positions(
        seed, {name: data[name] for name in ARM_JOINTS[arm]}
    )


def require_valid_state(node, client, state, label):
    req = GetStateValidity.Request()
    req.robot_state = state
    # Empty group checks COMPLETE robot, including gripper fingers outside arm chains.
    req.group_name = ""
    result = service_call(node, client, req, label, timeout=8.0)
    if not result.valid:
        contacts = [
            f"{c.contact_body_1} <-> {c.contact_body_2}"
            for c in result.contacts[:8]
        ]
        raise RuntimeError(
            f"{label}: full robot state invalid (collision/limits). "
            f"Contacts={contacts}. Do NOT disable collision checks."
        )


def check_target_fk(node, client, state, arms, pos_tol):
    """Check BOTH exact TCP poses from ONE combined joint state."""
    req = GetPositionFK.Request()
    req.header.frame_id = "world"
    req.header.stamp = node.get_clock().now().to_msg()
    req.fk_link_names = [TCP[arm] for arm in arms]
    req.robot_state = state
    result = service_call(node, client, req, "goal FK")
    if result.error_code.val != MoveItErrorCodes.SUCCESS:
        raise RuntimeError(f"FK failed code {result.error_code.val}")
    poses = dict(zip(result.fk_link_names, result.pose_stamped))
    for arm in arms:
        if TCP[arm] not in poses:
            raise RuntimeError(f"FK missing {TCP[arm]}")
        p = poses[TCP[arm]].pose
        xyz = (p.position.x, p.position.y, p.position.z)
        error_mm = distance(xyz, target_xyz(arm)) * 1000.0
        o = p.orientation
        q = QUATERNION[arm]
        dot = abs(o.x*q[0] + o.y*q[1] + o.z*q[2] + o.w*q[3])
        error_deg = math.degrees(
            2.0 * math.acos(max(0.0, min(1.0, dot)))
        )
        print(
            f"Arm {arm.upper()} FK goal error: "
            f"{error_mm:.3f} mm, {error_deg:.3f} deg"
        )
        if error_mm > pos_tol * 1000.0 or error_deg > 3.0:
            raise RuntimeError(
                f"Arm {arm.upper()} combined goal FK outside tolerance"
            )


def check_waypoints(node, client, state, trajectory):
    """Full-robot validity for every returned waypoint (other arm retained).

    MoveIt checks interpolated swept-path collisions during planning; this
    additional check does NOT substitute for real-world calibration.
    """
    jt = trajectory.joint_trajectory
    if len(jt.points) > 1500:
        raise RuntimeError("Too many waypoints to validate; no motion")
    print(f"Validating {len(jt.points)} full-robot joint waypoints ...")
    for i, point in enumerate(jt.points):
        if len(point.positions) != len(jt.joint_names):
            raise RuntimeError(f"Waypoint {i} joint count mismatch")
        require_valid_state(
            node, client,
            merge_joint_positions(
                state, dict(zip(jt.joint_names, point.positions))
            ),
            f"Waypoint {i + 1}/{len(jt.points)}",
        )


def wait_for_settled_tcp(node, arm, pos_tol, old_stamp, timeout=12.0):
    """Wait for delayed real robot feedback, not the first post-action TF."""
    time.sleep(0.5)
    deadline = time.monotonic() + timeout
    last_stamp = old_stamp
    consecutive = 0
    latest = None
    last_error_mm = float("inf")
    last_error_deg = float("inf")
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            t = node.tf.lookup_transform("world", TCP[arm], Time())
        except TransformException:
            continue
        stamp = Time.from_msg(t.header.stamp).nanoseconds
        age = (node.get_clock().now().nanoseconds - stamp) / 1e9
        if stamp <= last_stamp or age < -0.5 or age > node.max_tf_age:
            continue
        last_stamp = stamp
        latest = position(t)
        last_error_mm = distance(latest, target_xyz(arm)) * 1000.0
        o = t.transform.rotation
        q = QUATERNION[arm]
        dot = abs(o.x*q[0] + o.y*q[1] + o.z*q[2] + o.w*q[3])
        last_error_deg = math.degrees(
            2.0 * math.acos(max(0.0, min(1.0, dot)))
        )
        if last_error_mm <= pos_tol * 1000.0 and last_error_deg <= 3.0:
            consecutive += 1
            if consecutive >= 3:
                print(
                    f"Arm {arm.upper()} settled TCP: {fmt(latest)}, "
                    f"XYZ error {last_error_mm:.3f} mm, "
                    f"orientation error {last_error_deg:.3f} deg"
                )
                return t
        else:
            consecutive = 0
    raise RuntimeError(
        f"Arm {arm.upper()} TF not settled within {timeout:.1f}s; "
        f"last xyz={latest}, xyz error={last_error_mm:.3f} mm, "
        f"orientation error={last_error_deg:.3f} deg; NO auto retry"
    )


def checked_trajectory(trajectory, arms):
    joints = set(trajectory.joint_trajectory.joint_names)
    expected = {
        f"arm_{arm}_j{i}" for arm in arms for i in range(1, 7)
    }
    if not trajectory.joint_trajectory.points or joints != expected:
        raise RuntimeError(
            f"Empty or unexpected controller joint set: "
            f"got {sorted(joints)}, required {sorted(expected)}"
        )
    print(
        f"MoveIt plan OK: {len(trajectory.joint_trajectory.points)} "
        f"joint trajectory points, {len(joints)} controlled joints."
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--arm", required=True, choices=("a", "b", "both"),
        help="Use 'both' for ONE coordinated dual_arms plan"
    )
    parser.add_argument("--execute", action="store_true",
                        help="Send the planned trajectory after manual confirmation")
    parser.add_argument("--velocity-scale", type=float, default=0.05)
    parser.add_argument("--acceleration-scale", type=float, default=0.05)
    parser.add_argument("--max-displacement", type=float, default=0.50,
                        help="Per-arm max current-to-target TCP distance in m, <=0.50")
    parser.add_argument("--position-tolerance", type=float, default=0.005)
    parser.add_argument("--max-tf-age", type=float, default=1.5)
    args = parser.parse_args()

    if not all(math.isfinite(v) for v in (*TARGET, GAP)) or not 0.03 <= GAP <= 0.50:
        parser.error("TARGET must be finite; GAP must be 0.03..0.50 m")
    if not 0 < args.velocity_scale <= 0.2 or not 0 < args.acceleration_scale <= 0.2:
        parser.error("velocity/acceleration scale must be in (0, 0.2]")
    if not 0 < args.max_displacement <= 0.50:
        parser.error("--max-displacement must be in (0, 0.50] m")
    if not 0 < args.position_tolerance <= 0.02 or args.max_tf_age <= 0:
        parser.error("invalid tolerance or TF max age")

    arms = ("a", "b") if args.arm == "both" else (args.arm,)
    rclpy.init()
    node = None
    try:
        # Fail closed on a mock backend for either commanded arm.
        for arm in arms:
            read_real_backend(arm)
        node = WorldTcpGo("a" if args.arm == "both" else args.arm, args.max_tf_age)
        node._both_joint_msg = None
        node._both_joint_arrived = None

        def on_both_joints(msg):
            node._both_joint_msg = msg
            node._both_joint_arrived = time.monotonic()

        node._both_sub = node.create_subscription(
            JointState, "/joint_states", on_both_joints, qos_profile_sensor_data
        )
        before = {arm: fresh_tcp(node, arm) for arm in ("a", "b")}
        print(f"Shared world midpoint: {fmt(TARGET)}")
        print(f"Requested TCP-to-TCP distance along world X: {GAP * 1000:.1f} mm")
        print("TCP +Z: Arm A -> world -X, Arm B -> world +X")
        print("TCP +X: both -> world +Z (jaw opening/closing direction)")
        print("WARNING: 30 mm TCP separation is NOT guaranteed physical "
              "clearance; two grippers and fingers may still collide.")
        blocked_column = False
        for arm in ("a", "b"):
            xyz = target_xyz(arm)
            at_column, detail = column_guard(xyz)
            if at_column:
                blocked_column = True
                print(f"COLUMN GUARD {arm.upper()}: {detail}")
            print(f"Arm {arm.upper()} current: {fmt(position(before[arm]))}")
            print(f"Arm {arm.upper()} target:  {fmt(xyz)}, "
                  f"quat xyzw={QUATERNION[arm]}")
            if arm in arms and distance(position(before[arm]), xyz) > args.max_displacement:
                raise RuntimeError(
                    f"Arm {arm.upper()} target beyond max-displacement="
                    f"{args.max_displacement:.3f} m"
                )
        print(f"Target TCP origin separation: "
              f"{distance(target_xyz('a'), target_xyz('b')) * 1000:.1f} mm")

        group = "dual_arms" if args.arm == "both" else f"arm_{args.arm}"
        moveit = MoveItArm(
            node,
            group_name=group,
            ee_link=TCP["a" if args.arm == "both" else args.arm],
            base_frame="world",
            joint_names=tuple(
                f"arm_{arm}_j{i}" for arm in arms for i in range(1, 7)
            ),
            velocity_scale=args.velocity_scale,
            acceleration_scale=args.acceleration_scale,
            planning_time=15.0,
            planning_attempts=20,
            position_tolerance=args.position_tolerance,
            orientation_tolerance=math.radians(3),
        )
        moveit.wait_until_ready(timeout_sec=15)
        actual_state = current_robot_state(node)
        validity = await_service(node, GetStateValidity, "/check_state_validity")
        require_valid_state(node, validity, actual_state, "Current robot")
        fk = await_service(node, GetPositionFK, "/compute_fk")
        if args.arm == "both":
            ik = await_service(node, GetPositionIK, "/compute_ik")
            print("Computing collision-aware IK for EACH arm; checking "
                  "ONE combined 12-joint goal and full-robot collisions.")
            goal_state = solve_arm_ik(node, ik, "a", actual_state)
            goal_state = solve_arm_ik(node, ik, "b", goal_state)
            require_valid_state(node, validity, goal_state, "Combined goal")
            check_target_fk(
                node, fk, goal_state, ("a", "b"),
                args.position_tolerance
            )
            angles = dict(zip(
                goal_state.joint_state.name,
                goal_state.joint_state.position,
            ))
            trajectory = moveit.plan_joints(
                [angles[name] for arm in arms for name in ARM_JOINTS[arm]]
            )
        else:
            trajectory = moveit.plan_pose(
                target_pose(args.arm), frame_id="world"
            )
        checked_trajectory(trajectory, arms)
        check_waypoints(node, validity, actual_state, trajectory)
        print("This test does not publish the trajectory to the RViz "
              "planned-path display. Review scene/clearances before execution.")
        if not args.execute:
            print("PLAN ONLY: no controller or gripper command sent.")
            return 0
        if blocked_column:
            raise RuntimeError(
                "One target is at/in configured column +20 mm margin; "
                "real execution refused. Correct real geometry/target first."
            )
        # No arbitrary TCP-only radius: the full robot's geometry,
        # including the stationary other arm, must pass validity checks.
        if not sys.stdin.isatty():
            raise RuntimeError("--execute requires a human-operated terminal")

        token = (
            "MOVE_BOTH_ARMS_30MM" if args.arm == "both"
            else f"MOVE_ARM_{args.arm.upper()}_TO_OFFSET_POINT"
        )
        print("VERIFY: both real robots are enabled and stationary; "
              "all grippers/fingers, column, table and fixtures match the "
              "MoveIt collision scene. A 30 mm TCP gap may NOT be safe.")
        if input(f"Type {token} to execute (anything else cancels): ").strip() != token:
            raise RuntimeError("Operator cancelled; no trajectory sent")

        # Refuse a plan if either robot moved while the operator inspected it.
        for arm in ("a", "b"):
            fresh = fresh_tcp(node, arm)
            if (
                distance(position(before[arm]), position(fresh)) > 0.005 or
                quaternion_delta_deg(before[arm], fresh) > 2.0
            ):
                raise RuntimeError(
                    f"Arm {arm.upper()} moved after planning; replan."
                )

        # Fail closed if the full robot is no longer valid just before move.
        require_valid_state(
            node, validity, current_robot_state(node), "Pre-execute robot"
        )
        # ONE action goal, not two concurrent independent motion commands.
        moveit.execute(trajectory)
        for arm in arms:
            wait_for_settled_tcp(
                node, arm, args.position_tolerance,
                Time.from_msg(before[arm].header.stamp).nanoseconds,
            )
        if args.arm == "both":
            a, b = fresh_tcp(node, "a"), fresh_tcp(node, "b")
            print(
                f"Final TCP origin separation: "
                f"{distance(position(a), position(b)) * 1000.0:.3f} mm"
            )
        print("DONE: TF/model results only, not external physical metrology.")
        return 0
    except (RuntimeError, MoveItError, OSError, ValueError, KeyError,
            KeyboardInterrupt, EOFError) as exc:
        print(f"STOP: {exc}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
