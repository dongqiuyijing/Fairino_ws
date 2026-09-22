#!/usr/bin/env python3
"""Plan two opposed FR3 gripper TCPs to a 30 mm X-axis separation.

At the shared midpoint TARGET in world coordinates:
  Arm A TCP = TARGET + (GAP/2, 0, 0), finger forward (TCP +Z) = world -X.
  Arm B TCP = TARGET - (GAP/2, 0, 0), finger forward (TCP +Z) = world +X.
  Both TCP +X (the jaw opening/closing direction) = world +Z.

--arm both requests ONE MoveIt plan for the 'dual_arms' group with BOTH
TCP poses constrained. It executes ONE combined RobotTrajectory through
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


def dual_constraints(node: WorldTcpGo, pos_tol: float) -> Constraints:
    """One goal state must satisfy BOTH 6D TCP poses simultaneously."""
    goal = Constraints()
    header = Header()
    header.frame_id = "world"
    header.stamp = node.get_clock().now().to_msg()
    for arm in ("a", "b"):
        pose = target_pose(arm)
        region = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [pos_tol]
        region.primitives.append(sphere)
        center = Pose()
        center.position = pose.position
        center.orientation.w = 1.0
        region.primitive_poses.append(center)

        pc = PositionConstraint()
        pc.header = header
        pc.link_name = TCP[arm]
        pc.constraint_region = region
        pc.weight = 1.0
        goal.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header = header
        oc.link_name = TCP[arm]
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = math.radians(3)
        oc.absolute_y_axis_tolerance = math.radians(3)
        oc.absolute_z_axis_tolerance = math.radians(3)
        oc.weight = 1.0
        goal.orientation_constraints.append(oc)
    return goal


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
        if args.arm == "both":
            print("Planning ONE 12-joint trajectory with BOTH TCP goal poses "
                  "and both-arm collision checking.")
            # Reuse the existing MoveItArm's MoveGroup plan-only request path.
            # plan_pose() handles only one link; the combined goal has two.
            trajectory = moveit._plan(
                dual_constraints(node, args.position_tolerance),
                "dual_arms, Arm A/B opposing TCP poses",
            )
        else:
            trajectory = moveit.plan_pose(target_pose(args.arm), frame_id="world")
        checked_trajectory(trajectory, arms)
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
