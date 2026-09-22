#!/usr/bin/env python3
"""Plan an individual FR3 TCP at one shared world point, facing +/-world X.

Arm A: TCP +Z (finger forward) -> world -X.
Arm B: TCP +Z (finger forward) -> world +X.
Both: TCP +X (jaw opening direction) -> world +Z.
The long direction of the opening follows finger forward, NOT the line
between the fingers. These two meanings of "gap parallel X" are different.

The two arms must NEVER occupy this same target together. Run ONE arm at a
time, move it away before testing the other; there is no automatic retreat.

Requires running: ros2 launch fairino3_dual_moveit_config dual_bringup.launch.py
No extra SDK/RPC, no gripper commands, no controller switching, no TF override.
Default is plan-only. Real motion requires --execute AND manual confirmation.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
import rclpy
from rclpy.time import Time
from tf2_ros import TransformException
import yaml

from fr_control.moveit_arm import MoveItArm, MoveItError
from go_world_tcp import WorldTcpGo, distance, fmt, position, read_real_backend


TARGET = (0.0, 0.3, 1.1)
ROOT_HALF = math.sqrt(0.5)
# Quaternion xyzw. Jaw separation: TCP +X = world +Z for BOTH.
# A: TCP +Z = world -X (Ry(-90 deg))
# B: TCP +Z = world +X (180 deg about (world X+Z)/sqrt(2))
QUATERNION = {
    "a": (0.0, -ROOT_HALF, 0.0, ROOT_HALF),
    "b": (ROOT_HALF, 0.0, ROOT_HALF, 0.0),
}
OTHER_TCP = {"a": "arm_b_gripper_tcp", "b": "arm_a_gripper_tcp"}


def target_pose(arm: str) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = TARGET
    q = QUATERNION[arm]
    pose.orientation.x, pose.orientation.y = q[0], q[1]
    pose.orientation.z, pose.orientation.w = q[2], q[3]
    return pose


def near_column() -> tuple[bool, str]:
    """Independent hard stop for target inside/near the configured column box.

    Only checks nominal TCP point. MoveIt must ALSO collision-check the full
    robot, gripper and swept path. The guard cannot certify safe motion.
    """
    path = os.path.join(
        get_package_share_directory("fairino3_dual_moveit_config"),
        "config", "dual_workcell.yaml"
    )
    with open(path, encoding="utf-8") as stream:
        cfg = yaml.safe_load(stream)
    col = cfg["column"]
    center = col["initial_pose"]["position"]
    size = col["dimensions"]
    margin = 0.020  # 2 cm TCP clearance from modelled column
    hits = [
        abs(TARGET[i] - float(center[axis])) <=
        float(size[axis]) / 2.0 + margin
        for i, axis in enumerate(("x", "y", "z"))
    ]
    return all(hits), (
        f"configured mounting_column: center "
        f"({center['x']}, {center['y']}, {center['z']}), "
        f"size ({size['x']}, {size['y']}, {size['z']}); "
        f"target within {margin * 1000:.0f} mm margin"
    )


def other_arm_pose(node: WorldTcpGo, timeout=8.0) -> tuple[float, float, float]:
    """Require fresh other-arm TF; do not plan assuming an absent arm."""
    other = OTHER_TCP["a" if node.group == "arm_a" else "b"]
    deadline = time.monotonic() + timeout
    reason = ""
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            tf = node.tf.lookup_transform("world", other, Time())
        except TransformException as exc:
            reason = str(exc)
            continue
        stamp = Time.from_msg(tf.header.stamp).nanoseconds
        age = (node.get_clock().now().nanoseconds - stamp) / 1e9
        if stamp == 0 or age < -0.5 or age > node.max_tf_age:
            reason = f"other arm TF stale: {age:.3f} s"
            continue
        return position(tf)
    raise RuntimeError(f"Other arm {other} unavailable/stale: {reason}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm", required=True, choices=("a", "b"),
                        help="Test only one arm. Never send both to this point.")
    parser.add_argument("--execute", action="store_true",
                        help="Allow motion only after successful plan and typing confirmation")
    parser.add_argument("--velocity-scale", type=float, default=0.05)
    parser.add_argument("--acceleration-scale", type=float, default=0.05)
    parser.add_argument("--max-displacement", type=float, default=0.50,
                        help="Max start-to-target TCP displacement, metres (<=0.50)")
    parser.add_argument("--position-tolerance", type=float, default=0.005,
                        help="Post-motion TF/model position tolerance in metres")
    parser.add_argument("--max-tf-age", type=float, default=1.5)
    args = parser.parse_args()
    if not (0 < args.velocity_scale <= 0.2 and
            0 < args.acceleration_scale <= 0.2):
        parser.error("velocity/acceleration scales must be in (0, 0.2]")
    if not 0 < args.max_displacement <= 0.50:
        parser.error("--max-displacement must be in (0, 0.50] metres")
    if args.position_tolerance <= 0 or args.max_tf_age <= 0:
        parser.error("position-tolerance/max-tf-age must be positive")

    rclpy.init()
    node = None
    try:
        read_real_backend(args.arm)
        node = WorldTcpGo(args.arm, args.max_tf_age)
        now, old_stamp = node.current_world_tcp()
        current_xyz = position(now)
        other_xyz = other_arm_pose(node)
        separation = distance(other_xyz, TARGET)
        unsafe_column, column_detail = near_column()

        print(f"Arm {args.arm.upper()} only; TCP={node.tcp}, group={node.group}")
        print(f"Current world TCP: {fmt(current_xyz)}")
        print(f"Target  world TCP: {fmt(TARGET)}")
        print(f"Other arm TCP:     {fmt(other_xyz)}")
        print(f"Other TCP to target: {separation * 1000:.1f} mm")
        print(f"Travel: {distance(current_xyz, TARGET) * 1000:.1f} mm")
        print("Jaw opening TCP +X -> world +Z; finger forward TCP +Z -> " +
              ("world -X" if args.arm == "a" else "world +X"))
        print(f"Target quaternion xyzw: {QUATERNION[args.arm]}")
        print("WARNING: two grippers cannot occupy this identical TCP target "
              "at the same time; do not run the other arm until this one retreats.")
        if unsafe_column:
            print("COLUMN GUARD WARNING: " + column_detail)
            print("This point is in/near the CONFIGURED column; "
                  "real motion is disallowed until the workcell/target is corrected.")
        if distance(current_xyz, TARGET) > args.max_displacement:
            raise RuntimeError(
                f"Target exceeds max-displacement={args.max_displacement:.3f} m"
            )
        if separation < 0.15:
            raise RuntimeError(
                "Other arm TCP already within 150 mm of shared target. "
                "Both arms must not occupy the same target; move the other arm "
                "away safely before proceeding."
            )

        arm = MoveItArm(
            node, group_name=node.group, ee_link=node.tcp, base_frame=node.base,
            joint_names=tuple(f"{node.group}_j{i}" for i in range(1, 7)),
            velocity_scale=args.velocity_scale,
            acceleration_scale=args.acceleration_scale,
            planning_time=10.0, planning_attempts=16,
            position_tolerance=args.position_tolerance,
            orientation_tolerance=math.radians(3),
        )
        arm.wait_until_ready(timeout_sec=15)
        plan = arm.plan_pose(target_pose(args.arm), frame_id="world")
        points = plan.joint_trajectory.points
        actual_names = set(plan.joint_trajectory.joint_names)
        required = {f"{node.group}_j{i}" for i in range(1, 7)}
        if not points or not required.issubset(actual_names):
            raise RuntimeError("Empty or wrong-arm trajectory; refusing execution")
        print(f"MoveIt plan OK: {len(points)} trajectory points.")
        if not args.execute:
            print("PLAN ONLY. No motion sent. NOTE: this client does not "
                  "automatically publish a visible RViz planned path.")
            return 0
        if unsafe_column:
            raise RuntimeError(
                "Real execution blocked by configured column intersection. "
                "Check actual pillar dimensions/location and adjust target or "
                "workcell configuration; do NOT disable collision checks."
            )
        if not sys.stdin.isatty():
            raise RuntimeError("--execute requires an interactive terminal")

        token = f"MOVE_ARM_{args.arm.upper()}_TO_SHARED_POINT"
        print("VERIFY: real column/table/other arm match MoveIt scene. "
              "The other arm must be away from the shared target. "
              "A successful plan does NOT guarantee real clearance.")
        if input(f"Type {token} to execute (anything else cancels): ").strip() != token:
            raise RuntimeError("Cancelled before motion")

        # The old plan is invalid if the other arm has moved after planning.
        other_latest = other_arm_pose(node)
        if distance(other_xyz, other_latest) > 0.005:
            raise RuntimeError("Other arm moved after planning; refuse stale plan")
        latest, _ = node.current_world_tcp()
        if distance(current_xyz, position(latest)) > 0.005:
            raise RuntimeError("Selected arm moved after planning; refuse stale plan")
        arm.execute(plan)

        reached, _ = node.current_world_tcp(newer_than=old_stamp)
        actual = position(reached)
        error_mm = distance(actual, TARGET) * 1000.0
        print(f"Final world TCP:  {fmt(actual)}")
        print(f"Model/TF XYZ error: {error_mm:.3f} mm")
        if error_mm > args.position_tolerance * 1000.0:
            raise RuntimeError("Position tolerance exceeded; no auto-retry")
        print("Position tolerance passed (TF/model, not external metrology).")
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
