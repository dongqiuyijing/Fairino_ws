#!/usr/bin/env python3
"""Guarded world-frame TCP XYZ target for an already-running dual FR3 MoveIt stack.

Requires: ros2 launch fairino3_dual_moveit_config dual_bringup.launch.py
Uses the existing MoveItArm client, robot_description / TF and collision scene.
No FAIRINO RPC, no second ServoJ client, no gripper commands, no TF overrides.

Example (plan only):
  python3 src/fr_control/test/go_world_tcp.py --arm a --x 0.1 --y 0.4 --z 1.0
Execute only after inspecting the RViz plan, from an interactive terminal:
  python3 src/fr_control/test/go_world_tcp.py --arm a --x 0.1 --y 0.4 --z 1.0 --execute

Only XYZ is entered; the current world-frame TCP orientation is preserved.
World xyz must be in METRES, not millimetres.
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
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from fr_control.moveit_arm import MoveItArm, MoveItError


ARMS = {
    "a": ("arm_a", "arm_a_gripper_tcp", "arm_a_base_link"),
    "b": ("arm_b", "arm_b_gripper_tcp", "arm_b_base_link"),
}


def read_real_backend(arm: str) -> None:
    """Fail closed if installed dual configuration says this arm is mock."""
    share = get_package_share_directory("fairino3_dual_moveit_config")
    path = os.path.join(share, "config", "dual_backends.yaml")
    with open(path, encoding="utf-8") as stream:
        backends = yaml.safe_load(stream) or {}
    name = ARMS[arm][0]
    backend = str((backends.get(name) or {}).get("backend", "mock")).lower()
    if backend != "real":
        raise RuntimeError(
            f"{name} backend={backend!r} in {path}; expected real. "
            "Do not use this test with a mock robot."
        )


class WorldTcpGo(Node):
    def __init__(self, arm: str, max_tf_age: float) -> None:
        super().__init__("go_world_tcp")
        self.group, self.tcp, self.base = ARMS[arm]
        self.tf = Buffer(cache_time=Duration(seconds=10.0))
        self.listener = TransformListener(self.tf, self)
        self.max_tf_age = max_tf_age

    def current_world_tcp(self, timeout: float = 8.0, newer_than: int = -1):
        deadline = time.monotonic() + timeout
        last_error = ""
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                t = self.tf.lookup_transform("world", self.tcp, Time())
            except TransformException as exc:
                last_error = str(exc)
                continue

            stamp = Time.from_msg(t.header.stamp).nanoseconds
            age = (self.get_clock().now().nanoseconds - stamp) / 1e9
            if stamp == 0 or stamp <= newer_than:
                last_error = "world TCP TF has no new dynamic timestamp"
                continue
            if age < -0.5 or age > self.max_tf_age:
                last_error = f"world TCP TF is stale / time unsynchronized: age={age:.3f}s"
                continue
            return t, stamp
        raise RuntimeError(
            f"No fresh TF world <- {self.tcp} after {timeout:.1f}s: {last_error}. "
            "Check /joint_states and robot_state_publisher."
        )


def position(t):
    v = t.transform.translation
    return (v.x, v.y, v.z)


def distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def fmt(xyz):
    return "(" + ", ".join(f"{v:+.6f}" for v in xyz) + ") m"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm", required=True, choices=("a", "b"))
    parser.add_argument("--x", required=True, type=float, help="world X, metres")
    parser.add_argument("--y", required=True, type=float, help="world Y, metres")
    parser.add_argument("--z", required=True, type=float, help="world Z, metres")
    parser.add_argument("--execute", action="store_true",
                        help="Move after planning and a second human confirmation")
    parser.add_argument("--max-displacement", type=float, default=0.50,
                        help="Maximum permitted straight-line distance from current TCP "
                             "to target (metres); default 0.50")
    parser.add_argument("--velocity-scale", type=float, default=0.05)
    parser.add_argument("--acceleration-scale", type=float, default=0.05)
    parser.add_argument("--max-tf-age", type=float, default=1.5)
    parser.add_argument("--position-tolerance", type=float, default=0.005,
                        help="post-execution TCP position check, metres; default 0.005")
    args = parser.parse_args()

    if not all(math.isfinite(v) for v in (args.x, args.y, args.z)):
        parser.error("target XYZ must be finite numbers in metres")
    if not 0 < args.max_displacement <= 0.50:
        parser.error("0 < --max-displacement <= 0.50 m for this guarded test")
    if not (0 < args.velocity_scale <= 0.2 and
            0 < args.acceleration_scale <= 0.2):
        parser.error("velocity and acceleration scaling must be in (0, 0.2]")
    if args.max_tf_age <= 0 or args.position_tolerance <= 0:
        parser.error("--max-tf-age and --position-tolerance must be positive")

    rclpy.init()
    node = None
    try:
        read_real_backend(args.arm)
        node = WorldTcpGo(args.arm, args.max_tf_age)
        current, old_stamp = node.current_world_tcp()
        start = position(current)
        target = (args.x, args.y, args.z)
        travel = distance(start, target)

        print(f"Arm {args.arm.upper()}: group={node.group}, TCP={node.tcp}")
        print(f"current world TCP: {fmt(start)}")
        print(f"target  world TCP: {fmt(target)}")
        print(f"XYZ displacement: {travel * 1000:.3f} mm")
        print("TCP orientation: preserve current world-frame quaternion")
        if travel > args.max_displacement:
            raise RuntimeError(
                f"Target is {travel:.4f} m from current TCP, exceeding "
                f"{args.max_displacement:.4f} m. Use nearer staged targets."
            )

        q = current.transform.rotation
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = target
        pose.orientation.x, pose.orientation.y = q.x, q.y
        pose.orientation.z, pose.orientation.w = q.z, q.w

        arm = MoveItArm(
            node,
            group_name=node.group,
            ee_link=node.tcp,
            base_frame=node.base,
            joint_names=tuple(f"{node.group}_j{i}" for i in range(1, 7)),
            velocity_scale=args.velocity_scale,
            acceleration_scale=args.acceleration_scale,
            planning_time=10.0,
            planning_attempts=12,
            position_tolerance=args.position_tolerance,
            orientation_tolerance=0.05,
        )
        arm.wait_until_ready(timeout_sec=15.0)
        # This only asks MoveIt to compute a path; no controller goal is sent.
        trajectory = arm.plan_pose(pose, frame_id="world")
        points = trajectory.joint_trajectory.points
        if not points:
            raise RuntimeError("MoveIt returned an empty plan; no motion sent")
        joint_names = set(trajectory.joint_trajectory.joint_names)
        required = {f"{node.group}_j{i}" for i in range(1, 7)}
        if not required.issubset(joint_names):
            raise RuntimeError(
                f"Planned joints mismatch {node.group}: "
                f"got {sorted(joint_names)}; refusing execution"
            )
        print(f"MoveIt plan OK: {len(points)} trajectory points. "
              "Inspect the planned path and collision scene in RViz.")
        if not args.execute:
            print("PLAN ONLY: no trajectory executed. Add --execute when safe.")
            return 0

        if not sys.stdin.isatty():
            raise RuntimeError("--execute requires an interactive terminal")
        token = f"MOVE_ARM_{args.arm.upper()}"
        print("CAUTION: the arm will move to the world-frame TCP pose above. "
              "Verify real table, other arm, column and fixtures match the "
              "MoveIt scene; clear personnel and objects.")
        if input(f"Type {token} to execute (anything else cancels): ").strip() != token:
            raise RuntimeError("Operator cancelled; no trajectory executed")
        arm.execute(trajectory)

        actual, _ = node.current_world_tcp(timeout=8.0, newer_than=old_stamp)
        final = position(actual)
        error = distance(final, target)
        print(f"final world TCP:  {fmt(final)}")
        print(f"target world TCP: {fmt(target)}")
        print(f"model/TF position error: {error * 1000:.3f} mm")
        if error > args.position_tolerance:
            raise RuntimeError(
                f"TCP model/TF error exceeds tolerance: "
                f"{error * 1000:.3f} > {args.position_tolerance * 1000:.3f} mm; "
                "stop and inspect instead of automatically retrying"
            )
        print("DONE. TF/model tolerance passed; this is not independent "
              "physical metrology accuracy.")
        return 0

    except (RuntimeError, MoveItError, FileNotFoundError, KeyError,
            ValueError, KeyboardInterrupt, EOFError) as exc:
        print(f"STOP: {exc}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
