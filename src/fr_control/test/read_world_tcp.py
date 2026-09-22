#!/usr/bin/env python3
"""Read the actual-joint-state-driven gripper TCP pose in the world frame.

Run while the matching real ROS 2 bringup is running, after manually moving
the robot to a safe, stationary pose. NO SDK/RPC connection, motion, gripper
commands, controller switching, or TF overrides.

For dual_bringup:
    python3 src/fr_control/test/read_world_tcp.py --arm a
    python3 src/fr_control/test/read_world_tcp.py --arm b
For single-arm real_bringup:
    python3 src/fr_control/test/read_world_tcp.py --arm single
Add --repeat to capture additional points on Enter.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


FRAMES = {
    "a": ("arm_a_gripper_tcp", tuple(f"arm_a_j{i}" for i in range(1, 7))),
    "b": ("arm_b_gripper_tcp", tuple(f"arm_b_j{i}" for i in range(1, 7))),
    "single": ("gripper_tcp", tuple(f"j{i}" for i in range(1, 7))),
}


def quaternion_to_rpy_deg(x: float, y: float, z: float, w: float):
    """RPY of TCP relative to world; XYZ fixed-axis convention, degrees."""
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm < 1e-12:
        raise ValueError("TF contains a zero quaternion")
    x, y, z, w = (v / norm for v in (x, y, z, w))
    roll = math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w*y - z*x))))
    yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
    return tuple(math.degrees(v) for v in (roll, pitch, yaw))


class WorldTcpReader(Node):
    def __init__(self, arm: str, world: str, timeout: float, max_age: float):
        super().__init__("read_world_tcp")
        self.arm = arm
        self.world = world
        self.tcp, self.joint_names = FRAMES[arm]
        self.timeout = timeout
        self.max_age = max_age
        self.joint_msg = None
        self.joint_received_monotonic = None

        self.buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.listener = TransformListener(self.buffer, self)
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self._on_joints, qos_profile_sensor_data
        )

    def _on_joints(self, msg: JointState):
        if all(name in msg.name for name in self.joint_names):
            self.joint_msg = msg
            self.joint_received_monotonic = time.monotonic()

    def capture(self):
        # After a manual reposition, take a NEW complete joint-state sample.
        self.joint_msg = None
        self.joint_received_monotonic = None
        deadline = time.monotonic() + self.timeout
        last_error = ""
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.joint_msg is None:
                last_error = (
                    f"/joint_states has no fresh complete set of "
                    f"{', '.join(self.joint_names)}"
                )
                continue
            try:
                transform = self.buffer.lookup_transform(
                    self.world, self.tcp, Time()
                )
            except TransformException as exc:
                last_error = f"TF {self.world} <- {self.tcp} unavailable: {exc}"
                continue

            # A static-only or stale TF is NOT valid as a real-arm measurement.
            stamp_ns = Time.from_msg(transform.header.stamp).nanoseconds
            now_ns = self.get_clock().now().nanoseconds
            tf_age = (now_ns - stamp_ns) / 1e9
            joints_age = time.monotonic() - self.joint_received_monotonic
            if stamp_ns == 0 or tf_age > self.max_age or tf_age < -0.5:
                last_error = (
                    f"TCP TF timestamp missing/stale: age={tf_age:.3f}s "
                    f"(limit {self.max_age:.3f}s)"
                )
                continue
            if joints_age > self.max_age:
                last_error = f"joint_states stale: age={joints_age:.3f}s"
                continue
            return transform, self.joint_msg, tf_age

        raise RuntimeError(
            f"Cannot take a fresh world TCP measurement in {self.timeout:.1f}s: "
            f"{last_error}. Check bringup, /joint_states, world/base TF and "
            f"robot_state_publisher; do NOT start another FAIRINO SDK/RPC client."
        )


def print_capture(arm: str, world: str, tcp: str, transform, joints, tf_age):
    t = transform.transform.translation
    q = transform.transform.rotation
    rpy = quaternion_to_rpy_deg(q.x, q.y, q.z, q.w)
    data = dict(zip(joints.name, joints.position))
    names = FRAMES[arm][1]
    stamp = transform.header.stamp

    print("\n" + "=" * 64)
    print(f"MEASURED MODEL TCP: T_{world}_{tcp}  (target={world}, source={tcp})")
    print(f"TF stamp: {stamp.sec}.{stamp.nanosec:09d}  age={tf_age:.3f}s")
    print(f"position_m:  x={t.x:+.9f}  y={t.y:+.9f}  z={t.z:+.9f}")
    print(f"position_mm: x={t.x*1000:+.3f}  y={t.y*1000:+.3f}  z={t.z*1000:+.3f}")
    print(f"orientation_xyzw: [{q.x:+.9f}, {q.y:+.9f}, "
          f"{q.z:+.9f}, {q.w:+.9f}]")
    print(f"orientation_rpy_deg: [{rpy[0]:+.6f}, "
          f"{rpy[1]:+.6f}, {rpy[2]:+.6f}]")
    print("actual_joint_states_deg: " +
          "  ".join(f"{n}={math.degrees(data[n]):+.6f}" for n in names))
    print("COPY TARGET (world, metres):")
    print(f"  x: {t.x:.9f}\n  y: {t.y:.9f}\n  z: {t.z:.9f}")
    print("=" * 64, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--arm", choices=tuple(FRAMES), required=True,
        help="a / b for dual_bringup; single for single-arm real_bringup"
    )
    parser.add_argument("--world", default="world", help="world frame, default: world")
    parser.add_argument("--repeat", action="store_true",
                        help="Press Enter for each new captured point; q to quit")
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--max-age", type=float, default=1.5,
                        help="Max dynamic TF and joint sample age in seconds")
    args = parser.parse_args()
    if args.timeout <= 0 or args.max_age <= 0:
        parser.error("--timeout and --max-age must be positive")

    rclpy.init()
    node = WorldTcpReader(args.arm, args.world, args.timeout, args.max_age)
    try:
        print(
            f"READ ONLY: {args.world} <- {node.tcp}. "
            "No arm/gripper commands. World pose is derived from "
            "current /joint_states + published URDF/TF; not an external "
            "physical metrology measurement.",
            flush=True,
        )
        if args.repeat:
            while rclpy.ok():
                if input("Position the robot, then Enter to sample (q to quit): "
                         ).strip().lower() == "q":
                    break
                transform, joints, tf_age = node.capture()
                print_capture(args.arm, args.world, node.tcp,
                              transform, joints, tf_age)
        else:
            transform, joints, tf_age = node.capture()
            print_capture(args.arm, args.world, node.tcp,
                          transform, joints, tf_age)
        return 0
    except (KeyboardInterrupt, EOFError):
        return 0
    except (RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
