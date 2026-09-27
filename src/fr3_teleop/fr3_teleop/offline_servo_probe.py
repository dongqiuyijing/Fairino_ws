"""Repeatable isolated MoveIt Servo trajectory probe for either FR3 arm."""

from __future__ import annotations

import argparse
from math import isfinite
import sys
from time import monotonic

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory


DIRECTIONS = (("x", 1), ("x", -1), ("y", 1), ("y", -1), ("z", 1), ("z", -1),
              ("rx", 1), ("rx", -1), ("ry", 1), ("ry", -1), ("rz", 1), ("rz", -1))
OFFLINE_NAMESPACE = "/fr3_teleop_offline"


class Probe(Node):
    def __init__(self, arm: str) -> None:
        super().__init__(f"fr3_teleop_offline_servo_probe_{arm}")
        self.arm, self.expected = arm, [f"{arm}_j{i}" for i in range(1, 7)]
        self.pub = self.create_publisher(
            TwistStamped, f"{OFFLINE_NAMESPACE}/{arm}/delta_twist_cmds", 20
        )
        self.create_subscription(
            JointTrajectory,
            f"{OFFLINE_NAMESPACE}/{arm}_teleop_controller/joint_trajectory",
            self._trajectory,
            100,
        )
        self.start_servo = self.create_client(
            Trigger, f"{OFFLINE_NAMESPACE}/offline_{arm}_servo/start_servo"
        )
        if not self.start_servo.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"MoveIt Servo start service is unavailable for {arm}")
        self.start_future = self.start_servo.call_async(Trigger.Request())
        self.index, self.phase, self.failure = 0, "starting", ""
        self.started = self.phase_started = monotonic()
        self.samples: list[tuple[float, tuple[float, ...]]] = []
        self.responses: dict[tuple[str, int], tuple[float, ...]] = {}
        self.timer = self.create_timer(0.02, self._tick)

    def _trajectory(self, msg: JointTrajectory) -> None:
        if msg.joint_names != self.expected:
            self.failure = f"joint isolation failure: got {list(msg.joint_names)} expected {self.expected}"; return
        if not msg.points or len(msg.points[-1].positions) != 6:
            self.failure = "trajectory does not contain six position values"; return
        pos = tuple(msg.points[-1].positions)
        if not all(isfinite(value) for value in pos):
            self.failure = "trajectory contains non-finite position"; return
        self.samples.append((monotonic(), pos))

    def _message(self, axis: str, sign: int) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        value = 0.005 * sign if axis in {"x", "y", "z"} else 0.02 * sign
        if axis == "x": msg.twist.linear.x = value
        elif axis == "y": msg.twist.linear.y = value
        elif axis == "z": msg.twist.linear.z = value
        elif axis == "rx": msg.twist.angular.x = value
        elif axis == "ry": msg.twist.angular.y = value
        else: msg.twist.angular.z = value
        return msg

    def _zero_message(self) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        return msg

    def _tick(self) -> None:
        now = monotonic()
        if self.failure: self._finish(); return
        if now - self.started > 45.0: self.failure = "probe timeout"; self._finish(); return
        axis, sign = DIRECTIONS[self.index]; elapsed = now - self.phase_started
        if self.phase == "starting":
            if not self.start_future.done():
                return
            try:
                reply = self.start_future.result()
            except Exception as error:
                self.failure = f"could not start Servo: {error}"; self._finish(); return
            if not reply.success:
                self.failure = f"could not start Servo: {reply.message}"; self._finish(); return
            self.phase, self.phase_started = "command", now
            return
        if self.phase == "command":
            self.pub.publish(self._message(axis, sign))
            if elapsed >= 0.35:
                before = self.samples[0][1] if self.samples else None; after = self.samples[-1][1] if self.samples else None
                if before is None or after is None or max(abs(a - b) for a, b in zip(after, before)) < 1e-8:
                    self.failure = f"{axis}{sign:+}: no joint response"; self._finish(); return
                self.responses[(axis, sign)] = tuple(a - b for a, b in zip(after, before))
                self.samples.clear(); self.phase, self.phase_started = "halt", now
        else:
            self.pub.publish(self._zero_message())
            if elapsed >= 0.45:
                if len(self.samples) >= 2 and max(abs(a - b) for a, b in zip(self.samples[-1][1], self.samples[-2][1])) > 1e-5:
                    self.failure = f"{axis}{sign:+}: halt continued moving"; self._finish(); return
                self.samples.clear(); self.index += 1; self.phase, self.phase_started = "command", now
                if self.index == len(DIRECTIONS): self._validate_trends(); self._finish()

    def _validate_trends(self) -> None:
        for axis in ("x", "y", "z", "rx", "ry", "rz"):
            positive, negative = self.responses[(axis, 1)], self.responses[(axis, -1)]
            if max(abs(value) for value in positive) < 1e-8 or max(abs(value) for value in negative) < 1e-8 or sum(a * b for a, b in zip(positive, negative)) >= 0.0:
                self.failure = f"{axis}: positive/negative responses not opposed"; return

    def _finish(self) -> None:
        self.pub.publish(self._zero_message()); self.destroy_timer(self.timer)
        self.get_logger().info(f"OFFLINE_SERVO_PROBE_{self.arm.upper()}_{'FAIL: ' + self.failure if self.failure else 'PASS'}")
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--arm", choices=("arm_a", "arm_b"), required=True)
    args, ros_args = parser.parse_known_args(); rclpy.init(args=ros_args)
    node = Probe(args.arm)
    try: rclpy.spin(node)
    finally:
        failed = bool(node.failure); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    return 1 if failed else 0


if __name__ == "__main__": sys.exit(main())
