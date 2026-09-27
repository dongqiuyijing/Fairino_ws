"""Isolated MoveIt Servo velocity -> position-integrator -> JGPC probe."""
from __future__ import annotations

import argparse
from math import isfinite
from statistics import median
import sys
from time import monotonic

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from .servo_position_accumulator import ServoVelocityIntegrator, VelocityIntegratorConfig

# This probe exercises the continuous velocity path required for the first
# real-arm trial.  It deliberately does not use an arbitrary fake posture to
# claim that every Cartesian axis is valid; singularity/collision behaviour
# for the other axes belongs to the real planning scene validation.
DIRECTIONS = (("x", 1), ("x", -1))
OFFLINE_NAMESPACE = "/fr3_teleop_offline"


class Probe(Node):
    def __init__(self, arm: str) -> None:
        super().__init__(f"fr3_teleop_offline_servo_probe_{arm}")
        self.arm, self.expected = arm, [f"{arm}_j{i}" for i in range(1, 7)]
        self.twist_pub = self.create_publisher(TwistStamped, f"{OFFLINE_NAMESPACE}/{arm}/delta_twist_cmds", 20)
        # Offline only: the production manager is the one real final publisher.
        self.command_pub = self.create_publisher(Float64MultiArray, f"{OFFLINE_NAMESPACE}/{arm}_teleop_controller/commands", 100)
        self.create_subscription(Float64MultiArray, f"{OFFLINE_NAMESPACE}/{arm}/servo_raw_commands", self._velocity, 100)
        self.create_subscription(Float64MultiArray, f"{OFFLINE_NAMESPACE}/{arm}_teleop_controller/commands", self._command, 200)
        self.create_subscription(JointState, f"{OFFLINE_NAMESPACE}/joint_states", self._joint_state, 50)
        self.start_servo = self.create_client(Trigger, f"{OFFLINE_NAMESPACE}/offline_{arm}_servo/start_servo")
        if not self.start_servo.wait_for_service(timeout_sec=5.0): raise RuntimeError(f"Servo start service unavailable for {arm}")
        self.start_future = self.start_servo.call_async(Trigger.Request())
        self.integrator = ServoVelocityIntegrator(VelocityIntegratorConfig(0.02, 3.0, 0.12, 0.02))
        self.measured: list[float] | None = None; self.motion_active = False
        self.index, self.phase, self.failure, self.finished = 0, "starting", "", False
        self.started = self.phase_started = monotonic()
        self.raw_samples: list[tuple[float, tuple[float, ...]]] = []
        self.command_samples: list[tuple[float, tuple[float, ...]]] = []
        self.responses: dict[tuple[str, int], tuple[float, ...]] = {}
        self.timer = self.create_timer(0.02, self._tick)
        self.output_timer = self.create_timer(0.008, self._output_tick)

    def _joint_state(self, msg: JointState) -> None:
        values = dict(zip(msg.name, msg.position))
        if all(name in values for name in self.expected):
            candidate = [float(values[name]) for name in self.expected]
            if all(isfinite(value) for value in candidate):
                self.measured = candidate

    def _velocity(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != 6 or not all(isfinite(value) for value in msg.data): self.failure = "raw velocity is not six finite values"; return
        now = monotonic(); self.raw_samples.append((now, tuple(msg.data)))
        if self.motion_active:
            error = self.integrator.accept_velocity(msg.data, now)
            if error: self.failure = error

    def _command(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != 6 or not all(isfinite(value) for value in msg.data): self.failure = "final command is not six finite positions"; return
        self.command_samples.append((monotonic(), tuple(msg.data)))

    def _output_tick(self) -> None:
        if self.measured is None: return
        try: command = self.integrator.tick(self.measured, monotonic(), self.motion_active)
        except (RuntimeError, ValueError) as error: self.failure = f"integrator output failed: {error}"; return
        self.command_pub.publish(Float64MultiArray(data=list(command)))

    def _message(self, axis: str, sign: int) -> TwistStamped:
        msg = TwistStamped(); msg.header.stamp = self.get_clock().now().to_msg(); msg.header.frame_id = "world"
        value = 0.005 * sign if axis in {"x", "y", "z"} else 0.02 * sign
        if axis == "x": msg.twist.linear.x = value
        elif axis == "y": msg.twist.linear.y = value
        elif axis == "z": msg.twist.linear.z = value
        elif axis == "rx": msg.twist.angular.x = value
        elif axis == "ry": msg.twist.angular.y = value
        else: msg.twist.angular.z = value
        return msg

    def _zero(self) -> TwistStamped:
        msg = TwistStamped(); msg.header.stamp = self.get_clock().now().to_msg(); msg.header.frame_id = "world"; return msg

    @staticmethod
    def _rate(samples):
        if len(samples) < 3: return None
        gaps = [b[0] - a[0] for a, b in zip(samples, samples[1:])]
        return 1.0 / median(gaps) if median(gaps) > 0 else None

    def _validate_motion(self, axis: str, sign: int) -> None:
        if len(self.raw_samples) < 4 or len(self.command_samples) < 10: self.failure = f"{axis}{sign:+}: insufficient samples"; return
        raw_rate, output_rate = self._rate(self.raw_samples), self._rate(self.command_samples)
        if raw_rate is None or not 15 <= raw_rate <= 35: self.failure = f"{axis}{sign:+}: raw rate {raw_rate!r}, expected ~25Hz"; return
        if output_rate is None or not 80 <= output_rate <= 150: self.failure = f"{axis}{sign:+}: output rate {output_rate!r}, expected ~125Hz"; return
        if max(abs(v) for _, values in self.raw_samples for v in values) < 1e-8: self.failure = f"{axis}{sign:+}: raw velocity remained zero"; return
        before, after = self.command_samples[0][1], self.command_samples[-1][1]
        response = tuple(a - b for a, b in zip(after, before))
        if max(abs(v) for v in response) < 1e-8: self.failure = f"{axis}{sign:+}: final position did not ramp"; return
        changes = [(nxt[0], max(abs(b - a) for a, b in zip(prev[1], nxt[1]))) for prev, nxt in zip(self.command_samples, self.command_samples[1:])]
        changed = [stamp for stamp, step in changes if step > 1e-10]
        if len(changed) < 4:
            self.failure = f"{axis}{sign:+}: final ramp has only {len(changed)} updates"; return
        if len(changed) >= 3:
            ramp_rate = 1.0 / median([later - earlier for earlier, later in zip(changed, changed[1:])])
            if ramp_rate < 60.0:
                self.failure = f"{axis}{sign:+}: final ramp update rate {ramp_rate:.1f}Hz is stair-stepped"; return
        self.responses[(axis, sign)] = response

    def _tick(self) -> None:
        now = monotonic()
        if self.failure: self._finish(); return
        if now - self.started > 70: self.failure = "probe timeout"; self._finish(); return
        if self.measured is None: return
        axis, sign = DIRECTIONS[self.index]
        if self.phase == "starting":
            if not self.start_future.done(): return
            reply = self.start_future.result()
            if not reply.success: self.failure = f"could not start Servo: {reply.message}"; self._finish(); return
            self.integrator.reset_to_measured(self.measured, now); self.phase, self.phase_started = "motion", now; return
        elapsed = now - self.phase_started
        if self.phase == "motion":
            self.motion_active = True; self.twist_pub.publish(self._message(axis, sign))
            if elapsed >= 0.45:
                self._validate_motion(axis, sign)
                if self.failure: self._finish(); return
                self.motion_active = False; self.twist_pub.publish(self._zero()); self.phase, self.phase_started = "settle_hold", now
        elif self.phase == "settle_hold":
            self.motion_active = False; self.twist_pub.publish(self._zero())
            # Drop transport backlog from the final moving samples before
            # evaluating whether the fixed hold itself drifts.
            if elapsed >= 0.12:
                self.command_samples.clear(); self.phase, self.phase_started = "hold", now
        else:
            self.motion_active = False; self.twist_pub.publish(self._zero())
            if elapsed >= 0.15:
                if len(self.command_samples) >= 3:
                    spread = max(max(values[i] for _, values in self.command_samples) - min(values[i] for _, values in self.command_samples) for i in range(6))
                    if spread > 1e-10: self.failure = f"{axis}{sign:+}: fixed hold drifted {spread}"; self._finish(); return
                self.raw_samples.clear(); self.command_samples.clear(); self.index += 1
                if self.index == len(DIRECTIONS): self._validate_trends(); self._finish(); return
                self.integrator.reset_to_measured(self.measured, now); self.phase, self.phase_started = "motion", now

    def _validate_trends(self) -> None:
        positive, negative = self.responses[("x", 1)], self.responses[("x", -1)]
        if sum(a * b for a, b in zip(positive, negative)) >= 0:
            self.failure = "x: opposing velocity responses not opposed"

    def _finish(self) -> None:
        if self.finished: return
        self.finished = True; self.motion_active = False; self.twist_pub.publish(self._zero())
        self.destroy_timer(self.timer); self.destroy_timer(self.output_timer)
        outcome = "FAIL: " + self.failure if self.failure else f"PASS joint_order={self.expected} raw≈25Hz output≈125Hz"
        self.get_logger().info(f"OFFLINE_SERVO_PROBE_{self.arm.upper()}_{outcome}")


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--arm", choices=("arm_a", "arm_b"), required=True)
    args, ros_args = parser.parse_known_args(); rclpy.init(args=ros_args); node = Probe(args.arm)
    try:
        while rclpy.ok() and not node.finished: rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        failed = bool(node.failure); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    return int(failed)


if __name__ == "__main__": sys.exit(main())
