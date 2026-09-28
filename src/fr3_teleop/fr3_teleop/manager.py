"""ROS 2 manager. The only process allowed to call a motion backend."""

from __future__ import annotations

import json
from math import pi
from pathlib import Path
from time import monotonic
from typing import Any

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray, Int8, String
from tf2_ros import Buffer, TransformListener, TransformException
import yaml

from .core import ControlCore, MockBackend, Motion, SourceRejected
from .controller_manager_client import AutoTrajectoryMonitor, ControllerManagerClient
from .gripper_bridge import GripperBridgeClient
from .initial_hold import InitialHoldGate
from .robot_state import FeedbackCache
from .servo_position_accumulator import ServoVelocityIntegrator, VelocityIntegratorConfig


def load_config(path: str = "") -> dict[str, Any]:
    filename = Path(path) if path else Path(get_package_share_directory("fr3_teleop")) / "config" / "teleop.yaml"
    with filename.open(encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    if not isinstance(config, dict):
        raise ValueError("teleop configuration must be a mapping")
    return config


class TeleopManager(Node):
    """Watchdog and exclusive-control endpoint for GUI / future joystick inputs."""

    def __init__(self) -> None:
        super().__init__("fr3_teleop_manager")
        self.declare_parameter("config_file", "")
        self.declare_parameter("backend", "")
        config = load_config(str(self.get_parameter("config_file").value))
        self.declare_parameter("allow_real_motion", bool(config["teleop"].get("allow_real_motion", False)))
        config["teleop"]["allow_real_motion"] = bool(self.get_parameter("allow_real_motion").value)
        backend_name = str(self.get_parameter("backend").value) or str(config["teleop"].get("backend", "mock"))
        config["teleop"]["backend"] = backend_name
        if backend_name not in {"mock", "servo"}:
            raise RuntimeError(f"unsupported teleop backend: {backend_name}")
        self._servo_publishers: dict[str, Any] = {}
        self._servo_motion_allowed = bool(config["teleop"].get("allow_real_motion", False))
        if backend_name == "servo":
            backend = _ServoTopicBackend(self, config, self._servo_motion_allowed)
        else:
            backend = MockBackend()
        self.core = ControlCore(config, backend)
        self.config = config
        self._feedback = FeedbackCache(config["robots"])
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._grippers: dict[str, GripperBridgeClient] = {}
        self._manual_ready = backend_name == "mock"
        self._manual_transition = "MOCK" if backend_name == "mock" else "AUTO"
        self._switch_q_before: dict[str, dict[str, float]] = {}
        self._last_switch_q_jump_rad: dict[str, float | None] = {arm: None for arm in config["robots"]}
        self._auto_monitor: AutoTrajectoryMonitor | None = None
        self._controller_client: ControllerManagerClient | None = None
        self._hold_gates: dict[str, InitialHoldGate] = {}
        # This manager owns the only final JGPC publisher.  MoveIt Servo writes
        # exclusively to raw topics; no direct Servo publisher can race this.
        self._command_publishers: dict[str, Any] = {}
        self._integrators: dict[str, ServoVelocityIntegrator] = {}
        self._servo_status: dict[str, int] = {}
        self._hold_started: dict[str, float] = {}
        self._hold_timers: dict[str, Any] = {}
        self._shutdown_started = False
        if backend_name == "servo":
            safety = config["safety"]
            self._auto_monitor = AutoTrajectoryMonitor(self, config["robots"],
                settle_s=float(safety["auto_idle_settle_s"]),
                desired_error_rad=float(safety["auto_idle_desired_error_rad"]),
                stationary_delta_rad=float(safety["auto_idle_stationary_delta_rad"]),
                task_nodes=tuple(safety["known_task_executor_nodes"]))
            self._controller_client = ControllerManagerClient(self, config["robots"], self._auto_monitor)
            velocity_cfg = config["teleop"]["velocity_integrator"]
            integrator_cfg = VelocityIntegratorConfig(
                max_lead_rad=float(velocity_cfg["max_lead_rad"]),
                max_raw_joint_velocity_rad_s=float(velocity_cfg["max_raw_joint_velocity_rad_s"]),
                raw_velocity_timeout_s=float(velocity_cfg["raw_velocity_timeout_s"]),
                max_timer_dt_s=float(velocity_cfg["max_timer_dt_s"]),
                stop_correction_velocity_rad_s=float(velocity_cfg["stop_correction_velocity_rad_s"]),
                stop_hold_tolerance_rad=float(velocity_cfg["stop_hold_tolerance_rad"]),
                stop_settle_timeout_s=float(velocity_cfg["stop_settle_timeout_s"]),
                stop_stationary_delta_rad=float(velocity_cfg["stop_stationary_delta_rad"]),
            )
            for arm, spec in config["robots"].items():
                joints = [f"{spec['joint_prefix']}j{i}" for i in range(1, 7)]
                self._hold_gates[arm] = InitialHoldGate(
                    joints,
                    float(config["teleop"]["max_initial_joint_error_rad"]),
                    float(config["teleop"]["initial_hold_duration_s"]),
                )
                self._command_publishers[arm] = self.create_publisher(
                    Float64MultiArray,
                    f"/{spec['teleop_controller']}/commands", 10,
                )
                self._integrators[arm] = ServoVelocityIntegrator(integrator_cfg)
                self.create_subscription(
                    Float64MultiArray,
                    f"/fr3_teleop/{arm}/servo_raw_commands",
                    lambda msg, key=arm: self._on_servo_raw(key, msg), 50,
                )
                self.create_subscription(
                    Int8,
                    f"/fr3_teleop/{arm}/servo_status",
                    lambda msg, key=arm: self._on_servo_status(key, msg), 20,
                )
                self.create_subscription(
                    __import__("control_msgs.msg", fromlist=["JointTrajectoryControllerState"]).JointTrajectoryControllerState,
                    f"/{spec['auto_controller']}/controller_state",
                    lambda msg, key=arm: self._auto_monitor.note_controller_state(key, msg), 20,
                )
        if bool(config["teleop"].get("allow_real_gripper_service", False)):
            for arm, spec in config["robots"].items():
                self._grippers[arm] = GripperBridgeClient(
                    self, spec["gripper_service"], int(spec["gripper_open_position"]),
                    int(spec["gripper_close_position"]), self._set_gripper_result)
        self.create_subscription(String, "/fr3_teleop/command", self._on_command, 30)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)
        self._status_pub = self.create_publisher(String, "/fr3_teleop/status", 20)
        rate = float(config["teleop"]["input_rate_hz"])
        self.create_timer(1.0 / rate, self._tick)
        self.create_timer(1.0 / float(config["teleop"]["velocity_integrator"]["output_rate_hz"]), self._command_tick)
        self.get_logger().info(
            f"fr3_teleop manager started backend={backend_name}; "
            f"Servo nonzero output={'ENABLED' if self._servo_motion_allowed else 'DISABLED'}"
        )

    def _on_command(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError("JSON command must be an object")
            # Stop the integrator before ControlCore emits its zero Twist.
            # A normal stop keeps the last position command and settles; it
            # must not be overwritten by a later raw velocity sample.
            if data.get("type") in {"stop", "release", "select_arm", "set_frame", "set_speed"}:
                self._hold_integrator(self.core.arm, str(data.get("type")))
            if data.get("type") == "enable" and self.config["teleop"].get("backend") == "servo":
                self._handle_enable(data)
            elif data.get("type") == "motion":
                if not self._manual_ready:
                    # GUI repeats can arrive after an asynchronous safety
                    # fault.  They are input noise, not a new fault cause.
                    # Preserve the original FAULT state and its root reason.
                    self.get_logger().warning("motion ignored: manual control is not ready")
                    return
                if self.config["teleop"].get("first_real_test_mode", False):
                    if not (self.core.arm == "arm_a" and self.core.frame == "base" and data.get("axis") == "x" and int(data.get("sign", 0)) == 1):
                        raise ValueError("first_real_test_mode only permits Arm A / base / X+")
                was_motion_active = self.core.active is not None
                self.core.command(data)
                if self.config["teleop"].get("backend") == "servo":
                    integrator = self._integrators[self.core.arm]
                    if was_motion_active:
                        integrator.begin_motion(monotonic())
                    else:
                        # A new operator motion request is the only command
                        # path allowed to clear an integrator fault latch.
                        integrator.start_new_motion(monotonic())
            elif data.get("type") == "gripper" and self._grippers:
                self._handle_gripper(data)
            else:
                self.core.command(data)
        except SourceRejected as exc:
            self.get_logger().warning(f"teleop source rejected: {exc}")
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self.core._stop("invalid-command")
            self.core.fault = str(exc)
            self.get_logger().error(f"teleop command rejected: {exc}")

    def _measured_for_arm(self, arm: str) -> list[float] | None:
        values = self._feedback.status(
            arm, float(self.config["safety"]["robot_state_timeout_ms"]) / 1000.0
        )["joint_positions_rad"]
        names = [f"{self.config['robots'][arm]['joint_prefix']}j{i}" for i in range(1, 7)]
        try:
            result = [float(values[name]) for name in names]
        except (KeyError, TypeError, ValueError):
            return None
        return result if len(result) == 6 else None

    def _publish_command(self, arm: str, values: tuple[float, ...] | list[float]) -> None:
        publisher = self._command_publishers.get(arm)
        if publisher is not None:
            publisher.publish(Float64MultiArray(data=list(values)))

    def _hold_integrator(self, arm: str, event: str) -> None:
        integrator = self._integrators.get(arm)
        measured = self._measured_for_arm(arm)
        if integrator is None or measured is None:
            return
        # Button release keeps commanding the same arm, so the lead can converge
        # on later 125 Hz ticks. Fault, shutdown, arm changes, and leaving
        # manual mode still capture feedback immediately.
        if event in {"stop", "release"} and not integrator.fault_latched:
            command = integrator.begin_stop_settling(measured, monotonic(), event)
        else:
            command = integrator.enter_fixed_hold(measured, monotonic(), event)
        self._publish_command(arm, command)

    def _on_servo_status(self, arm: str, msg: Int8) -> None:
        self._servo_status[arm] = int(msg.data)
        # Humble: -1 INVALID, 2 singularity halt, 4 collision halt, 5 joint bound.
        # 1/3 are deceleration warnings and 6 means leaving singularity.
        if int(msg.data) in {-1, 2, 4, 5}:
            self._integrator_fault(arm, f"MoveIt Servo safety halt status={int(msg.data)}")

    def _on_servo_raw(self, arm: str, msg: Float64MultiArray) -> None:
        """Consume only raw Servo output; final JGPC output remains manager-owned."""
        if (not self._servo_motion_allowed or arm != self.core.arm or
                self._manual_transition != "MANUAL_READY" or self.core.active is None):
            return
        measured = self._measured_for_arm(arm)
        if measured is None:
            self._integrator_fault(arm, "joint state unavailable while accepting Servo raw velocity")
            return
        integrator = self._integrators[arm]
        # Settling and fixed hold ignore leftover Servo joint velocity.
        if integrator.settling or not integrator.integrator_active:
            return
        error = integrator.accept_velocity(msg.data, monotonic())
        if error:
            self._integrator_fault(arm, error)

    def _integrator_fault(self, arm: str, reason: str) -> None:
        integrator = self._integrators.get(arm)
        measured = self._measured_for_arm(arm)
        if integrator is not None and measured is not None:
            hold = integrator.enter_fault_hold(measured, monotonic(), reason)
            self._publish_command(arm, hold)
        if arm == self.core.arm:
            self._manual_ready = False
            self.core.command({"type": "stop", "reason": "velocity-integrator-fault", "keep_claim": True})
            self._manual_transition = "FAULT"
            self.core.fault = reason

    def _on_joint_state(self, msg: JointState) -> None:
        self._feedback.update_joints(list(msg.name), list(msg.position))
        if self._auto_monitor:
            values = dict(zip(msg.name, msg.position))
            for arm, spec in self.config["robots"].items():
                names = [f"{spec['joint_prefix']}j{i}" for i in range(1, 7)]
                if all(name in values for name in names):
                    self._auto_monitor.note_joint_positions(arm, [values[name] for name in names])

    def _handle_enable(self, data: dict[str, Any]) -> None:
        """Acquire/release controller ownership; no nonzero output is implied."""
        requested = bool(data.get("value", False))
        if not requested:
            self._cancel_hold(self.core.arm)
            self._hold_integrator(self.core.arm, "manual-release")
            self.core.command({"type": "stop", "reason": "manual-release", "source": data.get("source", "")})
            self.core.command({"type": "enable", "value": False, "source": data.get("source", "")})
            self._manual_ready, self._manual_transition = False, "RELEASING"
            assert self._controller_client is not None
            self._controller_client.release_manual(self.core.arm, self._on_release_result)
            return
        if self._manual_transition == "FAULT":
            self._begin_fault_recovery()
            return
        if self._manual_transition not in {"AUTO", "DENIED"}:
            self.get_logger().warning(f"manual request ignored while state={self._manual_transition}")
            return
        self._request_manual_from_auto(self._on_manual_switch)

    def _request_manual_from_auto(self, callback) -> None:
        """Run the existing strict AUTO -> MANUAL request path."""
        self._capture_switch_q(self.core.arm)
        self._manual_ready, self._manual_transition = False, "REQUESTING"
        assert self._controller_client is not None
        self._controller_client.request_manual(self.core.arm, callback)

    def _begin_fault_recovery(self) -> None:
        """Return a faulted teleop pair to AUTO before requesting MANUAL again."""
        arm = self.core.arm
        self._manual_ready = False
        self._cancel_hold(arm)
        self._hold_integrator(arm, "fault-recovery")
        self.core.command({"type": "stop", "reason": "fault-recovery", "keep_claim": True})
        self._manual_transition = "RECOVERY_CHECKING_AUTO"
        assert self._controller_client is not None
        if not self._controller_client.get_controller_state(arm, self._on_fault_recovery_pair_state):
            self._fault_recovery_failed("controller_manager state query unavailable")

    def _on_fault_recovery_pair_state(self, state: str) -> None:
        """Release TELEOP only when it is confirmed active; AUTO is already safe."""
        arm = self.core.arm
        if state == "AUTO":
            self._on_fault_recovery_auto(True, "automatic controller already active")
            return
        if state != "MANUAL":
            self._fault_recovery_failed(f"controller pair is not safely recoverable: {state}")
            return
        self._manual_transition = "RECOVERY_RELEASING"
        assert self._controller_client is not None
        if not self._controller_client.release_manual(arm, self._on_fault_recovery_auto):
            self._fault_recovery_failed("controller_manager AUTO restore request unavailable")

    def _on_fault_recovery_auto(self, ok: bool, message: str) -> None:
        if not ok:
            self._fault_recovery_failed(f"controller AUTO restore failed: {message}")
            return
        # Do not clear core.fault yet: it remains visible until the normal
        # strict request and Initial Hold have succeeded.
        self._request_manual_from_auto(self._on_fault_recovery_manual_switch)

    def _on_fault_recovery_manual_switch(self, ok: bool, message: str) -> None:
        if not ok:
            self._fault_recovery_failed(f"controller MANUAL request failed: {message}")
            return
        self._on_manual_switch(ok, message)

    def _fault_recovery_failed(self, reason: str) -> None:
        self._manual_ready = False
        self._manual_transition = "FAULT"
        self.core.fault = reason

    def _on_manual_switch(self, ok: bool, message: str) -> None:
        if not ok:
            self._manual_transition = "DENIED"; self.core.fault = message; return
        status = self._feedback.status(self.core.arm, float(self.config["safety"]["robot_state_timeout_ms"]) / 1000.0)
        if status["joint_feedback"] != "ONLINE" or status["tcp_feedback"] != "ONLINE":
            self._manual_transition = "FAULT"; self.core.fault = "fresh joint state and TCP TF required"; return
        try:
            gate = self._hold_gates[self.core.arm]
            hold = gate.build_hold(status["joint_positions_rad"])
        except (KeyError, ValueError) as error:
            self._manual_transition = "FAULT"; self.core.fault = f"initial hold failed: {error}"; return
        self._manual_transition = "HOLD_PENDING"
        arm = self.core.arm
        self._hold_started[arm] = monotonic()
        self._integrators[arm].reset_to_measured(hold.data, self._hold_started[arm], "initial-hold")
        period = min(0.02, float(self.config["teleop"]["initial_hold_duration_s"]))
        self._hold_timers[arm] = self.create_timer(max(period, 0.001), lambda key=arm: self._continue_hold(key))

    def _continue_hold(self, arm: str) -> None:
        if arm != self.core.arm or self._manual_transition != "HOLD_PENDING":
            self._cancel_hold(arm)
            return
        gate = self._hold_gates[arm]
        if monotonic() - self._hold_started[arm] < gate.hold_s:
            return
        self._cancel_hold(arm)
        status = self._feedback.status(arm, float(self.config["safety"]["robot_state_timeout_ms"]) / 1000.0)
        if status["joint_feedback"] != "ONLINE":
            self._manual_transition = "FAULT"; self.core.fault = "joint state stale during initial hold"; return
        if gate.verify_measured(status["joint_positions_rad"]):
            self._manual_ready, self._manual_transition = True, "MANUAL_READY"
            self._record_switch_q_jump(arm)
            self.core.fault = ""
            self.core.command({"type": "enable", "value": True})
        else:
            self._manual_transition = "FAULT"; self.core.fault = "initial hold measured error exceeds limit"

    def _cancel_hold(self, arm: str) -> None:
        timer = self._hold_timers.pop(arm, None)
        self._hold_started.pop(arm, None)
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)

    def _on_release_result(self, ok: bool, message: str) -> None:
        self._manual_transition = "AUTO" if ok else "FAULT"
        if ok:
            self._record_switch_q_jump(self.core.arm)
        if not ok: self.core.fault = message

    def _command_tick(self) -> None:
        """Emit final Float64 position commands at the 125 Hz ros2_control cadence."""
        if self.config["teleop"].get("backend") != "servo":
            return
        arm = self.core.arm
        if self._manual_transition not in {"HOLD_PENDING", "MANUAL_READY"}:
            return
        measured = self._measured_for_arm(arm)
        if measured is None:
            self._integrator_fault(arm, "joint state stale while publishing velocity-integrator command")
            return
        # An actively held input integrates qdot. A normal stop settles the
        # existing lead; initial hold and fixed hold repeat one captured pose.
        motion_active = (
            self._servo_motion_allowed and self._manual_transition == "MANUAL_READY"
            and self.core.active is not None
        )
        try:
            command = self._integrators[arm].tick(measured, monotonic(), motion_active)
        except ValueError as error:
            self._integrator_fault(arm, f"invalid measured joint state: {error}")
            return
        if self._integrators[arm].fault_latched:
            self._integrator_fault(arm, self._integrators[arm].last_event)
            return
        self._publish_command(arm, command)

    def shutdown_safely(self) -> None:
        """Stop integration, publish a fixed hold, then attempt AUTO restore."""
        if self._shutdown_started:
            return
        self._shutdown_started = True
        self.core.command({"type": "stop", "reason": "manager-shutdown"})
        if self.config["teleop"].get("backend") != "servo":
            return
        arm = self.core.arm
        self._cancel_hold(arm)
        integrator = self._integrators.get(arm)
        measured = self._measured_for_arm(arm)
        if integrator is not None and measured is not None:
            hold = integrator.enter_fixed_hold(measured, monotonic(), "manager-shutdown")
            # A short burst gives DDS a chance to deliver the immutable hold
            # before this node disappears; it never follows later feedback.
            for _ in range(3):
                self._publish_command(arm, hold)
        else:
            self.get_logger().warning("manager shutdown: measured joint state unavailable for fixed hold")
        # A safety FAULT can still leave the teleop controller active, so it
        # must also receive the best-effort AUTO restore attempt.
        if self._manual_transition in {"HOLD_PENDING", "MANUAL_READY", "RELEASING", "FAULT"}:
            self._manual_ready = False
            self._manual_transition = "RELEASING"
            if self._controller_client is None or not self._controller_client.release_manual(arm, self._on_release_result):
                self.get_logger().warning("manager shutdown: controller AUTO restore unavailable after fixed hold")

    def _capture_switch_q(self, arm: str) -> None:
        values = self._feedback.status(arm, float(self.config["safety"]["robot_state_timeout_ms"]) / 1000.0)["joint_positions_rad"]
        if len(values) == 6:
            self._switch_q_before[arm] = dict(values)

    def _record_switch_q_jump(self, arm: str) -> None:
        before = self._switch_q_before.get(arm)
        after = self._feedback.status(arm, float(self.config["safety"]["robot_state_timeout_ms"]) / 1000.0)["joint_positions_rad"]
        if before and set(before) == set(after):
            self._last_switch_q_jump_rad[arm] = max(abs(after[name] - before[name]) for name in before)

    def _handle_gripper(self, data: dict[str, Any]) -> None:
        """Gate the existing service through the same manager ownership state."""
        if not self.core.enabled:
            raise ValueError("manual control is disabled")
        command = str(data.get("command", ""))
        self.core.command({"type": "stop", "reason": "gripper-command"})
        if not self._grippers[self.core.arm].send(command):
            self.core.fault = "gripper service unavailable"

    def _set_gripper_result(self, result: str) -> None:
        self.core.fault = "" if result == "gripper command completed" else result
        self.get_logger().info(result)

    def _update_tcp_tf(self) -> None:
        frame_key = self.core.frame
        for arm in self.core.robots:
            spec = self.config["robots"][arm]
            reference = "world" if frame_key == "world" else spec["base_frame"] if frame_key == "base" else spec["tcp_frame"]
            try:
                transform = self._tf_buffer.lookup_transform(reference, spec["tcp_frame"], rclpy.time.Time(), timeout=Duration(seconds=0.0))
                self._feedback.update_tf(arm, transform, reference)
            except TransformException:
                # TF unavailability is represented as UNKNOWN/STALE, never by a target pose.
                pass

    def _tick(self) -> None:
        self.core.tick()
        self._update_tcp_tf()
        status = self.core.status()
        timeout = float(self.config["safety"]["robot_state_timeout_ms"]) / 1000.0
        feedback = self._feedback.status(self.core.arm, timeout)
        status.update(feedback)
        status["connection"] = "MOCK" if self.core.backend.name == "mock" else feedback["joint_feedback"]
        status["manual_transition"] = self._manual_transition
        status["auto_state"] = self._auto_monitor.state(self.core.arm) if self._auto_monitor else "MOCK"
        status["manual_ready"] = self._manual_ready
        status["servo_nonzero_output_enabled"] = self._servo_motion_allowed
        status["servo_status"] = self._servo_status.get(self.core.arm, None)
        integrator = self._integrators.get(self.core.arm)
        if integrator is not None:
            status["velocity_integrator_event"] = integrator.last_event
            status["velocity_integrator_fault_latched"] = integrator.fault_latched
            status["velocity_integrator_target_rad"] = integrator.target
            status["velocity_integrator_hold_rad"] = integrator.stop_hold
            status["velocity_integrator_lead_scale"] = integrator.last_lead_scale
            status["velocity_integrator_lead_reprojection_scale"] = integrator.last_lead_reprojection_scale
            status["stop_settle_max_lead_rad"] = integrator.stop_settle_max_lead_rad
        status["first_real_test_mode"] = bool(self.config["teleop"].get("first_real_test_mode", False))
        status["last_switch_max_q_jump_rad"] = self._last_switch_q_jump_rad[self.core.arm]
        status["fault_state"] = self.core.fault or feedback["fault_state"]
        status["robot_state_source"] = "JointState + TF (measured/model transform)"
        status["tcp_pose_note"] = "TF lookup reference_frame <- selected gripper_tcp"
        out = String()
        out.data = json.dumps(status, sort_keys=True)
        self._status_pub.publish(out)


class _ServoTopicBackend:
    """Servo topic writer; never owns SDK/ServoJ and defaults to zero output."""

    name = "moveit_servo"

    def __init__(self, node: Node, config: dict[str, Any], allow_nonzero: bool) -> None:
        self._node, self._config, self._allow_nonzero = node, config, allow_nonzero
        self._publishers = {
            arm: node.create_publisher(TwistStamped, f"/fr3_teleop/{arm}/delta_twist_cmds", 20)
            for arm in config["robots"]
        }

    def _zero(self, arm: str) -> TwistStamped:
        msg = TwistStamped(); msg.header.stamp = self._node.get_clock().now().to_msg(); msg.header.frame_id = "world"
        return msg

    def start(self, motion: Motion) -> None:
        if not self._allow_nonzero:
            self.stop(motion.arm, "real-motion-disabled"); return
        spec = self._config["robots"][motion.arm]
        msg = self._zero(motion.arm)
        msg.header.frame_id = "world" if motion.frame == "world" else spec["base_frame"] if motion.frame == "base" else spec["tcp_frame"]
        if motion.axis in {"x", "y", "z"}:
            setattr(msg.twist.linear, motion.axis, motion.sign * motion.linear_mm_s / 1000.0)
        else:
            setattr(msg.twist.angular, motion.axis[1], motion.sign * motion.angular_deg_s * pi / 180.0)
        self._publishers[motion.arm].publish(msg)

    def stop(self, arm: str, reason: str) -> None:
        self._publishers[arm].publish(self._zero(arm))

    def gripper(self, arm: str, command: str) -> None:
        self.stop(arm, "gripper")


def main() -> None:
    rclpy.init()
    node = TeleopManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_safely()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
