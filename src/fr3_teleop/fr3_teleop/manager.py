"""ROS 2 manager. The only process allowed to call a motion backend."""

from __future__ import annotations

import json
from math import isfinite, pi
from pathlib import Path
from time import monotonic
from typing import Any

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64, Float64MultiArray, Int8, Int32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException
import yaml

from .core import ControlCore, MockBackend, Motion, SourceRejected
from .manual_trace import from_environment, record as trace_record
from .controller_manager_client import AutoTrajectoryMonitor, ControllerManagerClient
from .gripper_bridge import GripperBridgeClient
from .initial_hold import InitialHoldGate
from .robot_state import FeedbackCache
from .servo_position_accumulator import ServoVelocityIntegrator, VelocityIntegratorConfig
from .stage1_cartesian import (
    is_stage1_motion,
    manual_namespace,
    stage1_configured,
)


def _stage1_on(node: Any) -> bool:
    """Harnesses that call manager methods directly do not construct Stage 1."""
    return bool(getattr(node, "_stage1", False))


def _stage1_servo(node: Any) -> bool:
    if not _stage1_on(node):
        return False
    config = getattr(node, "config", None)
    teleop = config.get("teleop", {}) if isinstance(config, dict) else {}
    return teleop.get("backend") == "servo"


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
        self._manual_trace = from_environment()
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
        self._stage1 = stage1_configured(config)
        self._stage1_pub = None
        self._enter_client = None
        self._exit_client = None
        self._hardware_mode = ""
        self._exit_zero_timer = None
        if backend_name == "servo" and self._stage1:
            backend = _Stage1CommandBackend()
        elif backend_name == "servo":
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
            self._controller_client = ControllerManagerClient(
                self, config["robots"], self._auto_monitor, cartesian_arm_a=self._stage1)
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
        if self._stage1:
            manual_ns = manual_namespace(str(config["robots"]["arm_a"]["gripper_service"]))
            self._stage1_pub = self.create_publisher(Float64, f"{manual_ns}/base_x_velocity_mm_s", 10)
            self._enter_client = self.create_client(Trigger, f"{manual_ns}/enter")
            self._exit_client = self.create_client(Trigger, f"{manual_ns}/exit")
            latched = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(String, f"{manual_ns}/runtime_mode", self._on_hardware_mode, latched)
            self.create_subscription(Int32, f"{manual_ns}/last_error", self._on_stage1_error, latched)
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
        trace_record(self, "command_received", raw=msg.data)
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError("JSON command must be an object")
            # Stop the integrator before ControlCore emits its zero Twist.
            # A normal stop keeps the last position command and settles; it
            # must not be overwritten by a later raw velocity sample.
            if data.get("type") == "select_arm" and _stage1_servo(self) and self._manual_transition not in {"AUTO", "MOCK", "DENIED"}:
                raise ValueError("disable Arm A manual before selecting another arm")
            if data.get("type") in {"stop", "release", "select_arm", "set_frame", "set_speed"}:
                if _stage1_servo(self) and self.core.arm == "arm_a":
                    if data.get("type") in {"stop", "release"}:
                        # Only operator-stop bypasses ownership. Validate before
                        # publishing so an ignored foreign release cannot zero motion.
                        if not (data.get("type") == "stop" and
                                str(data.get("reason", "operator-stop")) == "operator-stop"):
                            self.core._reject_if_foreign(data)
                        self.core.command(data)
                        self._publish_stage1_velocity(0.0)
                        return
                else:
                    self._hold_integrator(self.core.arm, str(data.get("type")))
            if data.get("type") == "enable" and self.config["teleop"].get("backend") == "servo":
                self._handle_enable(data)
            elif data.get("type") == "motion" and _stage1_servo(self):
                self._handle_stage1_motion(data)
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
            trace_record(self, "source_rejected", reason=str(exc))
            self.get_logger().warning(f"teleop source rejected: {exc}")
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            if _stage1_servo(self):
                self._publish_stage1_velocity(0.0)
            self.core._stop("invalid-command")
            self.core.fault = str(exc)
            self.get_logger().error(f"teleop command rejected: {exc}")
        finally:
            trace_record(self, "command_finished", active=repr(self.core.active),
                         source=getattr(self.core, "active_input_source", None),
                         stop_reason=getattr(self.core, "last_stop_reason", None),
                         fault=self.core.fault)

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
        if _stage1_on(self) and arm == "arm_a":
            return
        self._servo_status[arm] = int(msg.data)
        # Humble: -1 INVALID, 2 singularity halt, 4 collision halt, 5 joint bound.
        # 1/3 are deceleration warnings and 6 means leaving singularity.
        if int(msg.data) in {-1, 2, 4, 5}:
            self._integrator_fault(arm, f"MoveIt Servo safety halt status={int(msg.data)}")

    def _on_servo_raw(self, arm: str, msg: Float64MultiArray) -> None:
        """Consume only raw Servo output; final JGPC output remains manager-owned."""
        if _stage1_on(self) and arm == "arm_a":
            return
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
        if _stage1_on(self):
            self._handle_stage1_enable(requested, data)
            return
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
        if stage1_configured(getattr(self, "config", None)) and getattr(self.core, "arm", "") == "arm_a":
            self._recover_stage1(state)
            return
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
        if _stage1_servo(self):
            self._on_stage1_controllers_off(True, message)
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
        if _stage1_on(self) and arm == "arm_a":
            return
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
        if _stage1_on(self) and self.core.arm == "arm_a" and self._manual_transition in {
                "HOLD_PENDING", "MANUAL_READY", "RELEASING", "FAULT", "REQUESTING"}:
            self._shutdown_stage1()
            return
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
        self._refresh_stage1_velocity()
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
        if _stage1_on(self):
            status["hardware_runtime_mode"] = self._hardware_mode
            status["stage1_velocity_mm_s"] = self.core.active.linear_mm_s if self.core.active else 0.0
        out = String()
        out.data = json.dumps(status, sort_keys=True)
        self._status_pub.publish(out)

    def _publish_stage1_velocity(self, velocity_mm_s: float) -> None:
        publisher = getattr(self, "_stage1_pub", None)
        if publisher is None:
            return
        publisher.publish(Float64(data=float(velocity_mm_s)))
        trace_record(self, "velocity_published", velocity_mm_s=float(velocity_mm_s),
                     source=getattr(self.core, "active_input_source", None),
                     stop_reason=getattr(self.core, "last_stop_reason", None),
                     active=repr(self.core.active))

    def _refresh_stage1_velocity(self) -> None:
        # Nonzero velocity is published only when a motion command arrives.
        # Repeating it here would keep the hardware watchdog fresh after the
        # command source has already died.
        if not _stage1_on(self) or self.core.arm != "arm_a" or self._manual_transition != "MANUAL_READY":
            return
        motion = self.core.active
        if (self._servo_motion_allowed and motion is not None and
                is_stage1_motion(motion.arm, motion.frame, motion.axis, motion.sign)):
            return
        self._publish_stage1_velocity(0.0)

    def _handle_stage1_motion(self, data: dict[str, Any]) -> None:
        if not self._manual_ready or self._manual_transition != "MANUAL_READY":
            self.get_logger().warning("motion ignored: manual control is not ready")
            return
        if not is_stage1_motion(self.core.arm, self.core.frame, str(data.get("axis", "")), int(data.get("sign", 0))):
            raise ValueError("Stage 1 only permits Arm A / base / X+")
        if not self._servo_motion_allowed:
            raise ValueError("allow_real_motion is false")
        self.core.command(data)
        velocity = self.core.active.linear_mm_s
        if not isfinite(velocity) or velocity <= 0.0:
            raise ValueError("Stage 1 velocity must be finite and positive")
        self._publish_stage1_velocity(velocity)

    def _handle_stage1_enable(self, requested: bool, data: dict[str, Any]) -> None:
        if self.core.arm != "arm_a":
            self._manual_ready = False
            self._manual_transition = "DENIED"
            self.core.fault = "Stage 1 manual is Arm A only"
            return
        if not requested:
            self._begin_stage1_exit("manual-release")
            return
        if self._manual_transition == "FAULT":
            self._begin_fault_recovery()
            return
        if self._manual_transition not in {"AUTO", "DENIED"}:
            self.get_logger().warning(f"manual request ignored while state={self._manual_transition}")
            return
        status = self._feedback.status(self.core.arm, float(self.config["safety"]["robot_state_timeout_ms"]) / 1000.0)
        if status["joint_feedback"] != "ONLINE":
            self._manual_transition = "DENIED"
            self.core.fault = "fresh joint state required before manual cartesian"
            return
        self._manual_ready = False
        self._manual_transition = "REQUESTING"
        assert self._controller_client is not None
        self._controller_client.request_manual("arm_a", self._on_stage1_controllers_off)

    def _on_stage1_controllers_off(self, ok: bool, message: str) -> None:
        if not ok:
            self._manual_transition = "DENIED"
            self.core.fault = message
            return
        if self._enter_client is None or not self._enter_client.service_is_ready():
            self._restore_jtc_after_failed_enter("manual enter service unavailable")
            return
        future = self._enter_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_stage1_entered)

    def _on_stage1_entered(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._restore_jtc_after_failed_enter(f"manual enter failed: {exc}")
            return
        message = "" if response is None else str(response.message)
        if response is None or not response.success:
            if message.startswith("SERVO_ACTIVE"):
                self._restore_jtc_after_failed_enter(message or "manual enter failed")
            else:
                self._manual_ready = False
                self._manual_transition = "FAULT"
                self.core.fault = message or "manual enter failed; JTC left inactive"
            return
        self._hardware_mode = "MANUAL_CARTESIAN"
        self._manual_ready = True
        self._manual_transition = "MANUAL_READY"
        self.core.fault = ""
        self.core.command({"type": "enable", "value": True})
        self._publish_stage1_velocity(0.0)

    def _restore_jtc_after_failed_enter(self, message: str) -> None:
        self._manual_ready = False
        self._manual_transition = "RELEASING"
        self.core.fault = message
        assert self._controller_client is not None
        if not self._controller_client.release_manual("arm_a", self._on_failed_enter_jtc):
            self._manual_transition = "FAULT"
            self.core.fault = f"{message}; JTC restore unavailable"

    def _on_failed_enter_jtc(self, ok: bool, message: str) -> None:
        if ok:
            self._hardware_mode = "SERVO_ACTIVE"
            self._manual_transition = "AUTO"
            return
        self._manual_transition = "FAULT"
        self.core.fault = f"{self.core.fault}; JTC restore failed: {message}"

    def _begin_stage1_exit(self, reason: str) -> None:
        self._manual_ready = False
        self.core.command({"type": "stop", "reason": reason})
        self.core.command({"type": "enable", "value": False})
        self._manual_transition = "RELEASING"
        self._publish_stage1_velocity(0.0)
        if self._exit_zero_timer is not None:
            return
        self._exit_zero_timer = self.create_timer(0.02, self._stage1_exit_after_zero)

    def _stage1_exit_after_zero(self) -> None:
        timer = self._exit_zero_timer
        self._exit_zero_timer = None
        if timer is None:
            return
        timer.cancel()
        self.destroy_timer(timer)
        if self._exit_client is None or not self._exit_client.service_is_ready():
            self._manual_transition = "FAULT"
            self.core.fault = "manual exit service unavailable; JTC was not activated"
            return
        future = self._exit_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_stage1_hardware_exited)

    def _on_stage1_hardware_exited(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._manual_transition = "FAULT"
            self.core.fault = f"manual exit failed: {exc}; JTC was not activated"
            return
        if response is None or not response.success:
            self._manual_transition = "FAULT"
            self.core.fault = ("" if response is None else str(response.message)) or "manual exit failed; JTC was not activated"
            return
        self._hardware_mode = "SERVO_ACTIVE"
        assert self._controller_client is not None
        if not self._controller_client.release_manual("arm_a", self._on_release_result):
            self._manual_transition = "FAULT"
            self.core.fault = "JTC activate unavailable after ServoJ restore"

    def _recover_stage1(self, state: str) -> None:
        if state == "FAULT":
            self._fault_recovery_failed(f"controller pair is not safely recoverable: {state}")
            return
        self._publish_stage1_velocity(0.0)
        mode = getattr(self, "_hardware_mode", "")
        if mode == "":
            self._fault_recovery_failed("hardware runtime mode unknown; JTC was not activated")
            return
        if mode == "MANUAL_CARTESIAN":
            if self._exit_client is None or not self._exit_client.service_is_ready():
                self._fault_recovery_failed("manual exit service unavailable during fault recovery")
                return
            future = self._exit_client.call_async(Trigger.Request())
            future.add_done_callback(lambda done, pair_state=state: self._after_stage1_fault_exit(done, pair_state))
            return
        if mode != "SERVO_ACTIVE":
            self._fault_recovery_failed(f"hardware mode {mode} is not ServoJ; JTC was not activated")
            return
        if state == "AUTO":
            self._on_fault_recovery_auto(True, "automatic controller already active")
            return
        self._manual_transition = "RECOVERY_RELEASING"
        assert self._controller_client is not None
        if not self._controller_client.release_manual(self.core.arm, self._on_fault_recovery_auto):
            self._fault_recovery_failed("controller_manager AUTO restore request unavailable")

    def _after_stage1_fault_exit(self, future, state: str) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._fault_recovery_failed(f"manual exit failed during fault recovery: {exc}")
            return
        if response is None or not response.success:
            self._fault_recovery_failed(
                ("" if response is None else str(response.message)) or "manual exit failed during fault recovery")
            return
        self._hardware_mode = "SERVO_ACTIVE"
        self._recover_stage1(state)

    def _on_hardware_mode(self, msg: String) -> None:
        self._hardware_mode = str(msg.data)

    def _on_stage1_error(self, msg: Int32) -> None:
        code = int(msg.data)
        if code == 0 or self._manual_transition in {"RELEASING", "AUTO", "MOCK"}:
            return
        self._publish_stage1_velocity(0.0)
        self._manual_ready = False
        self.core.command({"type": "stop", "reason": "servocart-error", "keep_claim": True})
        self._manual_transition = "FAULT"
        self.core.fault = f"ServoCart error {code}"

    def _shutdown_stage1(self) -> None:
        self._manual_ready = False
        self._manual_transition = "RELEASING"
        self._publish_stage1_velocity(0.0)
        if self._exit_client is None or not self._exit_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("manager shutdown: manual exit service unavailable; JTC was not activated")
            return
        future = self._exit_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None or not response.success:
            self.get_logger().error("manager shutdown: manual exit failed; JTC was not activated")
            return
        self._hardware_mode = "SERVO_ACTIVE"
        done: list[tuple[bool, str]] = []
        if self._controller_client is None or not self._controller_client.release_manual(
                "arm_a", lambda ok, message: done.append((ok, message))):
            self.get_logger().error("manager shutdown: JTC activate unavailable after ServoJ restore")
            return
        deadline = monotonic() + 3.0
        while not done and monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not done or not done[0][0]:
            self.get_logger().error("manager shutdown: JTC activate failed after ServoJ restore")


class _Stage1CommandBackend:
    """Records manual motion without publishing MoveIt Servo twists or joint targets."""

    name = "servocart"

    def start(self, motion: Motion) -> None:
        del motion

    def stop(self, arm: str, reason: str) -> None:
        del arm, reason

    def gripper(self, arm: str, command: str) -> None:
        del arm, command


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
        if node._manual_trace is not None:
            node._manual_trace.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
