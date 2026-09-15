"""
Gripper backends used by task nodes.

Task code should only call open / close / move. Swap the backend when
moving from Gazebo to a real FAIRINO gripper; do not change pick logic.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
import os
import time
from typing import Any, Sequence

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import GripperCommand
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState
import yaml

from fr_control.constants import (
    GRIPPER_CLOSED,
    GRIPPER_LEFT_ACTION,
    GRIPPER_LEFT_JOINT,
    GRIPPER_OPEN,
    GRIPPER_RIGHT_ACTION,
    GRIPPER_RIGHT_JOINT,
)
from fairino_msgs.srv import GripperBridge

_GAZEBO_KWARGS = {
    "left_action",
    "right_action",
    "open_position",
    "closed_position",
    "max_effort",
    "wait_timeout_sec",
}


class GripperError(RuntimeError):
    """Raised when a gripper command is rejected or does not finish."""


class GripperInterface(ABC):
    """Task-layer gripper API. Backends implement this, tasks do not."""

    def activate(self, timeout_sec: float = 5.0) -> None:
        """Activate the gripper. Default is a no-op for simulation."""
        del timeout_sec

    def reset(self, timeout_sec: float = 5.0) -> None:
        """Explicit reset. Default is a no-op; never call this from create()."""
        del timeout_sec

    def check_connection(self) -> None:
        """Verify the backend is reachable without commanding motion."""

    def describe(self) -> dict[str, Any]:
        """Return a JSON-serialisable summary of this backend."""
        return {"backend": type(self).__name__}

    def get_state(self) -> dict[str, Any]:
        """Optional feedback. Current is not clamp force in newtons."""
        return {}

    @abstractmethod
    def open(self, timeout_sec: float = 5.0) -> None:
        """Fully open the gripper."""

    @abstractmethod
    def close(self, timeout_sec: float = 5.0) -> None:
        """Fully close the gripper."""

    @abstractmethod
    def move(self, position: float, timeout_sec: float = 5.0) -> None:
        """Move to a position in the backend's native units."""


class _FingerClient:
    """One effort GripperActionController client for a single finger."""

    def __init__(
        self,
        node: Node,
        action_name: str,
        wait_timeout_sec: float,
    ) -> None:
        """Connect to one already running gripper action server."""
        self.name = action_name
        self._node = node
        self._client = ActionClient(node, GripperCommand, action_name)
        node.get_logger().info(f"等待夹爪 action：{action_name}")
        if not self._client.wait_for_server(timeout_sec=wait_timeout_sec):
            raise GripperError(f"夹爪 action 未就绪：{action_name}")

    def send_goal(self, position: float, max_effort: float) -> Future:
        """Send a GripperCommand goal without waiting for the result."""
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
        return self._client.send_goal_async(goal)

    def wait_result(
        self, send_future: Future, timeout_sec: float
    ) -> GripperCommand.Result:
        """Wait until a previously sent goal finishes."""
        rclpy.spin_until_future_complete(
            self._node, send_future, timeout_sec=timeout_sec
        )
        if not send_future.done():
            raise GripperError(f"发送夹爪目标超时：{self.name}")
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise GripperError(f"夹爪控制器拒绝目标：{self.name}")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self._node, result_future, timeout_sec=timeout_sec
        )
        if not result_future.done():
            raise GripperError(f"等待夹爪结果超时：{self.name}")
        wrapped = result_future.result()
        if wrapped is None or wrapped.result is None:
            raise GripperError(f"夹爪没有返回结果：{self.name}")
        return wrapped.result


class GazeboGripper(GripperInterface):
    """
    Gazebo dual-finger gripper via two effort GripperActionControllers.

    User-facing position is still close-travel: 0.0 open, 0.1 fully closed.
    Left slider target = +0.5 * travel, right slider target = -0.5 * travel.
    Each controller runs position PID and writes a limited effort.
    """

    def __init__(
        self,
        node: Node,
        *,
        left_action: str = GRIPPER_LEFT_ACTION,
        right_action: str = GRIPPER_RIGHT_ACTION,
        open_position: float = GRIPPER_OPEN,
        closed_position: float = GRIPPER_CLOSED,
        max_effort: float = 25.0,
        wait_timeout_sec: float = 10.0,
    ) -> None:
        """Connect to the already running left and right action servers."""
        self._node = node
        self._open_position = float(open_position)
        self._closed_position = float(closed_position)
        self._max_effort = float(max_effort)
        self._left = _FingerClient(node, left_action, wait_timeout_sec)
        self._right = _FingerClient(node, right_action, wait_timeout_sec)
        self._joint_state: JointState | None = None
        node.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )

    def describe(self) -> dict[str, Any]:
        """Return Gazebo close-travel parameters."""
        return {
            "backend": "gazebo",
            "open_position": self._open_position,
            "closed_position": self._closed_position,
            "conservative_position": 0.5
            * (self._open_position + self._closed_position),
            "left_action": self._left.name,
            "right_action": self._right.name,
        }

    def check_connection(self) -> None:
        """Action servers were already required during construction."""
        self._node.get_logger().info(
            "Gazebo 夹爪 action 已就绪："
            f"{self._left.name} {self._right.name}"
        )

    def open(self, timeout_sec: float = 5.0) -> None:
        """Fully open the gripper."""
        self.move(self._open_position, timeout_sec=timeout_sec)

    def close(self, timeout_sec: float = 5.0) -> None:
        """Fully close the gripper."""
        self.move(self._closed_position, timeout_sec=timeout_sec)

    def move(
        self,
        position: float,
        timeout_sec: float = 5.0,
        **_unused,
    ) -> None:
        """Command both fingers to a symmetric close-travel position."""
        travel = min(
            self._closed_position,
            max(self._open_position, float(position)),
        )
        left_target = 0.5 * travel
        right_target = -0.5 * travel
        self._node.get_logger().info(
            f"夹爪目标 travel={travel:.4f} "
            f"left={left_target:.4f} right={right_target:.4f}"
        )
        left_future = self._left.send_goal(left_target, self._max_effort)
        right_future = self._right.send_goal(right_target, self._max_effort)
        # 慢速仿真里 action 结果经常滞后于真实关节。发送目标后轮询 joint_states。
        left_pos, right_pos, left_vel, right_vel = self._wait_joints(
            left_target, right_target, timeout_sec
        )
        left = self._try_result(self._left, left_future, 0.5, left_pos)
        right = self._try_result(self._right, right_future, 0.5, right_pos)
        self._node.get_logger().info(
            "夹爪完成 "
            f"left cmd={left_target:.4f} actual={left_pos:.4f} "
            f"vel={left_vel:.4f} reached={left.reached_goal} "
            f"stalled={left.stalled} "
            f"right cmd={right_target:.4f} actual={right_pos:.4f} "
            f"vel={right_vel:.4f} reached={right.reached_goal} "
            f"stalled={right.stalled}"
        )

    def _wait_joints(
        self,
        left_target: float,
        right_target: float,
        timeout_sec: float,
    ) -> tuple[float, float, float, float]:
        """Wait until both fingers stall or stay near the commanded pose."""
        deadline = time.monotonic() + max(timeout_sec, 20.0)
        last = (0.0, 0.0, 0.0, 0.0)
        slow_since = None
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.05)
            last = self._finger_state()
            left_pos, right_pos, left_vel, right_vel = last
            left_err = abs(left_pos - left_target)
            right_err = abs(right_pos - right_target)
            slow = abs(left_vel) < 0.005 and abs(right_vel) < 0.005
            if left_err < 0.003 and right_err < 0.003:
                return last
            now = self._node.get_clock().now()
            if slow:
                if slow_since is None:
                    slow_since = now
                elif (now - slow_since).nanoseconds / 1e9 >= 0.25:
                    return last
            else:
                slow_since = None
        left_pos, right_pos, left_vel, right_vel = last
        left_err = abs(left_pos - left_target)
        right_err = abs(right_pos - right_target)
        if left_err > 0.004 and right_err > 0.004:
            self._node.get_logger().warn(
                "夹爪在超时前未到达目标，但两侧都被挡住，视为接触 stall："
                f" left={left_pos:.4f}/{left_target:.4f}"
                f" right={right_pos:.4f}/{right_target:.4f}"
            )
            return last
        raise GripperError(
            "夹爪关节未在超时内到位或 stall："
            f" left={last[0]:.4f}/{left_target:.4f}"
            f" right={last[1]:.4f}/{right_target:.4f}"
        )

    def _finger_state(self) -> tuple[float, float, float, float]:
        """Return latest left/right position and velocity."""
        if self._joint_state is None:
            return 0.0, 0.0, 0.0, 0.0
        names = list(self._joint_state.name)
        positions = list(self._joint_state.position)
        velocities = list(self._joint_state.velocity or [])

        def read(joint: str) -> tuple[float, float]:
            if joint not in names:
                return 0.0, 0.0
            index = names.index(joint)
            vel = float(velocities[index]) if index < len(velocities) else 0.0
            return float(positions[index]), vel

        left_pos, left_vel = read(GRIPPER_LEFT_JOINT)
        right_pos, right_vel = read(GRIPPER_RIGHT_JOINT)
        return left_pos, right_pos, left_vel, right_vel

    def _try_result(
        self,
        client: _FingerClient,
        send_future: Future,
        timeout_sec: float,
        actual: float,
    ) -> GripperCommand.Result:
        """Best-effort action result; synthesize stall/reached from joint state."""
        try:
            return client.wait_result(send_future, timeout_sec)
        except GripperError:
            result = GripperCommand.Result()
            result.position = actual
            result.reached_goal = False
            result.stalled = True
            return result

    def _on_joint_state(self, msg: JointState) -> None:
        """Cache joint states for polling."""
        self._joint_state = msg


class RealFairinoGripper(GripperInterface):
    """
    Real HKV TG-9801 via FAIRINO controller Lua, not via FR3 joints.

    Commands go through /fairino_gripper/command, which FairinoHardwareInterface
    executes on the same FRRobot already used by ServoJ. This object never
    opens XML-RPC 20003 and does not start ros2_cmd_server. Creating it does
    not reset, activate, or move the gripper.
    """

    def __init__(
        self,
        node: Node,
        *,
        dry_run: bool = False,
        config_file: str = "",
        **overrides: Any,
    ) -> None:
        """Load YAML parameters. Does not call the hardware service yet."""
        self._node = node
        self._dry_run = bool(dry_run)
        cfg = load_real_gripper_config(node, config_file)
        allowed = set(cfg.keys())
        cfg.update(
            {
                key: value
                for key, value in overrides.items()
                if key in allowed and value is not None
            }
        )
        self._cfg = cfg
        self._client = node.create_client(
            GripperBridge, str(cfg["service_name"])
        )
        node.get_logger().info(
            "RealFairinoGripper 已创建（尚未发令，不会自动 reset/activate/运动）"
            f" id={cfg['id']} service={cfg['service_name']}"
            f" open={cfg['open_position']} close={cfg['close_position']}"
            f" vel={cfg['velocity']} force={cfg['force']}"
            f" dry_run={self._dry_run}"
        )

    def describe(self) -> dict[str, Any]:
        """Return the loaded real-gripper parameters."""
        data = dict(self._cfg)
        data["backend"] = "real"
        data["dry_run"] = self._dry_run
        data["service_name"] = self._cfg["service_name"]
        data["auto_reset"] = False
        data["rpc_direct"] = False
        return data

    def check_connection(self) -> None:
        """Wait for the hardware gripper service and ping it. Does not Act/Move."""
        name = str(self._cfg["service_name"])
        timeout = float(self._cfg["service_timeout_sec"])
        self._node.get_logger().info(
            f"等待夹爪桥接服务 {name}（ping，不发送 Act/Move）"
        )
        if not self._client.wait_for_service(timeout_sec=timeout):
            raise GripperError(
                f"未找到夹爪服务 {name}。请重启已加载夹爪桥接的 real_bringup，"
                "不要另开 XML-RPC 20003。"
            )
        result = self._call("ping")
        self._node.get_logger().info(
            f"夹爪桥接探测成功：{result.message} code={result.error_code}"
        )

    def activate(self, timeout_sec: float = 5.0) -> None:
        """Explicit ActGripper(id, 1). Not called from create_gripper()."""
        self._act("activate", timeout_sec)

    def reset(self, timeout_sec: float = 5.0) -> None:
        """Explicit ActGripper(id, 0). Never run automatically."""
        self._act("reset", timeout_sec)

    def open(self, timeout_sec: float = 5.0) -> None:
        """Move to the configured open position."""
        self.move(int(self._cfg["open_position"]), timeout_sec=timeout_sec)

    def close(self, timeout_sec: float = 5.0) -> None:
        """Move to the configured close position."""
        self.move(int(self._cfg["close_position"]), timeout_sec=timeout_sec)

    def move(
        self,
        position: float,
        timeout_sec: float = 5.0,
        velocity: int | None = None,
        force: int | None = None,
        **_unused,
    ) -> None:
        """MoveGripper to a 0-100 position using YAML vel/force."""
        pos = int(round(float(position)))
        command = {
            "method": "MoveGripper",
            "service": str(self._cfg["service_name"]),
            "id": int(self._cfg["id"]),
            "pos": pos,
            "vel": int(self._cfg["velocity"] if velocity is None else velocity),
            "force": int(self._cfg["force"] if force is None else force),
            "max_time_ms": int(self._cfg["max_time_ms"]),
            "block": int(self._cfg["block"]),
            "type": int(self._cfg["type"]),
            "rot_num": float(self._cfg["rot_num"]),
            "rot_vel": int(self._cfg["rot_vel"]),
            "rot_torque": int(self._cfg["rot_torque"]),
        }
        if self._dry_run:
            self._node.get_logger().info(f"dry_run 跳过运动：{command}")
            return
        self._node.get_logger().info(f"发送 {command}")
        result = self._call(
            "move",
            position=command["pos"],
            velocity=command["vel"],
            force=command["force"],
            max_time_ms=command["max_time_ms"],
            block=command["block"],
            gripper_type=command["type"],
            rot_num=command["rot_num"],
            rot_vel=command["rot_vel"],
            rot_torque=command["rot_torque"],
        )
        self._node.get_logger().info(
            f"MoveGripper 返回码={result.error_code} {result.message}"
        )
        self._wait_motion(timeout_sec)

    def get_state(self) -> dict[str, Any]:
        """Service reachability only. Do not poll GetGripperMotionDone on 20003."""
        try:
            result = self._call("ping")
        except GripperError as exc:
            self._node.get_logger().warn(f"get_state 不可用：{exc}")
            return {"error": str(exc)}
        return {
            "service": str(self._cfg["service_name"]),
            "ping_code": result.error_code,
            "message": result.message,
            "note": "fault/current 百分比不是夹持力 N；不在 20003 上轮询 motion done",
        }

    def _act(self, name: str, timeout_sec: float) -> None:
        """Send ActGripper via the hardware bridge. name is activate or reset."""
        command = {
            "method": "ActGripper",
            "service": str(self._cfg["service_name"]),
            "id": int(self._cfg["id"]),
            "name": name,
        }
        if self._dry_run:
            self._node.get_logger().info(f"dry_run 跳过 {name}：{command}")
            return
        self._node.get_logger().info(f"发送 {command}")
        result = self._call(name)
        self._node.get_logger().info(
            f"ActGripper({name}) 返回码={result.error_code} {result.message}"
        )
        if timeout_sec > 0.0:
            time.sleep(min(timeout_sec, 2.0))

    def _wait_motion(self, timeout_sec: float) -> None:
        """Hardware service already waited for GetGripperMotionDone and ServoJ resume."""
        del timeout_sec
        self._node.get_logger().info(
            "夹爪服务已返回：硬件确认动作完成且 ServoJ 已恢复"
        )

    def _call(self, command: str, **fields: Any) -> GripperBridge.Response:
        """Send one gripper command through the hardware-interface service."""
        timeout = max(
            float(self._cfg["service_timeout_sec"]),
            float(fields.get("max_time_ms", self._cfg["max_time_ms"])) / 1000.0
            + 5.0,
        )
        if not self._client.wait_for_service(timeout_sec=timeout):
            raise GripperError(
                f"未找到夹爪服务 {self._cfg['service_name']}。"
                "请重启已加载夹爪桥接的 real_bringup。"
            )
        request = GripperBridge.Request()
        request.command = str(command)
        request.gripper_id = int(self._cfg["id"])
        request.position = int(fields.get("position", 0))
        request.velocity = int(fields.get("velocity", self._cfg["velocity"]))
        request.force = int(fields.get("force", self._cfg["force"]))
        request.max_time_ms = int(
            fields.get("max_time_ms", self._cfg["max_time_ms"])
        )
        request.block = int(fields.get("block", self._cfg["block"]))
        request.gripper_type = int(fields.get("gripper_type", self._cfg["type"]))
        request.rot_num = float(fields.get("rot_num", self._cfg["rot_num"]))
        request.rot_vel = int(fields.get("rot_vel", self._cfg["rot_vel"]))
        request.rot_torque = int(
            fields.get("rot_torque", self._cfg["rot_torque"])
        )
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=timeout
        )
        if not future.done():
            raise GripperError(
                f"夹爪服务调用超时：{command} {self._cfg['service_name']}"
            )
        result = future.result()
        if result is None:
            raise GripperError(f"夹爪服务无响应：{command}")
        if int(result.error_code) != 0:
            raise GripperError(
                f"{command} 失败，返回码={result.error_code} {result.message}"
            )
        return result


def load_real_gripper_config(node: Node, config_file: str = "") -> dict[str, Any]:
    """Load gripper.yaml. ROS param gripper_config_file can override the path."""
    path = str(config_file or "").strip()
    if not path:
        path = _node_string_param(node, "gripper_config_file", "")
    if not path:
        path = os.path.join(
            get_package_share_directory("fr_control"),
            "config",
            "gripper.yaml",
        )
    with open(path, encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    block = raw.get("gripper", raw)
    real = dict(block.get("real", {}))
    cfg = {
        "id": int(block.get("id", 1)),
        "controller_ip": str(real.get("controller_ip", "192.168.58.2")),
        "service_name": str(
            real.get("service_name", "/fairino_gripper/command")
        ),
        "service_timeout_sec": float(real.get("service_timeout_sec", 15.0)),
        "open_position": int(real.get("open_position", 0)),
        "close_position": int(real.get("close_position", 80)),
        "velocity": int(real.get("velocity", 20)),
        "force": int(real.get("force", 20)),
        "conservative_position": int(real.get("conservative_position", 20)),
        "conservative_velocity": int(real.get("conservative_velocity", 10)),
        "conservative_force": int(real.get("conservative_force", 15)),
        "max_time_ms": int(real.get("max_time_ms", 5000)),
        "block": int(real.get("block", 1)),
        "type": int(real.get("type", 0)),
        "rot_num": float(real.get("rot_num", 0.0)),
        "rot_vel": int(real.get("rot_vel", 0)),
        "rot_torque": int(real.get("rot_torque", 0)),
        "config_file": path,
    }
    node.get_logger().info(f"已加载夹爪配置：{path}")
    return cfg


def _node_string_param(node: Node, name: str, default: str) -> str:
    """Read a string ROS parameter if the node already declared it."""
    if not node.has_parameter(name):
        return default
    return str(node.get_parameter(name).get_parameter_value().string_value)


def create_gripper(
    node: Node,
    backend: str = "gazebo",
    **kwargs,
) -> GripperInterface:
    """Build a gripper backend. Task nodes should call this factory."""
    name = str(backend).strip().lower()
    dry_run = bool(kwargs.pop("dry_run", False))
    config_file = str(kwargs.pop("config_file", "") or "")
    if name in ("gazebo", "sim", "simulation"):
        gazebo_kwargs = {
            key: value
            for key, value in kwargs.items()
            if key in _GAZEBO_KWARGS
        }
        gripper = GazeboGripper(node, **gazebo_kwargs)
        if dry_run:
            node.get_logger().info("gazebo backend 已创建；dry_run 由调用方跳过运动")
        return gripper
    if name in ("real", "fairino", "hardware"):
        return RealFairinoGripper(
            node,
            dry_run=dry_run,
            config_file=config_file,
            **kwargs,
        )
    raise ValueError(
        f"未知夹爪 backend：{backend}。可选 gazebo 或 real"
    )


def split_travel(travel: float) -> tuple[float, float]:
    """Map close-travel to left / right slider targets."""
    value = float(travel)
    return 0.5 * value, -0.5 * value


def joint_snapshot(
    names: Sequence[str],
    positions: Sequence[float],
    velocities: Sequence[float] | None = None,
    efforts: Sequence[float] | None = None,
) -> dict[str, dict[str, float]]:
    """Build a name -> {position, velocity, effort} map from JointState fields."""
    snapshot: dict[str, dict[str, float]] = {}
    for index, name in enumerate(names):
        item = {"position": float(positions[index])}
        if velocities is not None and index < len(velocities):
            item["velocity"] = float(velocities[index])
        if efforts is not None and index < len(efforts):
            item["effort"] = float(efforts[index])
        snapshot[name] = item
    return snapshot
