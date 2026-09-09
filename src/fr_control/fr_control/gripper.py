"""
Gripper backends used by task nodes.

Task code should only call open / close / move. Swap the backend when
moving from Gazebo to a real FAIRINO gripper; do not change pick logic.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
import time
from typing import Sequence

from control_msgs.action import GripperCommand
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState

from fr_control.constants import (
    GRIPPER_CLOSED,
    GRIPPER_LEFT_ACTION,
    GRIPPER_LEFT_JOINT,
    GRIPPER_OPEN,
    GRIPPER_RIGHT_ACTION,
    GRIPPER_RIGHT_JOINT,
)


class GripperError(RuntimeError):
    """Raised when a gripper command is rejected or does not finish."""


class GripperInterface(ABC):
    """Task-layer gripper API. Backends implement this, tasks do not."""

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

    def open(self, timeout_sec: float = 5.0) -> None:
        """Fully open the gripper."""
        self.move(self._open_position, timeout_sec=timeout_sec)

    def close(self, timeout_sec: float = 5.0) -> None:
        """Fully close the gripper."""
        self.move(self._closed_position, timeout_sec=timeout_sec)

    def move(self, position: float, timeout_sec: float = 5.0) -> None:
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
    Placeholder for the real FAIRINO gripper (Lua / RS485).

    Stage 2 only implements GazeboGripper. Keep this class so later
    hardware code can drop in without changing task nodes.
    """

    def __init__(self, node: Node, **_kwargs) -> None:
        """Record the node; methods raise until the real driver exists."""
        self._node = node

    def open(self, timeout_sec: float = 5.0) -> None:
        """Not implemented for real hardware yet."""
        raise NotImplementedError(self._message())

    def close(self, timeout_sec: float = 5.0) -> None:
        """Not implemented for real hardware yet."""
        raise NotImplementedError(self._message())

    def move(self, position: float, timeout_sec: float = 5.0) -> None:
        """Not implemented for real hardware yet."""
        raise NotImplementedError(self._message())

    def _message(self) -> str:
        """Explain that only the Gazebo backend exists in stage 2."""
        return (
            "RealFairinoGripper 尚未实现。"
            "阶段 2 请使用 create_gripper(node, backend='gazebo')。"
        )


def create_gripper(
    node: Node,
    backend: str = "gazebo",
    **kwargs,
) -> GripperInterface:
    """Build a gripper backend. Task nodes should call this factory."""
    name = str(backend).strip().lower()
    if name in ("gazebo", "sim", "simulation"):
        return GazeboGripper(node, **kwargs)
    if name in ("real", "fairino", "hardware"):
        return RealFairinoGripper(node, **kwargs)
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
