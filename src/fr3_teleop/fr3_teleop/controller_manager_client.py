"""ROS service/action-status adapter for strict AUTO <-> TELEOP ownership."""

from __future__ import annotations

from time import monotonic
from typing import Callable

from action_msgs.msg import GoalStatus, GoalStatusArray
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController

from .stage1_cartesian import controller_pair_state, switch_lists


class AutoTrajectoryMonitor:
    """Conservative AUTO activity detector using action, JTC and measured state."""

    def __init__(self, node, arms: dict[str, dict], timeout_s: float = 1.0,
                 settle_s: float = 0.5, desired_error_rad: float = 0.01,
                 stationary_delta_rad: float = 0.001, task_nodes: tuple[str, ...] = ()) -> None:
        self._node, self._arms, self._timeout_s = node, arms, timeout_s
        self._settle_s, self._desired_error, self._stationary_delta = settle_s, desired_error_rad, stationary_delta_rad
        self._task_nodes = set(task_nodes)
        self._latest = {arm: (0.0, "UNKNOWN") for arm in arms}
        self._controller = {arm: (0.0, False, 0.0) for arm in arms}
        self._joints = {arm: (0.0, None, 0.0) for arm in arms}
        for arm, spec in arms.items():
            topic = f"/{spec['auto_controller']}/follow_joint_trajectory/_action/status"
            node.create_subscription(GoalStatusArray, topic, lambda msg, key=arm: self._callback(key, msg), 10)

    def _callback(self, arm: str, message: GoalStatusArray) -> None:
        active = {GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING}
        state = "ACTIVE" if any(item.status in active for item in message.status_list) else "IDLE"
        self._latest[arm] = (monotonic(), state)

    def note_controller_state(self, arm: str, message) -> None:
        now = monotonic()
        desired, actual = list(message.desired.positions), list(message.actual.positions)
        stable = len(desired) == 6 and len(actual) == 6 and max(abs(a - b) for a, b in zip(desired, actual)) <= self._desired_error
        _, previous, since = self._controller[arm]
        self._controller[arm] = (now, stable, since if stable and previous else now)

    def note_joint_positions(self, arm: str, positions: list[float]) -> None:
        now = monotonic(); _, previous, since = self._joints[arm]
        stable = previous is not None and len(previous) == len(positions) and max(abs(a - b) for a, b in zip(previous, positions)) <= self._stationary_delta
        _, _, prior_since = self._joints[arm]
        self._joints[arm] = (now, positions, prior_since if stable else now)

    def state(self, arm: str) -> str:
        now = monotonic(); action_stamp, action = self._latest[arm]
        if action == "ACTIVE" and now - action_stamp <= self._timeout_s:
            return "ACTIVE"
        c_stamp, c_stable, c_since = self._controller[arm]
        j_stamp, _, j_since = self._joints[arm]
        known_nodes = {name for name, _ in self._node.get_node_names_and_namespaces()}
        if self._task_nodes & known_nodes:
            return "UNKNOWN"
        if now - c_stamp > self._timeout_s or now - j_stamp > self._timeout_s or not c_stable:
            return "UNKNOWN"
        return "IDLE" if now - c_since >= self._settle_s and now - j_since >= self._settle_s else "UNKNOWN"


class ControllerManagerClient:
    """Non-blocking strict switch client. Success callback is the ownership grant."""

    def __init__(self, node, arms: dict[str, dict], monitor: AutoTrajectoryMonitor,
                 cartesian_arm_a: bool = False) -> None:
        self._node, self._arms, self._monitor = node, arms, monitor
        self._cartesian_arm_a = cartesian_arm_a
        self._list = node.create_client(ListControllers, "/controller_manager/list_controllers")
        self._switch = node.create_client(SwitchController, "/controller_manager/switch_controller")

    def get_controller_state(self, arm: str, callback: Callable[[str], None]) -> bool:
        if arm not in self._arms or not self._list.service_is_ready(): return False
        future = self._list.call_async(ListControllers.Request())
        future.add_done_callback(lambda done: callback(self._pair_state(arm, done.result())))
        return True

    def request_manual(self, arm: str, callback: Callable[[bool, str], None]) -> bool:
        if arm not in self._arms or self._monitor.state(arm) != "IDLE":
            callback(False, "automatic trajectory is ACTIVE or UNKNOWN"); return False
        if not self._list.service_is_ready() or not self._switch.service_is_ready():
            callback(False, "controller_manager services unavailable"); return False
        future = self._list.call_async(ListControllers.Request())
        future.add_done_callback(lambda done: self._switch_if_pair(done, arm, True, callback))
        return True

    def release_manual(self, arm: str, callback: Callable[[bool, str], None]) -> bool:
        if arm not in self._arms or not self._list.service_is_ready() or not self._switch.service_is_ready():
            callback(False, "controller_manager services unavailable"); return False
        future = self._list.call_async(ListControllers.Request())
        future.add_done_callback(lambda done: self._switch_if_pair(done, arm, False, callback))
        return True

    def _pair_state(self, arm: str, response) -> str:
        if response is None: return "UNKNOWN"
        states = {item.name: item.state for item in response.controller}
        spec = self._arms[arm]
        return controller_pair_state(
            arm, states.get(spec["auto_controller"]), states.get(spec["teleop_controller"]),
            self._cartesian_arm_a)

    def _switch_if_pair(self, future, arm: str, manual: bool, callback: Callable[[bool, str], None]) -> None:
        state = self._pair_state(arm, future.result())
        expected = "AUTO" if manual else "MANUAL"
        if state != expected:
            callback(False, f"controller pair not in expected {expected} state: {state}"); return
        spec = self._arms[arm]; request = SwitchController.Request()
        activate, deactivate = switch_lists(
            arm, manual, self._cartesian_arm_a, spec["auto_controller"], spec["teleop_controller"])
        request.activate_controllers = activate
        request.deactivate_controllers = deactivate
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = False
        request.timeout = Duration(sec=2)
        switch = self._switch.call_async(request)
        switch.add_done_callback(lambda done: callback(bool(done.result() and done.result().ok), "manual granted" if manual else "automatic restored"))
