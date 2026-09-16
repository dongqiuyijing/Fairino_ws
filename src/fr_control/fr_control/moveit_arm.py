"""
Reusable MoveIt arm client.

This class does not plan locally. It sends goals to a running move_group
node: Plan first, then Execute. Task nodes only call go_joints / go_pose.
"""

from __future__ import annotations

from typing import Sequence

from geometry_msgs.msg import Pose, Quaternion, Vector3
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    RobotTrajectory,
    WorkspaceParameters,
)
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
import rclpy

from fr_control.constants import (
    ARM_JOINTS,
    BASE_FRAME,
    CARTESIAN_SERVICE,
    EE_LINK,
    EXECUTE_ACTION,
    MOVE_ACTION,
    PLANNING_GROUP,
)

_ERROR_NAMES = {
    value: name
    for name, value in vars(MoveItErrorCodes).items()
    if name.isupper() and isinstance(value, int)
}


class MoveItError(RuntimeError):
    """Raised when MoveIt planning or execution fails."""

    def __init__(self, message: str, error_code: int | None = None) -> None:
        """Attach an optional MoveIt error code."""
        super().__init__(message)
        self.error_code = error_code


class MoveItArm:
    """Thin client around an already running move_group node."""

    def __init__(
        self,
        node: Node,
        *,
        group_name: str = PLANNING_GROUP,
        ee_link: str = EE_LINK,
        base_frame: str = BASE_FRAME,
        joint_names: Sequence[str] = ARM_JOINTS,
        velocity_scale: float = 0.2,
        acceleration_scale: float = 0.2,
        planning_time: float = 5.0,
        planning_attempts: int = 10,
        position_tolerance: float = 0.005,
        orientation_tolerance: float = 0.05,
        plan_timeout_sec: float = 30.0,
        execute_timeout_sec: float = 240.0,
    ) -> None:
        """Store planning settings and create MoveIt action clients."""
        self._node = node
        self._group_name = group_name
        self._ee_link = ee_link
        self._base_frame = base_frame
        self._joint_names = tuple(joint_names)
        self._velocity_scale = velocity_scale
        self._acceleration_scale = acceleration_scale
        self._planning_time = planning_time
        self._planning_attempts = planning_attempts
        self._position_tolerance = position_tolerance
        self._orientation_tolerance = orientation_tolerance
        self._plan_timeout_sec = plan_timeout_sec
        self._execute_timeout_sec = execute_timeout_sec

        self._move_client = ActionClient(node, MoveGroup, MOVE_ACTION)
        self._exec_client = ActionClient(
            node, ExecuteTrajectory, EXECUTE_ACTION
        )
        self._cartesian_client = node.create_client(
            GetCartesianPath, CARTESIAN_SERVICE
        )

    def wait_until_ready(self, timeout_sec: float = 30.0) -> None:
        """Wait until move_group Plan / Execute actions are available."""
        self._node.get_logger().info("等待 MoveIt move_group ...")
        if not self._move_client.wait_for_server(timeout_sec=timeout_sec):
            raise MoveItError(
                f"在 {timeout_sec:.0f}s 内未找到 {MOVE_ACTION}，"
                "请先启动 fairino3_v6_moveit2_config 的 move_group"
            )
        if not self._exec_client.wait_for_server(timeout_sec=timeout_sec):
            raise MoveItError(
                f"在 {timeout_sec:.0f}s 内未找到 {EXECUTE_ACTION}"
            )
        if not self._cartesian_client.wait_for_service(timeout_sec=timeout_sec):
            raise MoveItError(
                f"在 {timeout_sec:.0f}s 内未找到 {CARTESIAN_SERVICE}"
            )
        self._node.get_logger().info("MoveIt move_group 已就绪")

    def set_velocity_scale(self, velocity_scale: float) -> None:
        """Update max velocity scaling for later Plan / Cartesian timing."""
        self._velocity_scale = float(velocity_scale)

    def plan_joints(self, positions: Sequence[float]) -> RobotTrajectory:
        """Plan to a joint target without executing."""
        if len(positions) != len(self._joint_names):
            raise ValueError(
                f"关节数量应为 {len(self._joint_names)}，"
                f"实际为 {len(positions)}"
            )
        constraints = Constraints()
        for name, value in zip(self._joint_names, positions):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(value)
            joint.tolerance_above = 0.001
            joint.tolerance_below = 0.001
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        return self._plan(constraints, f"joints={list(positions)}")

    def plan_pose(
        self,
        pose: Pose,
        frame_id: str | None = None,
    ) -> RobotTrajectory:
        """Plan to an end-effector Pose without executing."""
        frame = frame_id or self._base_frame
        header = Header()
        header.frame_id = frame
        header.stamp = self._node.get_clock().now().to_msg()

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self._position_tolerance]

        region = BoundingVolume()
        region.primitives.append(sphere)
        sphere_pose = Pose()
        sphere_pose.position = pose.position
        sphere_pose.orientation.w = 1.0
        region.primitive_poses.append(sphere_pose)

        position = PositionConstraint()
        position.header = header
        position.link_name = self._ee_link
        position.constraint_region = region
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header = header
        orientation.link_name = self._ee_link
        orientation.orientation = pose.orientation
        orientation.absolute_x_axis_tolerance = self._orientation_tolerance
        orientation.absolute_y_axis_tolerance = self._orientation_tolerance
        orientation.absolute_z_axis_tolerance = self._orientation_tolerance
        orientation.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position)
        constraints.orientation_constraints.append(orientation)

        pos = pose.position
        ori = pose.orientation
        label = (
            f"pose xyz=({pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f}) "
            f"xyzw=({ori.x:.4f}, {ori.y:.4f}, {ori.z:.4f}, {ori.w:.4f})"
        )
        return self._plan(constraints, label)

    def execute(self, trajectory: RobotTrajectory) -> None:
        """Execute a previously planned trajectory."""
        if not trajectory.joint_trajectory.points:
            self._node.get_logger().info("规划轨迹为空，跳过执行")
            return

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        self._node.get_logger().info("Execute ...")
        result = self._send_action(
            self._exec_client,
            goal,
            self._execute_timeout_sec,
        )
        self._check_error(result.error_code.val, "Execute")
        self._node.get_logger().info("Execute 完成")

    def go_joints(self, positions: Sequence[float]) -> None:
        """Plan and execute a joint-space goal."""
        self.execute(self.plan_joints(positions))

    def go_pose(self, pose: Pose, frame_id: str | None = None) -> None:
        """Plan and execute a Cartesian Pose goal."""
        self.execute(self.plan_pose(pose, frame_id=frame_id))

    def plan_cartesian(
        self,
        waypoints: Sequence[Pose],
        frame_id: str | None = None,
        *,
        eef_step: float = 0.01,
        jump_threshold: float = 0.0,
        avoid_collisions: bool = True,
        fraction_min: float = 0.95,
    ) -> RobotTrajectory:
        """Plan a straight-line Cartesian path through waypoints."""
        if not waypoints:
            raise ValueError("Cartesian 路径至少需要一个 waypoint")
        request = GetCartesianPath.Request()
        request.header.frame_id = frame_id or self._base_frame
        request.header.stamp = self._node.get_clock().now().to_msg()
        request.start_state.is_diff = True
        request.group_name = self._group_name
        request.link_name = self._ee_link
        request.waypoints = list(waypoints)
        request.max_step = float(eef_step)
        request.jump_threshold = float(jump_threshold)
        request.prismatic_jump_threshold = 0.0
        request.revolute_jump_threshold = 0.0
        request.avoid_collisions = bool(avoid_collisions)

        self._node.get_logger().info(
            f"Cartesian Plan: {len(waypoints)} waypoints, "
            f"eef_step={eef_step:.3f}"
        )
        future = self._cartesian_client.call_async(request)
        result = self._wait_future(future, self._plan_timeout_sec)
        if result is None:
            raise MoveItError("Cartesian 规划无响应")
        self._check_error(result.error_code.val, "Cartesian Plan")
        fraction = float(result.fraction)
        self._node.get_logger().info(
            f"Cartesian Plan fraction={fraction:.3f}"
        )
        if fraction < float(fraction_min):
            raise MoveItError(
                f"Cartesian 路径不完整：fraction={fraction:.3f} "
                f"< {fraction_min:.3f}"
            )
        trajectory = result.solution
        self._ensure_trajectory_timing(trajectory, eef_step)
        return trajectory

    def go_cartesian(
        self,
        waypoints: Sequence[Pose],
        frame_id: str | None = None,
        **kwargs,
    ) -> None:
        """Plan and execute a straight-line Cartesian path."""
        self.execute(
            self.plan_cartesian(waypoints, frame_id=frame_id, **kwargs)
        )

    def _ensure_trajectory_timing(
        self,
        trajectory: RobotTrajectory,
        eef_step: float,
    ) -> None:
        """Fill time_from_start if Cartesian path returned zero times."""
        points = trajectory.joint_trajectory.points
        if not points:
            return
        last = points[-1].time_from_start
        if last.sec > 0 or last.nanosec > 0:
            return
        speed = max(0.03, 0.25 * self._velocity_scale)
        dt = max(float(eef_step) / speed, 0.05)
        for index, point in enumerate(points):
            elapsed = dt * index
            point.time_from_start.sec = int(elapsed)
            point.time_from_start.nanosec = int((elapsed % 1.0) * 1e9)

    def _plan(
        self,
        constraints: Constraints,
        label: str,
    ) -> RobotTrajectory:
        """Request a plan-only MoveGroup action."""
        goal = MoveGroup.Goal()
        goal.request = self._make_request(constraints)
        goal.planning_options = self._make_planning_options(plan_only=True)

        self._node.get_logger().info(f"Plan: {label}")
        result = self._send_action(
            self._move_client,
            goal,
            self._plan_timeout_sec,
        )
        self._check_error(result.error_code.val, "Plan")
        n_points = len(result.planned_trajectory.joint_trajectory.points)
        self._node.get_logger().info(
            f"Plan 成功：{n_points} 个轨迹点，"
            f"用时 {result.planning_time:.2f}s"
        )
        return result.planned_trajectory

    def _make_request(self, constraints: Constraints) -> MotionPlanRequest:
        """Build a MotionPlanRequest."""
        request = MotionPlanRequest()
        request.group_name = self._group_name
        request.num_planning_attempts = self._planning_attempts
        request.allowed_planning_time = self._planning_time
        request.max_velocity_scaling_factor = self._velocity_scale
        request.max_acceleration_scaling_factor = self._acceleration_scale
        request.goal_constraints.append(constraints)
        request.start_state.is_diff = True
        request.workspace_parameters = self._workspace()
        return request

    def _workspace(self) -> WorkspaceParameters:
        """Return a workspace box large enough for FR3."""
        workspace = WorkspaceParameters()
        workspace.header.frame_id = self._base_frame
        workspace.min_corner = Vector3(x=-2.0, y=-2.0, z=-2.0)
        workspace.max_corner = Vector3(x=2.0, y=2.0, z=2.0)
        return workspace

    def _make_planning_options(self, plan_only: bool) -> PlanningOptions:
        """Build PlanningOptions; default is plan only."""
        options = PlanningOptions()
        options.plan_only = plan_only
        options.replan = True
        options.replan_attempts = 5
        options.replan_delay = 0.1
        options.planning_scene_diff.is_diff = True
        options.planning_scene_diff.robot_state.is_diff = True
        return options

    def _send_action(self, client: ActionClient, goal, timeout_sec: float):
        """Send an action goal and wait for the result."""
        send_future = client.send_goal_async(goal)
        goal_handle = self._wait_future(send_future, timeout_sec)
        if goal_handle is None or not goal_handle.accepted:
            raise MoveItError("MoveIt 拒绝了该目标")
        result_future = goal_handle.get_result_async()
        wrapped = self._wait_future(result_future, timeout_sec)
        if wrapped is None:
            raise MoveItError("等待 MoveIt 结果超时")
        return wrapped.result

    def _wait_future(self, future, timeout_sec: float):
        """Block until the future completes."""
        rclpy.spin_until_future_complete(
            self._node,
            future,
            timeout_sec=timeout_sec,
        )
        if not future.done():
            raise MoveItError(f"等待 MoveIt 响应超时（{timeout_sec:.0f}s）")
        return future.result()

    def _check_error(self, error_code: int, stage: str) -> None:
        """Raise MoveItError unless the error code is SUCCESS."""
        if error_code == MoveItErrorCodes.SUCCESS:
            return
        name = _ERROR_NAMES.get(error_code, "UNKNOWN")
        raise MoveItError(
            f"{stage} 失败：{name} ({error_code})",
            error_code=error_code,
        )


def make_pose(
    xyz: Sequence[float],
    xyzw: Sequence[float],
) -> Pose:
    """Build a geometry_msgs/Pose from position and xyzw quaternion."""
    pose = Pose()
    pose.position.x = float(xyz[0])
    pose.position.y = float(xyz[1])
    pose.position.z = float(xyz[2])
    pose.orientation = Quaternion(
        x=float(xyzw[0]),
        y=float(xyzw[1]),
        z=float(xyzw[2]),
        w=float(xyzw[3]),
    )
    return pose
