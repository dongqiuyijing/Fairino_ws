"""
Stage 3 known-object grasp test.

Home -> open gripper -> pre-grasp -> linear grasp -> close ->
attach (MoveIt + optional Gazebo weld) -> lift -> hold.

Does not use cameras. Object pose comes from config/grasp.yaml.
"""

from __future__ import annotations

import os
import time
from typing import Any

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from fr_control.constants import (
    ARM_JOINTS,
    ATTACH_LINK,
    BASE_FRAME,
    EE_LINK,
    GRIPPER_TOUCH_LINKS,
    PLANNING_GROUP,
)
from fr_control.grasp_poses import (
    apply_transform,
    compute_grasp_poses,
    format_pose,
    pose_in_frame,
)
from fr_control.gripper import GripperError, create_gripper
from fr_control.moveit_arm import MoveItArm, MoveItError, make_pose
from fr_control.planning_scene import PlanningSceneClient
from fr_control.preflight import check_tcp_height, check_unique_sim_graph
from fr_control.sim_grasp import create_sim_grasp


class GraspTestNode(Node):
    """Run one known-pose pick sequence against a running MoveIt + Gazebo."""

    def __init__(self) -> None:
        """Load grasp.yaml and prepare TF."""
        super().__init__("fr_grasp_test")
        self.declare_parameter("config_file", "")
        self.declare_parameter("gripper_backend", "gazebo")
        self.declare_parameter("sim_grasp_backend", "gazebo")

        config_file = (
            self.get_parameter("config_file")
            .get_parameter_value()
            .string_value
        )
        if not config_file:
            config_file = os.path.join(
                get_package_share_directory("fr_control"),
                "config",
                "grasp.yaml",
            )
        self._config = _load_yaml(config_file)
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._joint_state: JointState | None = None
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )
        self.get_logger().info(f"已加载抓取配置：{config_file}")

    def run(self) -> None:
        """Execute the full grasp sequence."""
        self._wait_for_clock()
        cfg = self._config
        planning_frame = str(cfg.get("planning_frame", BASE_FRAME))
        ee_link = str(cfg.get("ee_link", EE_LINK))
        attach_link = str(cfg.get("attach_link", ATTACH_LINK))
        grasp_cfg = cfg["grasp"]
        object_cfg = cfg["object"]
        table_cfg = cfg["table"]
        touch_links = list(cfg.get("gripper_touch_links", GRIPPER_TOUCH_LINKS))

        arm = MoveItArm(
            self,
            group_name=str(cfg.get("move_group", PLANNING_GROUP)),
            ee_link=ee_link,
            base_frame=str(cfg.get("base_frame", BASE_FRAME)),
            joint_names=ARM_JOINTS,
            velocity_scale=float(cfg.get("velocity_scale", 0.15)),
            acceleration_scale=float(cfg.get("acceleration_scale", 0.15)),
            planning_time=float(cfg.get("planning_time", 8.0)),
            planning_attempts=int(cfg.get("planning_attempts", 12)),
            position_tolerance=float(cfg.get("position_tolerance", 0.008)),
            orientation_tolerance=float(
                cfg.get("orientation_tolerance", 0.08)
            ),
        )
        arm.wait_until_ready()
        check_unique_sim_graph(self)
        scene = PlanningSceneClient(self)
        scene.wait_until_ready()
        gripper = create_gripper(
            self,
            backend=self._string_param("gripper_backend", "gazebo"),
        )
        sim_grasp = create_sim_grasp(
            self,
            backend=self._sim_grasp_backend(grasp_cfg),
        )

        self._wait_for_tf(planning_frame, ee_link)
        self._log_tf(planning_frame, ee_link)
        tcp = self._lookup_tcp(planning_frame, ee_link)
        check_tcp_height(tcp.position.z)

        object_pose = self._to_planning_frame(
            _pose_from_block(object_cfg),
            str(object_cfg.get("frame", "world")),
            planning_frame,
        )
        table_pose = self._to_planning_frame(
            _pose_from_block(table_cfg),
            str(table_cfg.get("frame", "world")),
            planning_frame,
        )
        poses = compute_grasp_poses(
            object_pose,
            approach_rpy=grasp_cfg["approach_rpy"],
            pre_grasp_offset=float(grasp_cfg["pre_grasp_offset"]),
            lift_height=float(grasp_cfg["lift_height"]),
            table_top_z=float(grasp_cfg["table_top_z"]),
            fingertip_from_tcp=float(grasp_cfg["fingertip_from_tcp"]),
            table_clearance=float(grasp_cfg["table_clearance"]),
            planning_frame=planning_frame,
        )
        object_size = [float(value) for value in object_cfg["size"]]
        table_size = [float(value) for value in table_cfg["size"]]
        object_id = str(object_cfg["name"])
        table_id = str(table_cfg["name"])

        self.get_logger().info(
            "零件 "
            f"{object_id} size={object_size} mass={object_cfg.get('mass')} "
            f"frame={planning_frame} {format_pose(poses.object_pose)}"
        )
        self.get_logger().info(
            f"approach={poses.approach_vector} "
            f"pre_grasp {format_pose(poses.pre_grasp_pose)}"
        )
        self.get_logger().info(f"grasp {format_pose(poses.grasp_pose)}")
        self.get_logger().info(f"lift {format_pose(poses.lift_pose)}")

        sim_grasp.detach()

        scene.add_box(table_id, table_pose, table_size, planning_frame)
        scene.add_box(object_id, object_pose, object_size, planning_frame)
        scene.allow_collisions(object_id, touch_links, allowed=True)
        scene.allow_collisions(object_id, [table_id], allowed=True)

        home = [float(value) for value in cfg["home"]["joints"]]
        tcp_tol = float(cfg.get("tcp_tolerance", 0.025))
        pose_attempts = int(cfg.get("pose_attempts", 3))
        gripper_timeout = float(grasp_cfg.get("gripper_timeout_sec", 20.0))

        self.get_logger().info("1. Home")
        gripper.open(timeout_sec=gripper_timeout)
        self._go_joints_retry(arm, home, pose_attempts)

        self.get_logger().info("2. Open gripper")
        gripper.open(timeout_sec=gripper_timeout)

        self.get_logger().info("3. Move to pre-grasp")
        self._go_pose_verified(
            arm,
            poses.pre_grasp_pose,
            planning_frame,
            ee_link,
            "pre-grasp",
            tcp_tol,
            pose_attempts,
        )

        self.get_logger().info("4. Linear approach to grasp")
        self._linear_move(
            arm,
            poses.grasp_pose,
            planning_frame,
            ee_link,
            grasp_cfg,
            tcp_tol,
            pose_attempts,
        )

        self.get_logger().info("5. Close gripper")
        gripper.close(timeout_sec=gripper_timeout)
        settle = float(grasp_cfg.get("close_settle_seconds", 1.5))
        if settle > 0.0:
            time.sleep(settle)

        self.get_logger().info("6. Attach object (MoveIt + optional Gazebo)")
        sim_grasp.attach()
        scene.attach_box(
            object_id,
            pose_in_frame(poses.grasp_pose, poses.object_pose),
            object_size,
            link_name=attach_link,
            touch_links=touch_links,
        )

        self.get_logger().info("7. Lift object")
        self._linear_move(
            arm,
            poses.lift_pose,
            planning_frame,
            ee_link,
            grasp_cfg,
            tcp_tol,
            pose_attempts,
        )

        hold = float(grasp_cfg.get("hold_seconds", 5.0))
        self.get_logger().info(f"8. Hold {hold:.1f}s")
        if hold > 0.0:
            time.sleep(hold)
        self.get_logger().info("阶段 3 抓取测试完成：零件应已离开桌面")

    def _linear_move(
        self,
        arm: MoveItArm,
        goal: Pose,
        frame_id: str,
        ee_link: str,
        grasp_cfg: dict[str, Any],
        tcp_tol: float,
        attempts: int,
    ) -> None:
        """Move linearly to goal, then fall back to verified Pose planning."""
        eef_step = float(grasp_cfg.get("cartesian_eef_step", 0.01))
        fraction_min = float(grasp_cfg.get("cartesian_fraction_min", 0.95))
        try:
            arm.go_cartesian(
                [goal],
                frame_id=frame_id,
                eef_step=eef_step,
                fraction_min=fraction_min,
            )
            error = self._tcp_error(goal, frame_id, ee_link)
            self.get_logger().info(
                f"Cartesian TCP 误差 {error * 1000.0:.1f} mm"
            )
            if error <= tcp_tol:
                self._log_tf(frame_id, ee_link)
                return
        except MoveItError as exc:
            self.get_logger().warn(f"Cartesian 失败（{exc}），改用 Pose 规划")
        self._go_pose_verified(
            arm, goal, frame_id, ee_link, "linear-fallback", tcp_tol, attempts
        )

    def _go_pose_verified(
        self,
        arm: MoveItArm,
        pose: Pose,
        frame_id: str,
        ee_link: str,
        name: str,
        tcp_tol: float,
        attempts: int,
    ) -> None:
        """Plan to pose and retry until TF confirms the TCP is close."""
        last_error: MoveItError | None = None
        tcp_error = tcp_tol + 1.0
        for attempt in range(1, attempts + 1):
            try:
                arm.go_pose(pose, frame_id=frame_id)
            except MoveItError as exc:
                last_error = exc
                self.get_logger().warn(
                    f"{name} 第 {attempt}/{attempts} 次执行失败：{exc}"
                )
                self._wait_arm_settled()
                tcp_error = self._tcp_error(pose, frame_id, ee_link)
                if tcp_error <= tcp_tol:
                    self.get_logger().warn(
                        f"{name} 控制器报失败，但 TCP 已到位 "
                        f"({tcp_error * 1000.0:.1f} mm)，继续"
                    )
                    self._log_tf(frame_id, ee_link)
                    return
                continue
            time.sleep(0.4)
            tcp_error = self._tcp_error(pose, frame_id, ee_link)
            self.get_logger().info(
                f"{name} 第 {attempt}/{attempts} 次，"
                f"TCP 误差 {tcp_error * 1000.0:.1f} mm"
            )
            self._log_tf(frame_id, ee_link)
            if tcp_error <= tcp_tol:
                return
        if last_error is not None and tcp_error > tcp_tol:
            raise last_error
        raise MoveItError(
            f"{name} 末端未到位，误差 {tcp_error:.3f} m（阈值 {tcp_tol:.3f} m）"
        )

    def _go_joints_retry(
        self,
        arm: MoveItArm,
        joints: list[float],
        attempts: int,
    ) -> None:
        """Execute a joint goal, retrying CONTROL_FAILED from Gazebo lag."""
        last_error: MoveItError | None = None
        for attempt in range(1, attempts + 1):
            try:
                arm.go_joints(joints)
                return
            except MoveItError as exc:
                last_error = exc
                self.get_logger().warn(
                    f"Home 第 {attempt}/{attempts} 次执行失败：{exc}"
                )
                self._wait_arm_settled()
                if self._joints_near(joints, 0.08):
                    self.get_logger().warn(
                        "控制器报失败，但 Home 关节已到位，继续"
                    )
                    return
        if last_error is not None:
            raise last_error

    def _on_joint_state(self, msg: JointState) -> None:
        """Cache the latest joint state."""
        self._joint_state = msg

    def _joints_near(self, target: list[float], tol: float) -> bool:
        """Return True if arm joints are within tol of the target."""
        self._spin_joints()
        if self._joint_state is None:
            return False
        names = list(self._joint_state.name)
        positions = list(self._joint_state.position)
        for joint_name, goal in zip(ARM_JOINTS, target):
            if joint_name not in names:
                return False
            actual = positions[names.index(joint_name)]
            if abs(actual - goal) > tol:
                return False
        return True

    def _wait_arm_settled(self, timeout_sec: float = 3.0) -> None:
        """Wait until arm joint velocities drop after a cancelled trajectory."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            self._spin_joints()
            if self._arm_speed() < 0.08:
                return
            time.sleep(0.05)

    def _arm_speed(self) -> float:
        """Return the max absolute arm joint velocity."""
        if self._joint_state is None or not self._joint_state.velocity:
            return 0.0
        names = list(self._joint_state.name)
        velocities = list(self._joint_state.velocity)
        speeds = []
        for joint_name in ARM_JOINTS:
            if joint_name in names:
                speeds.append(abs(velocities[names.index(joint_name)]))
        return max(speeds) if speeds else 0.0

    def _spin_joints(self) -> None:
        """Process callbacks so joint_states stay fresh."""
        rclpy.spin_once(self, timeout_sec=0.1)

    def _tcp_error(self, goal: Pose, frame_id: str, ee_link: str) -> float:
        """Return the TCP position error versus goal in the planning frame."""
        actual = self._lookup_tcp(frame_id, ee_link)
        dx = actual.position.x - goal.position.x
        dy = actual.position.y - goal.position.y
        dz = actual.position.z - goal.position.z
        return (dx * dx + dy * dy + dz * dz) ** 0.5

    def _lookup_tcp(self, frame_id: str, ee_link: str) -> Pose:
        """Read the current TCP pose from TF."""
        transform = self._tf.lookup_transform(
            frame_id, ee_link, rclpy.time.Time()
        )
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose

    def _to_planning_frame(
        self,
        pose: Pose,
        source_frame: str,
        planning_frame: str,
    ) -> Pose:
        """Transform a Pose into the MoveIt planning frame using TF."""
        if source_frame == planning_frame:
            return pose
        try:
            transform = self._tf.lookup_transform(
                planning_frame,
                source_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            if {source_frame, planning_frame} <= {"world", "base_link"}:
                self.get_logger().warn(
                    f"TF {source_frame}->{planning_frame} 不可用（{exc}），"
                    "本仿真 world 与 base_link 重合，使用原 Pose"
                )
                return pose
            raise MoveItError(
                f"TF 失败：{source_frame} -> {planning_frame}: {exc}"
            ) from exc
        converted = apply_transform(transform.transform, pose)
        t = transform.transform.translation
        self.get_logger().info(
            f"TF {source_frame}->{planning_frame} "
            f"t=({t.x:.4f}, {t.y:.4f}, {t.z:.4f})"
        )
        return converted

    def _wait_for_tf(
        self,
        planning_frame: str,
        ee_link: str,
        timeout_sec: float = 15.0,
    ) -> None:
        """Wait until the planning frame to TCP transform exists."""
        deadline = time.monotonic() + timeout_sec
        last_error = "timeout"
        while time.monotonic() < deadline and rclpy.ok():
            try:
                self._tf.lookup_transform(
                    planning_frame, ee_link, rclpy.time.Time()
                )
                return
            except TransformException as exc:
                last_error = str(exc)
                rclpy.spin_once(self, timeout_sec=0.1)
        raise MoveItError(
            f"等待 TF {planning_frame} -> {ee_link} 失败：{last_error}"
        )

    def _log_tf(self, planning_frame: str, ee_link: str) -> None:
        """Log the current TCP pose in the planning frame."""
        try:
            transform = self._tf.lookup_transform(
                planning_frame, ee_link, rclpy.time.Time()
            )
        except TransformException as exc:
            self.get_logger().warn(f"无法读取 {ee_link} TF：{exc}")
            return
        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        self.get_logger().info(
            f"TF {planning_frame}->{ee_link} {format_pose(pose.pose)}"
        )

    def _sim_grasp_backend(self, grasp_cfg: dict[str, Any]) -> str:
        """Choose Gazebo weld vs no-op from YAML and CLI."""
        requested = self._string_param("sim_grasp_backend", "")
        if requested:
            return requested
        if bool(grasp_cfg.get("use_sim_attach", True)):
            return "gazebo"
        return "none"

    def _string_param(self, name: str, default: str) -> str:
        """Read a string ROS parameter with a default."""
        value = (
            self.get_parameter(name).get_parameter_value().string_value
        )
        return value if value else default

    def _wait_for_clock(self, timeout_sec: float = 20.0) -> None:
        """Wait until Gazebo publishes /clock when use_sim_time is set."""
        use_sim = (
            self.get_parameter("use_sim_time")
            .get_parameter_value()
            .bool_value
        )
        if not use_sim:
            return
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            if self.get_clock().now().nanoseconds > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise MoveItError(
            "未收到 Gazebo /clock。请先启动仿真，并保持 use_sim_time:=true"
        )


def _pose_from_block(block: dict[str, Any]) -> Pose:
    """Build a Pose from a YAML position / orientation_xyzw block."""
    return make_pose(block["position"], block["orientation_xyzw"])


def _load_yaml(path: str) -> dict[str, Any]:
    """Load a YAML grasp file."""
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"抓取配置格式错误：{path}")
    return data


def main(args=None) -> None:
    """ROS 2 entry point; exits after the grasp sequence finishes."""
    rclpy.init(args=args)
    node = GraspTestNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("已中断")
    except (MoveItError, GripperError, TransformException) as exc:
        node.get_logger().error(f"抓取测试失败：{exc}")
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
