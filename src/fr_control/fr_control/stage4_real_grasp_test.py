"""
Stage 4 real-robot known-pose grasp test.

Current pose -> pre-grasp -> Cartesian descend -> close ->
MoveIt attach -> Cartesian lift 5 cm -> stop.

Does not go Home, P1, or Face1/2/3. Does not call Gazebo or sim_grasp.
"""

from __future__ import annotations

import math
import os
import time
from typing import Any

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

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
    pose_inverse,
    pose_multiply,
    quat_to_matrix,
    quat_xyzw,
    rotate_vec,
)
from fr_control.gripper import GripperError, create_gripper
from fr_control.inspection_poses import as_vec3, pose_position_error
from fr_control.moveit_arm import MoveItArm, MoveItError
from fr_control.planning_scene import PlanningSceneClient
from fr_control.preflight import check_unique_sim_graph
from fr_control.stage4_config import (
    block_pose,
    grasp_tcp_xyzw,
    load_yaml,
    robot_base_pose,
)


class Stage4RealGraspNode(Node):
    """Run a top-down real grasp against an already running real_bringup."""

    def __init__(self) -> None:
        """Load stage4_config.yaml and prepare TF."""
        super().__init__("fr_stage4_real_grasp_test")
        self.declare_parameter("config_file", "")
        self.declare_parameter("gripper_backend", "real")
        self.declare_parameter("gripper_config_file", "")
        self.declare_parameter("skip_confirm", False)
        self.declare_parameter("pre_move_delay_sec", 3.0)

        config_file = (
            self.get_parameter("config_file")
            .get_parameter_value()
            .string_value
        )
        if not config_file:
            config_file = os.path.join(
                get_package_share_directory("fr_control"),
                "config",
                "stage4_config.yaml",
            )
        self._config = load_yaml(config_file)
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._joint_state: JointState | None = None
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )
        self.get_logger().info(f"已加载真机抓取配置：{config_file}")

    def run(self) -> None:
        """Execute the real grasp sequence. Does not start arm motion itself."""
        cfg = self._config
        world_frame = str(cfg.get("frames", {}).get("world", "world"))
        planning_frame = str(cfg.get("planning_frame", BASE_FRAME))
        ee_link = str(cfg.get("ee_link", EE_LINK))
        attach_link = str(cfg.get("attach_link", ATTACH_LINK))
        grasp_cfg = cfg["grasp"]
        object_cfg = cfg["object"]
        table_cfg = cfg["table"]
        motion_cfg = cfg["motion"]
        touch_links = list(cfg.get("gripper_touch_links", GRIPPER_TOUCH_LINKS))
        t_world_base = robot_base_pose(cfg)
        target_frame = world_frame

        transit_scale = float(
            motion_cfg.get("transit_velocity_scaling", 0.30)
        )
        approach_scale = float(
            motion_cfg.get("approach_velocity_scaling", 0.10)
        )
        lift_scale = float(motion_cfg.get("lift_velocity_scaling", 0.30))
        fingertip_from_tcp = float(grasp_cfg["real_fingertip_from_tcp"])
        lift_distance = float(grasp_cfg["real_lift_distance"])
        pregrasp_distance = float(grasp_cfg["pregrasp_distance"])

        arm = MoveItArm(
            self,
            group_name=str(cfg.get("move_group", PLANNING_GROUP)),
            ee_link=ee_link,
            base_frame=str(cfg.get("base_frame", BASE_FRAME)),
            joint_names=ARM_JOINTS,
            velocity_scale=transit_scale,
            acceleration_scale=float(
                motion_cfg.get("acceleration_scaling", 0.2)
            ),
            planning_time=float(motion_cfg.get("planning_time", 10.0)),
            planning_attempts=int(motion_cfg.get("planning_attempts", 16)),
            position_tolerance=float(
                motion_cfg.get("position_tolerance", 0.005)
            ),
            orientation_tolerance=math.radians(
                float(motion_cfg.get("orientation_tolerance_deg", 3.0))
            ),
        )
        arm.wait_until_ready()
        check_unique_sim_graph(self)
        scene = PlanningSceneClient(self)
        scene.wait_until_ready()

        gripper_backend = self._string_param("gripper_backend", "real")
        gripper = create_gripper(
            self,
            backend=gripper_backend,
            config_file=self._string_param("gripper_config_file", ""),
        )
        gripper.check_connection()
        gripper_info = gripper.describe()

        self._wait_for_tf(planning_frame, world_frame)
        self._wait_for_tf(world_frame, ee_link)
        self._log_tf(world_frame, ee_link)

        object_source = str(
            object_cfg["initial_pose"].get("frame", world_frame)
        )
        object_in_source = block_pose(object_cfg["initial_pose"])
        object_world = self._to_frame(
            object_in_source,
            object_source,
            world_frame,
            t_world_base,
        )
        poses = compute_grasp_poses(
            object_world,
            approach_xyzw=grasp_tcp_xyzw(grasp_cfg),
            pre_grasp_offset=pregrasp_distance,
            lift_height=lift_distance,
            table_top_z=float(grasp_cfg["table_top_z"]),
            fingertip_from_tcp=fingertip_from_tcp,
            table_clearance=float(grasp_cfg["table_clearance"]),
            planning_frame=world_frame,
        )

        object_shape = str(object_cfg.get("shape", "box")).lower()
        if object_shape != "cylinder":
            raise MoveItError(
                f"真机抓取目前只支持 cylinder，当前 object.shape={object_shape}"
            )
        object_radius = float(object_cfg["dimensions"]["radius"])
        object_height = float(object_cfg["dimensions"]["height"])
        object_name = str(object_cfg["name"])
        table_id = str(table_cfg["name"])
        tcp_tol = float(motion_cfg.get("tcp_tolerance", 0.020))
        pose_attempts = int(motion_cfg.get("pose_attempts", 3))
        gripper_timeout = float(grasp_cfg.get("gripper_timeout_sec", 40.0))

        object_in_planning = self._to_frame(
            object_world,
            world_frame,
            planning_frame,
            t_world_base,
        )

        tcp_x_world = rotate_vec(
            quat_to_matrix(quat_xyzw(poses.grasp_pose.orientation)),
            (1.0, 0.0, 0.0),
        )
        tcp_z_world = poses.approach_vector

        self.get_logger().info(
            "========== REAL GRASP PREFLIGHT =========="
        )
        self.get_logger().info(
            f"零件 {object_name} cylinder "
            f"radius={object_radius:.4f} height={object_height:.4f}"
        )
        self.get_logger().info(
            f"object pose ({world_frame}) {format_pose(poses.object_pose)}"
        )
        self.get_logger().info(
            f"pre-grasp pose ({world_frame}) {format_pose(poses.pre_grasp_pose)}"
        )
        self.get_logger().info(
            f"grasp pose ({world_frame}) {format_pose(poses.grasp_pose)}"
        )
        self.get_logger().info(
            f"lift pose ({world_frame}) {format_pose(poses.lift_pose)}"
        )
        self.get_logger().info(
            "TCP orientation xyzw="
            f"({poses.grasp_pose.orientation.x:.4f}, "
            f"{poses.grasp_pose.orientation.y:.4f}, "
            f"{poses.grasp_pose.orientation.z:.4f}, "
            f"{poses.grasp_pose.orientation.w:.4f})"
        )
        self.get_logger().info(
            f"TCP +Z in {world_frame} = "
            f"({tcp_z_world[0]:.3f}, {tcp_z_world[1]:.3f}, {tcp_z_world[2]:.3f})"
        )
        self.get_logger().info(
            f"TCP +X in {world_frame} = "
            f"({tcp_x_world[0]:.3f}, {tcp_x_world[1]:.3f}, {tcp_x_world[2]:.3f})"
        )
        self.get_logger().info(f"planning frame = {planning_frame}")
        self.get_logger().info(f"target frame = {target_frame}")
        self.get_logger().info(f"transit speed = {transit_scale:.2f}")
        self.get_logger().info(f"approach speed = {approach_scale:.2f}")
        self.get_logger().info(f"lift speed = {lift_scale:.2f}")
        self.get_logger().info(
            f"real_fingertip_from_tcp = {fingertip_from_tcp:.3f}"
        )
        self.get_logger().info(f"real_lift_distance = {lift_distance:.3f}")
        self.get_logger().info(f"gripper backend = {gripper_backend}")
        self.get_logger().info(
            "close_position = "
            f"{gripper_info.get('close_position', gripper_info)}"
        )
        self.get_logger().info(f"夹爪配置：{gripper_info}")
        self.get_logger().info(
            "MoveIt / TF / Planning Scene / 真实夹爪服务已就绪。"
            "尚未发送任何机械臂或夹爪运动。"
        )

        scene.detach(object_name, attach_link)
        scene.remove(object_name)
        scene.add_cylinder(
            object_name,
            object_in_planning,
            object_height,
            object_radius,
            planning_frame,
        )
        scene.allow_collisions(object_name, touch_links, allowed=True)
        scene.allow_collisions(object_name, [table_id], allowed=True)

        self._confirm_or_abort()
        self._countdown()

        self.get_logger().warn(
            "开始真实硬件动作：先夹爪 open()，再 MoveIt 运动到 pre-grasp"
        )

        self.get_logger().info("1. Open gripper")
        gripper.open(timeout_sec=gripper_timeout)

        self.get_logger().info("2. Move to pre-grasp (30%)")
        arm.set_velocity_scale(transit_scale)
        self._go_pose_verified(
            arm,
            poses.pre_grasp_pose,
            target_frame,
            ee_link,
            "pre-grasp",
            tcp_tol,
            pose_attempts,
        )

        self.get_logger().info("3. Cartesian descend to grasp (10%)")
        arm.set_velocity_scale(approach_scale)
        self._linear_move(
            arm,
            poses.grasp_pose,
            target_frame,
            ee_link,
            grasp_cfg,
            tcp_tol,
            pose_attempts,
        )

        self.get_logger().info("4. Close gripper")
        gripper.close(timeout_sec=gripper_timeout)
        settle = float(grasp_cfg.get("close_settle_seconds", 1.5))
        if settle > 0.0:
            time.sleep(settle)

        self.get_logger().info("5. MoveIt attach cylinder")
        scene.attach_cylinder(
            object_name,
            pose_in_frame(poses.grasp_pose, poses.object_pose),
            object_height,
            object_radius,
            link_name=attach_link,
            touch_links=touch_links,
        )

        self.get_logger().info("6. Cartesian lift (30%)")
        arm.set_velocity_scale(lift_scale)
        self._linear_move(
            arm,
            poses.lift_pose,
            target_frame,
            ee_link,
            grasp_cfg,
            tcp_tol,
            pose_attempts,
        )

        self.get_logger().info(
            "真机固定抓取测试完成：已抬升 5 cm 并停止，不进入 P1/Face"
        )

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
        """Cartesian straight-line move. No joint-space fallback."""
        eef_step = float(grasp_cfg.get("cartesian_eef_step", 0.01))
        fraction_min = float(grasp_cfg.get("cartesian_fraction_min", 0.95))
        last_error: MoveItError | None = None
        tcp_error = tcp_tol + 1.0
        for attempt in range(1, attempts + 1):
            try:
                arm.go_cartesian(
                    [goal],
                    frame_id=frame_id,
                    eef_step=eef_step,
                    fraction_min=fraction_min,
                    avoid_collisions=True,
                )
            except MoveItError as exc:
                last_error = exc
                self.get_logger().warn(
                    f"Cartesian 第 {attempt}/{attempts} 次失败：{exc}"
                )
                self._wait_arm_settled()
                tcp_error = self._tcp_error(goal, frame_id, ee_link)
                if tcp_error <= tcp_tol:
                    self.get_logger().warn(
                        "Cartesian 控制器报失败，但 TCP 已到位 "
                        f"({tcp_error * 1000.0:.1f} mm)，继续"
                    )
                    self._log_tf(frame_id, ee_link)
                    return
                continue
            time.sleep(0.4)
            tcp_error = self._tcp_error(goal, frame_id, ee_link)
            self.get_logger().info(
                f"Cartesian 第 {attempt}/{attempts} 次，"
                f"TCP 误差 {tcp_error * 1000.0:.1f} mm"
            )
            self._log_tf(frame_id, ee_link)
            if tcp_error <= tcp_tol:
                return
        if last_error is not None and tcp_error > tcp_tol:
            raise last_error
        raise MoveItError(
            "Cartesian 末端未到位，误差 "
            f"{tcp_error:.3f} m（阈值 {tcp_tol:.3f} m）。"
            "真机下降/抬升禁止改用关节空间规划。"
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

    def _confirm_or_abort(self) -> None:
        """Require an operator Enter before any real motion."""
        skip = (
            self.get_parameter("skip_confirm")
            .get_parameter_value()
            .bool_value
        )
        if skip:
            self.get_logger().warn("skip_confirm=true：跳过 Enter 确认")
            return
        self.get_logger().info(
            "真机即将运动。确认周围安全、急停可触及后，在此终端按 Enter。"
            "若要跳过确认，使用 skip_confirm:=true"
        )
        try:
            input("按 Enter 继续真机固定抓取测试...")
        except EOFError as exc:
            raise GripperError(
                "无法读取确认输入。请用 skip_confirm:=true 或在 TTY 中运行"
            ) from exc

    def _countdown(self) -> None:
        """Short wall-clock delay after Enter, before the first command."""
        delay = float(
            self.get_parameter("pre_move_delay_sec")
            .get_parameter_value()
            .double_value
        )
        if delay <= 0.0:
            return
        self.get_logger().warn(
            f"{delay:.1f} 秒后开始运动。请确认工作空间安全、急停可触及。"
        )
        deadline = time.monotonic() + delay
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            self.get_logger().warn(f"倒计时 {remaining:.1f} s")
            time.sleep(min(1.0, remaining))

    def _on_joint_state(self, msg: JointState) -> None:
        """Cache the latest joint state."""
        self._joint_state = msg

    def _wait_arm_settled(self, timeout_sec: float = 3.0) -> None:
        """Wait until arm joint velocities drop after a cancelled trajectory."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
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

    def _tcp_error(self, goal: Pose, frame_id: str, ee_link: str) -> float:
        """Return the TCP position error versus goal in frame_id."""
        actual = self._lookup_tcp(frame_id, ee_link)
        return pose_position_error(actual, goal)

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

    def _to_frame(
        self,
        pose: Pose,
        source_frame: str,
        target_frame: str,
        t_world_base: Pose,
    ) -> Pose:
        """Transform a Pose between frames via TF, with YAML world/base fallback."""
        if source_frame == target_frame:
            return pose
        try:
            transform = self._tf.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            if {source_frame, target_frame} <= {"world", "base_link"}:
                self.get_logger().warn(
                    f"TF {source_frame}->{target_frame} 不可用（{exc}），"
                    "使用 YAML robot.base_pose"
                )
                return _fallback_world_base(
                    pose, source_frame, target_frame, t_world_base
                )
            raise MoveItError(
                f"TF 失败：{source_frame} -> {target_frame}: {exc}"
            ) from exc
        converted = apply_transform(transform.transform, pose)
        t = transform.transform.translation
        self.get_logger().info(
            f"TF {source_frame}->{target_frame} "
            f"t=({t.x:.4f}, {t.y:.4f}, {t.z:.4f})"
        )
        return converted

    def _wait_for_tf(
        self,
        target_frame: str,
        source_frame: str,
        timeout_sec: float = 15.0,
    ) -> None:
        """Wait until target_frame <- source_frame exists."""
        if target_frame == source_frame:
            return
        deadline = time.monotonic() + timeout_sec
        last_error = "timeout"
        while time.monotonic() < deadline and rclpy.ok():
            try:
                self._tf.lookup_transform(
                    target_frame, source_frame, rclpy.time.Time()
                )
                return
            except TransformException as exc:
                last_error = str(exc)
                rclpy.spin_once(self, timeout_sec=0.1)
        raise MoveItError(
            f"等待 TF {source_frame} -> {target_frame} 失败：{last_error}"
        )

    def _log_tf(self, frame_id: str, ee_link: str) -> None:
        """Log the current TCP pose in frame_id."""
        try:
            transform = self._tf.lookup_transform(
                frame_id, ee_link, rclpy.time.Time()
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
            f"TF {frame_id}->{ee_link} {format_pose(pose.pose)}"
        )

    def _string_param(self, name: str, default: str) -> str:
        """Read a string ROS parameter with a default."""
        if not self.has_parameter(name):
            return default
        value = self.get_parameter(name).get_parameter_value().string_value
        return value if value else default


def _fallback_world_base(
    pose: Pose,
    source_frame: str,
    target_frame: str,
    t_world_base: Pose,
) -> Pose:
    """Convert between world and base_link using YAML T_world_base."""
    if source_frame == "world" and target_frame == "base_link":
        return pose_multiply(pose_inverse(t_world_base), pose)
    if source_frame == "base_link" and target_frame == "world":
        return pose_multiply(t_world_base, pose)
    raise MoveItError(
        f"无法在 {source_frame} 与 {target_frame} 之间转换位姿"
    )


def main(args=None) -> None:
    """ROS 2 entry point; exits after the real grasp sequence finishes."""
    rclpy.init(args=args)
    node = Stage4RealGraspNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("已中断")
    except (MoveItError, GripperError, TransformException, KeyError) as exc:
        node.get_logger().error(f"真机抓取测试失败：{exc}")
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
