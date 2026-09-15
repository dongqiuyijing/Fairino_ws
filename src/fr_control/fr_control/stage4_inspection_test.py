"""
Stage 4 known-object grasp and three-face inspection test.

Home -> open gripper -> grasp -> lift -> Face1/2/3 at P1.
Object poses are computed from YAML; TCP poses come from T_tcp_object.
Validation reads the Gazebo physics-world pose of the part.
"""

from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass, field
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
from fr_control.gazebo_world_pose import (
    GazeboPoseError,
    pose_from_rpy,
    read_model_pose,
    set_model_pose,
)
from fr_control.grasp_poses import (
    apply_transform,
    compute_grasp_poses,
    format_pose,
    pose_inverse,
    pose_multiply,
    pose_in_frame,
)
from fr_control.gripper import GripperError, create_gripper
from fr_control.inspection_poses import (
    InspectionError,
    angle_between_deg,
    as_vec3,
    interpolate_object_poses,
    object_in_tcp,
    object_pose_for_face,
    pose_position_error,
    projected_up,
    quat_angle_deg,
    rotate_pose_vector,
    tcp_pose_from_object,
)
from fr_control.moveit_arm import MoveItArm, MoveItError
from fr_control.planning_scene import PlanningSceneClient
from fr_control.preflight import check_tcp_height, check_unique_sim_graph
from fr_control.sim_grasp import create_sim_grasp
from fr_control.stage4_config import (
    block_pose,
    face_order,
    grasp_tcp_xyzw,
    joint_positions,
    load_yaml,
    robot_base_pose,
    validate_inspection_vectors,
)


@dataclass
class FaceReport:
    """Measured errors for one inspection face."""

    name: str
    position_target: tuple[float, float, float]
    position_actual: tuple[float, float, float]
    position_error_m: float
    face_error_deg: float
    up_error_deg: float
    slip_position_m: float
    slip_angle_deg: float
    passed: bool
    notes: list[str] = field(default_factory=list)


class Stage4InspectionNode(Node):
    """Run grasp + three-face inspection against MoveIt and Gazebo."""

    def __init__(self) -> None:
        """Load stage4_config.yaml and prepare TF."""
        super().__init__("fr_stage4_inspection_test")
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
                "stage4_config.yaml",
            )
        self._config = load_yaml(config_file)
        validate_inspection_vectors(self._config)
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._joint_state: JointState | None = None
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )
        self.get_logger().info(f"已加载阶段 4 配置：{config_file}")

    def run(self) -> bool:
        """Execute grasp and inspection. Return True if every check passed."""
        self._wait_for_clock()
        cfg = self._config
        planning_frame = str(cfg.get("planning_frame", BASE_FRAME))
        ee_link = str(cfg.get("ee_link", EE_LINK))
        attach_link = str(cfg.get("attach_link", ATTACH_LINK))
        grasp_cfg = cfg["grasp"]
        object_cfg = cfg["object"]
        table_cfg = cfg["table"]
        motion_cfg = cfg["motion"]
        inspection_cfg = cfg["inspection"]
        touch_links = list(cfg.get("gripper_touch_links", GRIPPER_TOUCH_LINKS))
        t_world_base = robot_base_pose(cfg)

        arm = MoveItArm(
            self,
            group_name=str(cfg.get("move_group", PLANNING_GROUP)),
            ee_link=ee_link,
            base_frame=str(cfg.get("base_frame", BASE_FRAME)),
            joint_names=ARM_JOINTS,
            velocity_scale=float(motion_cfg.get("velocity_scaling", 0.2)),
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
        self.get_logger().info(
            "固定工作站障碍物（column/table）由 workcell_scene_loader 加载；"
            "本节点只管理 object"
        )
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

        object_world = block_pose(object_cfg["initial_pose"])

        object_name = str(object_cfg["name"])
        sim_grasp.detach()
        self._wait_sim(0.3)
        try:
            set_model_pose(object_name, object_world)
            self.get_logger().info(
                f"已按 YAML 设置 Gazebo {object_name} "
                f"{format_pose(object_world)}"
            )
            self._wait_sim(0.4)
        except GazeboPoseError as exc:
            self.get_logger().warn(f"set_pose 跳过（{exc}），使用世界中的现有零件")

        object_pose = self._to_planning_frame(
            object_world,
            str(object_cfg["initial_pose"].get("frame", "world")),
            planning_frame,
            t_world_base,
        )
        poses = compute_grasp_poses(
            object_pose,
            approach_xyzw=grasp_tcp_xyzw(grasp_cfg),
            pre_grasp_offset=float(grasp_cfg["pregrasp_distance"]),
            lift_height=float(grasp_cfg["lift_distance"]),
            table_top_z=float(grasp_cfg["table_top_z"]),
            fingertip_from_tcp=float(grasp_cfg["fingertip_from_tcp"]),
            table_clearance=float(grasp_cfg["table_clearance"]),
            planning_frame=planning_frame,
        )
        object_size = list(as_vec3(object_cfg["dimensions"]))
        table_id = str(table_cfg["name"])

        self.get_logger().info(
            f"零件 {object_name} size={object_size} "
            f"{format_pose(poses.object_pose)}"
        )
        self.get_logger().info(f"pre_grasp {format_pose(poses.pre_grasp_pose)}")
        self.get_logger().info(f"grasp {format_pose(poses.grasp_pose)}")
        self.get_logger().info(f"lift {format_pose(poses.lift_pose)}")

        sim_grasp.detach()
        scene.detach(object_name, attach_link)
        scene.remove(object_name)
        scene.add_box(
            object_name,
            object_pose,
            object_size,
            planning_frame,
        )
        scene.allow_collisions(object_name, touch_links, allowed=True)
        scene.allow_collisions(object_name, [table_id], allowed=True)

        home = joint_positions(cfg)
        tcp_tol = float(motion_cfg.get("tcp_tolerance", 0.020))
        pose_attempts = int(motion_cfg.get("pose_attempts", 3))
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
            self._wait_sim(settle)

        self.get_logger().info("6. Attach object (MoveIt + optional Gazebo)")
        sim_grasp.attach()
        scene.attach_box(
            object_name,
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
        self._wait_sim(0.5)

        tcp_lift = self._lookup_tcp(planning_frame, ee_link)
        object_lift = self._gazebo_object_in_planning(
            object_name,
            planning_frame,
            t_world_base,
        )
        tcp_t_object = object_in_tcp(tcp_lift, object_lift)
        self.get_logger().info(
            f"Lift 后 T_tcp_object {format_pose(tcp_t_object)}"
        )
        table_top = float(grasp_cfg["table_top_z"])
        if object_lift.position.z < table_top + 0.02:
            raise MoveItError(
                "抬起后 Gazebo 零件仍在桌面附近，夹持失败。"
                f" object_z={object_lift.position.z:.4f} "
                f"table_top={table_top:.4f}。"
                "阶段 4 翻面不能在零件未离桌时继续。"
            )

        p1 = as_vec3(inspection_cfg["position"])
        p1_frame = str(inspection_cfg["position"].get("frame", planning_frame))
        if p1_frame != planning_frame:
            p1_pose = self._to_planning_frame(
                pose_from_rpy(p1, (0.0, 0.0, 0.0)),
                p1_frame,
                planning_frame,
                t_world_base,
            )
            p1 = (
                p1_pose.position.x,
                p1_pose.position.y,
                p1_pose.position.z,
            )
        direction = as_vec3(inspection_cfg["direction"])
        up_direction = as_vec3(inspection_cfg["up_direction"])
        hold = float(inspection_cfg.get("hold_seconds", 2.0))
        waypoint_count = int(inspection_cfg.get("face_waypoint_count", 12))

        object_targets: dict[str, Pose] = {}
        tcp_targets: dict[str, Pose] = {}
        for name in face_order(cfg):
            face = inspection_cfg["faces"][name]
            object_targets[name] = object_pose_for_face(
                p1,
                face_normal=as_vec3(face["normal_in_object"]),
                face_up=as_vec3(face["up_in_object"]),
                inspection_direction=direction,
                inspection_up=up_direction,
            )
            tcp_targets[name] = tcp_pose_from_object(
                object_targets[name], tcp_t_object
            )
            self.get_logger().info(
                f"{name} object {format_pose(object_targets[name])}"
            )
            self.get_logger().info(
                f"{name} tcp {format_pose(tcp_targets[name])}"
            )

        reports: list[FaceReport] = []
        previous_object = object_lift
        for name in face_order(cfg):
            self.get_logger().info(f"Move object to {name} at P1")
            self._move_object_path(
                arm,
                previous_object,
                object_targets[name],
                tcp_t_object,
                planning_frame,
                ee_link,
                grasp_cfg,
                tcp_tol,
                pose_attempts,
                waypoint_count,
                name,
            )
            if hold > 0.0:
                self.get_logger().info(f"{name} hold {hold:.1f}s")
                self._wait_sim(hold)
            reports.append(
                self._validate_face(
                    name=name,
                    object_name=object_name,
                    planning_frame=planning_frame,
                    t_world_base=t_world_base,
                    ee_link=ee_link,
                    target=object_targets[name],
                    tcp_t_object_lift=tcp_t_object,
                    face=inspection_cfg["faces"][name],
                    direction=direction,
                    up_direction=up_direction,
                )
            )
            previous_object = object_targets[name]

        passed = self._print_report(reports, cfg["validation"])
        return passed

    def _move_object_path(
        self,
        arm: MoveItArm,
        start_object: Pose,
        goal_object: Pose,
        tcp_t_object: Pose,
        frame_id: str,
        ee_link: str,
        grasp_cfg: dict[str, Any],
        tcp_tol: float,
        attempts: int,
        waypoint_count: int,
        name: str,
    ) -> None:
        """Keep the object center near P1 while changing orientation."""
        object_waypoints = interpolate_object_poses(
            start_object, goal_object, waypoint_count
        )
        tcp_waypoints = [
            tcp_pose_from_object(item, tcp_t_object)
            for item in object_waypoints
        ]
        eef_step = float(grasp_cfg.get("cartesian_eef_step", 0.01))
        fraction_min = float(grasp_cfg.get("cartesian_fraction_min", 0.85))
        try:
            arm.go_cartesian(
                tcp_waypoints,
                frame_id=frame_id,
                eef_step=eef_step,
                fraction_min=min(fraction_min, 0.5),
                avoid_collisions=True,
            )
            error = self._tcp_error(tcp_waypoints[-1], frame_id, ee_link)
            self.get_logger().info(
                f"{name} Cartesian TCP 误差 {error * 1000.0:.1f} mm"
            )
            if error <= tcp_tol:
                self._log_tf(frame_id, ee_link)
                return
        except MoveItError as exc:
            self.get_logger().warn(
                f"{name} Cartesian 失败（{exc}），改用不避障 Cartesian"
            )
            try:
                arm.go_cartesian(
                    tcp_waypoints,
                    frame_id=frame_id,
                    eef_step=eef_step,
                    fraction_min=0.4,
                    avoid_collisions=False,
                )
                error = self._tcp_error(tcp_waypoints[-1], frame_id, ee_link)
                self.get_logger().info(
                    f"{name} 无避障 Cartesian TCP 误差 {error * 1000.0:.1f} mm"
                )
                if error <= tcp_tol:
                    self._log_tf(frame_id, ee_link)
                    return
            except MoveItError as exc2:
                self.get_logger().warn(
                    f"{name} 无避障 Cartesian 失败（{exc2}），改用 Pose 规划"
                )
        self._go_pose_verified(
            arm,
            tcp_waypoints[-1],
            frame_id,
            ee_link,
            name,
            tcp_tol,
            attempts,
        )

    def _validate_face(
        self,
        *,
        name: str,
        object_name: str,
        planning_frame: str,
        t_world_base: Pose,
        ee_link: str,
        target: Pose,
        tcp_t_object_lift: Pose,
        face: dict[str, Any],
        direction: tuple[float, float, float],
        up_direction: tuple[float, float, float],
    ) -> FaceReport:
        """Compare Gazebo object pose against the commanded object pose."""
        actual = self._gazebo_object_in_planning(
            object_name, planning_frame, t_world_base
        )
        tcp = self._lookup_tcp(planning_frame, ee_link)
        current_rel = object_in_tcp(tcp, actual)
        slip_p = pose_position_error(current_rel, tcp_t_object_lift)
        slip_a = quat_angle_deg(
            current_rel.orientation, tcp_t_object_lift.orientation
        )
        normal = as_vec3(face["normal_in_object"])
        face_up = as_vec3(face["up_in_object"])
        actual_normal = rotate_pose_vector(actual, normal)
        actual_up = rotate_pose_vector(actual, face_up)
        desired_up = projected_up(up_direction, direction)
        measured_up = projected_up(actual_up, actual_normal)
        position_error = pose_position_error(actual, target)
        face_error = angle_between_deg(actual_normal, direction)
        up_error = angle_between_deg(measured_up, desired_up)
        report = FaceReport(
            name=name,
            position_target=(
                target.position.x,
                target.position.y,
                target.position.z,
            ),
            position_actual=(
                actual.position.x,
                actual.position.y,
                actual.position.z,
            ),
            position_error_m=position_error,
            face_error_deg=face_error,
            up_error_deg=up_error,
            slip_position_m=slip_p,
            slip_angle_deg=slip_a,
            passed=False,
        )
        self.get_logger().info(
            f"{name} Gazebo object {format_pose(actual)} "
            f"err={position_error * 1000.0:.1f} mm "
            f"face={face_error:.2f} deg up={up_error:.2f} deg "
            f"slip={slip_p * 1000.0:.1f} mm / {slip_a:.2f} deg"
        )
        return report

    def _print_report(
        self,
        reports: list[FaceReport],
        validation: dict[str, Any],
    ) -> bool:
        """Log the per-face report and return overall pass/fail."""
        max_p = float(validation["max_position_error_m"])
        max_face = float(validation["max_face_angle_error_deg"])
        max_up = float(validation["max_up_angle_error_deg"])
        max_slip_p = float(validation.get("max_slip_position_m", 0.005))
        max_slip_a = float(validation.get("max_slip_angle_deg", 8.0))
        overall = True
        lines = ["", "========== STAGE 4 INSPECTION REPORT =========="]
        for report in reports:
            pos_ok = report.position_error_m <= max_p
            face_ok = report.face_error_deg <= max_face
            up_ok = report.up_error_deg <= max_up
            slip_ok = (
                report.slip_position_m <= max_slip_p
                and report.slip_angle_deg <= max_slip_a
            )
            report.passed = pos_ok and face_ok and up_ok
            if not slip_ok:
                report.notes.append(
                    "零件相对 TCP 滑动超过阈值（物理夹持问题，未用改 TCP 掩盖）"
                )
            if not report.passed:
                overall = False
            if not slip_ok:
                overall = False
            verdict = "PASS" if report.passed and slip_ok else "FAIL"
            title = report.name.upper().replace("FACE", "FACE ")
            lines.extend(
                [
                    "",
                    title,
                    "Position target:",
                    "  "
                    f"xyz=({report.position_target[0]:.4f}, "
                    f"{report.position_target[1]:.4f}, "
                    f"{report.position_target[2]:.4f})",
                    "Position actual (Gazebo):",
                    "  "
                    f"xyz=({report.position_actual[0]:.4f}, "
                    f"{report.position_actual[1]:.4f}, "
                    f"{report.position_actual[2]:.4f})",
                    f"Position error: {report.position_error_m * 1000.0:.1f} mm",
                    f"Face direction error: {report.face_error_deg:.2f} deg",
                    f"Up direction error: {report.up_error_deg:.2f} deg",
                    "Relative object slip vs lift:",
                    f"  {report.slip_position_m * 1000.0:.1f} mm, "
                    f"{report.slip_angle_deg:.2f} deg",
                    verdict,
                ]
            )
            lines.extend(f"  NOTE: {note}" for note in report.notes)
        result = "PASS" if overall else "FAIL"
        lines.extend(["", f"STAGE 4 RESULT: {result}", ""])
        text = "\n".join(lines)
        self.get_logger().info(text)
        print(text, flush=True)
        return overall

    def _gazebo_object_in_planning(
        self,
        object_name: str,
        planning_frame: str,
        t_world_base: Pose,
    ) -> Pose:
        """Read the Gazebo model pose and express it in the planning frame."""
        last_error: Exception | None = None
        world_frame = str(self._config.get("frames", {}).get("world", "world"))
        for _ in range(5):
            try:
                world_pose = read_model_pose(object_name)
                return self._to_planning_frame(
                    world_pose,
                    world_frame,
                    planning_frame,
                    t_world_base,
                )
            except GazeboPoseError as exc:
                last_error = exc
                self._wait_sim(0.2)
        raise MoveItError(f"无法读取 Gazebo {object_name} 位姿：{last_error}")

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
            self.get_logger().warn(f"Cartesian 失败（{exc}），尝试无避障 Cartesian")
            try:
                arm.go_cartesian(
                    [goal],
                    frame_id=frame_id,
                    eef_step=eef_step,
                    fraction_min=0.5,
                    avoid_collisions=False,
                )
                error = self._tcp_error(goal, frame_id, ee_link)
                if error <= tcp_tol:
                    self._log_tf(frame_id, ee_link)
                    return
            except MoveItError as exc2:
                self.get_logger().warn(
                    f"无避障 Cartesian 失败（{exc2}），改用 Pose 规划"
                )
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

    def _to_planning_frame(
        self,
        pose: Pose,
        source_frame: str,
        planning_frame: str,
        t_world_base: Pose,
    ) -> Pose:
        """Transform a Pose into the MoveIt planning frame."""
        if source_frame == planning_frame:
            return pose
        if {source_frame, planning_frame} <= {"world", "base_link"}:
            if source_frame == "world" and planning_frame == "base_link":
                return pose_multiply(pose_inverse(t_world_base), pose)
            if source_frame == "base_link" and planning_frame == "world":
                return pose_multiply(t_world_base, pose)
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
                    "使用 YAML robot.base_pose"
                )
                if source_frame == "world":
                    return pose_multiply(pose_inverse(t_world_base), pose)
                return pose_multiply(t_world_base, pose)
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
        value = self.get_parameter(name).get_parameter_value().string_value
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

    def _wait_sim(self, seconds: float) -> None:
        """Wait until the Gazebo clock advances by the given seconds."""
        start = self.get_clock().now()
        wall_deadline = time.monotonic() + max(60.0, float(seconds) * 120.0)
        while rclpy.ok() and time.monotonic() < wall_deadline:
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed >= seconds:
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        raise MoveItError(f"等待仿真 {seconds:.2f}s 超时")


def main(args=None) -> None:
    """ROS 2 entry point; exits after the inspection sequence finishes."""
    rclpy.init(args=args)
    node = Stage4InspectionNode()
    passed = False
    try:
        passed = node.run()
    except KeyboardInterrupt:
        node.get_logger().info("已中断")
    except (
        MoveItError,
        GripperError,
        TransformException,
        InspectionError,
        GazeboPoseError,
    ) as exc:
        node.get_logger().error(f"阶段 4 测试失败：{exc}")
        print("\nSTAGE 4 RESULT: FAIL\n", flush=True)
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
