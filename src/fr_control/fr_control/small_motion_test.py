"""
Minimal Sim/Real shared motion test for FAIRINO FR3.

The task code does NOT know whether the backend is Gazebo or the real robot.

Sequence:
    current TCP pose
        -> world +Z by 5 mm
        -> hold 1 s
        -> return to the exact start pose

Safety:
- reads the current TCP from TF at runtime
- never moves to a hard-coded Home
- velocity / acceleration scaling = 5 %
- maximum permitted test displacement = 10 mm
- Cartesian path keeps collision checking enabled
- verifies the actual TCP after both motions
"""

from __future__ import annotations

import math
import time

from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from fr_control.constants import (
    ARM_JOINTS,
    BASE_FRAME,
    EE_LINK,
    PLANNING_GROUP,
)
from fr_control.moveit_arm import MoveItArm, MoveItError


class SmallMotionTestNode(Node):
    """Move the current TCP +Z in world by a few millimetres and return."""

    def __init__(self) -> None:
        super().__init__("fr_small_motion_test")

        # Do NOT declare use_sim_time here.
        # ROS 2 handles it when passed from launch / --ros-args.

        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_frame", BASE_FRAME)
        self.declare_parameter("ee_link", EE_LINK)
        self.declare_parameter("move_group", PLANNING_GROUP)

        # Test motion
        self.declare_parameter("distance_m", 0.005)
        self.declare_parameter("hold_seconds", 1.0)

        # Conservative real-robot settings
        self.declare_parameter("velocity_scaling", 0.05)
        self.declare_parameter("acceleration_scaling", 0.05)

        # Planning
        self.declare_parameter("planning_time", 5.0)
        self.declare_parameter("planning_attempts", 10)
        self.declare_parameter("position_tolerance", 0.0015)
        self.declare_parameter("orientation_tolerance", 0.05)

        # Cartesian path
        self.declare_parameter("eef_step", 0.001)
        self.declare_parameter("cartesian_fraction_min", 0.95)

        # Safety checks
        self.declare_parameter("max_test_distance_m", 0.010)
        self.declare_parameter("verify_position_tolerance_m", 0.003)
        self.declare_parameter("verify_orientation_tolerance_deg", 5.0)
        self.declare_parameter("tf_timeout_sec", 10.0)

        # Gives the operator time to stop the node before motion begins.
        self.declare_parameter("pre_move_delay_sec", 3.0)

        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)

    def run(self) -> None:
        world_frame = str(self.get_parameter("world_frame").value)
        base_frame = str(self.get_parameter("base_frame").value)
        ee_link = str(self.get_parameter("ee_link").value)
        move_group = str(self.get_parameter("move_group").value)

        distance = float(self.get_parameter("distance_m").value)
        hold_seconds = float(self.get_parameter("hold_seconds").value)

        velocity = float(self.get_parameter("velocity_scaling").value)
        acceleration = float(
            self.get_parameter("acceleration_scaling").value
        )

        max_distance = float(
            self.get_parameter("max_test_distance_m").value
        )

        verify_position_tol = float(
            self.get_parameter("verify_position_tolerance_m").value
        )

        verify_orientation_tol_deg = float(
            self.get_parameter(
                "verify_orientation_tolerance_deg"
            ).value
        )

        pre_move_delay = float(
            self.get_parameter("pre_move_delay_sec").value
        )

        # ----------------------------------------------------------
        # Safety validation
        # ----------------------------------------------------------

        if distance <= 0.0:
            raise MoveItError(
                f"distance_m 必须 > 0，当前为 {distance}"
            )

        if distance > max_distance:
            raise MoveItError(
                "拒绝执行：测试位移超过安全上限。"
                f" distance={distance:.4f} m,"
                f" max={max_distance:.4f} m"
            )

        if not (0.0 < velocity <= 0.10):
            raise MoveItError(
                f"velocity_scaling 不安全：{velocity}"
            )

        if not (0.0 < acceleration <= 0.10):
            raise MoveItError(
                f"acceleration_scaling 不安全：{acceleration}"
            )

        # ----------------------------------------------------------
        # Connect to the already-running MoveIt
        # ----------------------------------------------------------

        arm = MoveItArm(
            self,
            group_name=move_group,
            ee_link=ee_link,
            base_frame=base_frame,
            joint_names=ARM_JOINTS,
            velocity_scale=velocity,
            acceleration_scale=acceleration,
            planning_time=float(
                self.get_parameter("planning_time").value
            ),
            planning_attempts=int(
                self.get_parameter("planning_attempts").value
            ),
            position_tolerance=float(
                self.get_parameter("position_tolerance").value
            ),
            orientation_tolerance=float(
                self.get_parameter("orientation_tolerance").value
            ),
        )

        self.get_logger().info("等待 MoveIt...")
        arm.wait_until_ready()
        self.get_logger().info("MoveIt 已就绪")

        # ----------------------------------------------------------
        # Read the REAL CURRENT TCP pose.
        #
        # No hard-coded Home and no hard-coded SDK TCP value.
        # ----------------------------------------------------------

        start_pose = self._lookup_pose(world_frame, ee_link)

        self.get_logger().info(
            "启动时 TCP (world): "
            f"x={start_pose.position.x:.6f}, "
            f"y={start_pose.position.y:.6f}, "
            f"z={start_pose.position.z:.6f}"
        )

        self.get_logger().info(
            "启动时 TCP quaternion: "
            f"x={start_pose.orientation.x:.6f}, "
            f"y={start_pose.orientation.y:.6f}, "
            f"z={start_pose.orientation.z:.6f}, "
            f"w={start_pose.orientation.w:.6f}"
        )

        # ----------------------------------------------------------
        # Target = current pose + world Z 5 mm.
        #
        # Orientation remains exactly unchanged.
        # ----------------------------------------------------------

        target_pose = _copy_pose(start_pose)
        target_pose.position.z += distance

        displacement = _position_error(start_pose, target_pose)

        if displacement > max_distance + 1e-9:
            raise MoveItError(
                "内部安全检查失败：生成的目标位移过大。"
            )

        self.get_logger().info(
            f"测试目标：沿 {world_frame} +Z "
            f"{distance * 1000.0:.1f} mm"
        )

        self.get_logger().info(
            "目标 TCP: "
            f"x={target_pose.position.x:.6f}, "
            f"y={target_pose.position.y:.6f}, "
            f"z={target_pose.position.z:.6f}"
        )

        # ----------------------------------------------------------
        # Operator safety countdown
        # ----------------------------------------------------------

        if pre_move_delay > 0.0:
            self.get_logger().warn(
                f"{pre_move_delay:.1f} 秒后开始运动。"
                "请确认工作空间安全、急停可触及。"
            )
            time.sleep(pre_move_delay)

        # ----------------------------------------------------------
        # Motion 1: world +Z 5 mm
        # ----------------------------------------------------------

        self.get_logger().info(
            "STEP 1/3：沿 world +Z 移动"
        )

        self._cartesian_verified(
            arm=arm,
            goal=target_pose,
            frame_id=world_frame,
            ee_link=ee_link,
            position_tolerance=verify_position_tol,
            orientation_tolerance_deg=verify_orientation_tol_deg,
        )

        # ----------------------------------------------------------
        # Hold
        # ----------------------------------------------------------

        self.get_logger().info(
            f"STEP 2/3：保持 {hold_seconds:.1f} 秒"
        )

        if hold_seconds > 0.0:
            self._spin_for(hold_seconds)

        # ----------------------------------------------------------
        # Motion 2: return to exact start pose
        # ----------------------------------------------------------

        self.get_logger().info(
            "STEP 3/3：返回启动位置"
        )

        self._cartesian_verified(
            arm=arm,
            goal=start_pose,
            frame_id=world_frame,
            ee_link=ee_link,
            position_tolerance=verify_position_tol,
            orientation_tolerance_deg=verify_orientation_tol_deg,
        )

        final_pose = self._lookup_pose(world_frame, ee_link)

        return_error = _position_error(final_pose, start_pose)
        return_angle = _orientation_error_deg(
            final_pose,
            start_pose,
        )

        self.get_logger().info(
            "========== SMALL MOTION TEST PASS =========="
        )

        self.get_logger().info(
            f"返回位置误差：{return_error * 1000.0:.2f} mm"
        )

        self.get_logger().info(
            f"返回姿态误差：{return_angle:.3f} deg"
        )

    def _cartesian_verified(
        self,
        *,
        arm: MoveItArm,
        goal: Pose,
        frame_id: str,
        ee_link: str,
        position_tolerance: float,
        orientation_tolerance_deg: float,
    ) -> None:
        """Execute a collision-checked Cartesian move and verify with TF."""

        eef_step = float(self.get_parameter("eef_step").value)
        fraction_min = float(
            self.get_parameter("cartesian_fraction_min").value
        )

        move_error: MoveItError | None = None

        try:
            arm.go_cartesian(
                [goal],
                frame_id=frame_id,
                eef_step=eef_step,
                fraction_min=fraction_min,
                avoid_collisions=True,
            )

        except MoveItError as exc:
            # Sometimes a controller can report failure very near the goal.
            # We never fall back to collision-disabled motion.
            move_error = exc

        deadline = time.monotonic() + 2.0

        actual = None
        pos_error = float("inf")
        angle_error = float("inf")

        while rclpy.ok() and time.monotonic() < deadline:

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            actual = self._lookup_pose(
                frame_id,
                ee_link,
            )

            pos_error = _position_error(
                actual,
                goal,
            )

            angle_error = _orientation_error_deg(
                actual,
                goal,
            )

            if (
                pos_error <= position_tolerance
                and
                angle_error <= orientation_tolerance_deg
            ):
                break

        self.get_logger().info(
            "TCP 验证："
            f"位置误差={pos_error * 1000.0:.2f} mm, "
            f"姿态误差={angle_error:.3f} deg"
        )

        if (
            pos_error <= position_tolerance
            and angle_error <= orientation_tolerance_deg
        ):
            if move_error is not None:
                self.get_logger().warn(
                    "控制器/MoveIt 报告失败，但 TF 显示 TCP 已到位；"
                    f"原错误：{move_error}"
                )
            return

        if move_error is not None:
            raise move_error

        raise MoveItError(
            "TCP 未达到测试目标："
            f"位置误差={pos_error * 1000.0:.2f} mm, "
            f"姿态误差={angle_error:.3f} deg"
        )

    def _spin_for(self, seconds: float) -> None:
        """Process ROS callbacks for a wall-clock duration."""

        deadline = time.monotonic() + seconds

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _lookup_pose(
        self,
        target_frame: str,
        source_frame: str,
    ) -> Pose:
        """Read source_frame pose expressed in target_frame."""

        timeout = float(
            self.get_parameter("tf_timeout_sec").value
        )

        deadline = time.monotonic() + timeout
        last_error = "unknown"

        while rclpy.ok() and time.monotonic() < deadline:
            try:
                transform = self._tf.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time(),
                )

                pose = Pose()

                pose.position.x = transform.transform.translation.x
                pose.position.y = transform.transform.translation.y
                pose.position.z = transform.transform.translation.z

                pose.orientation.x = transform.transform.rotation.x
                pose.orientation.y = transform.transform.rotation.y
                pose.orientation.z = transform.transform.rotation.z
                pose.orientation.w = transform.transform.rotation.w

                return pose

            except TransformException as exc:
                last_error = str(exc)
                rclpy.spin_once(self, timeout_sec=0.1)

        raise MoveItError(
            f"等待 TF {target_frame} -> {source_frame} 超时："
            f"{last_error}"
        )


def _copy_pose(source: Pose) -> Pose:
    """Return an independent copy of a Pose."""

    result = Pose()

    result.position.x = source.position.x
    result.position.y = source.position.y
    result.position.z = source.position.z

    result.orientation.x = source.orientation.x
    result.orientation.y = source.orientation.y
    result.orientation.z = source.orientation.z
    result.orientation.w = source.orientation.w

    return result


def _position_error(a: Pose, b: Pose) -> float:
    """Euclidean XYZ distance."""

    dx = a.position.x - b.position.x
    dy = a.position.y - b.position.y
    dz = a.position.z - b.position.z

    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _orientation_error_deg(a: Pose, b: Pose) -> float:
    """Shortest quaternion angular difference in degrees."""

    dot = (
        a.orientation.x * b.orientation.x
        + a.orientation.y * b.orientation.y
        + a.orientation.z * b.orientation.z
        + a.orientation.w * b.orientation.w
    )

    dot = min(1.0, max(-1.0, abs(dot)))

    angle_rad = 2.0 * math.acos(dot)

    return math.degrees(angle_rad)

def _spin_for(self, seconds: float) -> None:
    """Process ROS callbacks for a wall-clock duration."""

    deadline = time.monotonic() + seconds

    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(self, timeout_sec=0.05)

def main(args=None) -> None:
    rclpy.init(args=args)

    node = SmallMotionTestNode()

    try:
        node.run()

    except KeyboardInterrupt:
        node.get_logger().warn(
            "测试被人工中断。不会自动发送额外运动。"
        )

    except Exception as exc:
        node.get_logger().error(
            f"Small motion test FAILED：{exc}"
        )
        raise

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()