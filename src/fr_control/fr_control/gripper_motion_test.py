"""
Unified gripper motion test for Gazebo and real HKV TG-9801.

Task code is the same. Launch / ROS parameters choose the backend:

  backend:=gazebo  -> GazeboGripper
  backend:=real    -> RealFairinoGripper

Does not move the FR3 arm. Does not reset the gripper unless asked.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from fr_control.gripper import GripperError, create_gripper


class GripperMotionTestNode(Node):
    """Run open/close on whichever gripper backend the factory returns."""

    def __init__(self) -> None:
        """Declare backend, dry_run, and sequence parameters."""
        super().__init__("fr_gripper_motion_test")
        self.declare_parameter("mode", "")
        self.declare_parameter("backend", "")
        self.declare_parameter("dry_run", False)
        self.declare_parameter("sequence", "full")
        self.declare_parameter("do_activate", False)
        self.declare_parameter("do_reset", False)
        self.declare_parameter("hold_seconds", 2.0)
        self.declare_parameter("skip_confirm", False)
        self.declare_parameter("gripper_config_file", "")
        self.declare_parameter("pre_move_delay_sec", 3.0)
        self._joint_state: JointState | None = None
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )

    def run(self) -> None:
        """Create the backend, optionally probe it, then run the sequence."""
        self._wait_for_clock()
        backend = self._backend()
        dry_run = self._bool_param("dry_run")
        sequence = self._string_param("sequence", "full").strip().lower()
        do_activate = self._bool_param("do_activate")
        do_reset = self._bool_param("do_reset")
        hold = float(
            self.get_parameter("hold_seconds").get_parameter_value().double_value
        )
        skip_confirm = self._bool_param("skip_confirm")

        self.get_logger().info(
            f"夹爪测试 backend={backend} dry_run={dry_run} "
            f"sequence={sequence} activate={do_activate} reset={do_reset}"
        )
        self._log_joint_snapshot("创建 backend 之前")

        gripper = create_gripper(
            self,
            backend=backend,
            dry_run=dry_run,
            config_file=self._string_param("gripper_config_file", ""),
        )
        info = gripper.describe()
        self.get_logger().info(f"夹爪配置：{info}")

        gripper.check_connection()
        self._log_joint_snapshot("通信检查之后")

        steps = self._steps(gripper, info, sequence, do_activate, do_reset)
        for name, _action in steps:
            self.get_logger().info(f"计划步骤：{name}")

        if dry_run:
            self.get_logger().info(
                "dry_run=true：已创建 backend、检查配置/通信，不发送夹爪运动"
            )
            return

        if backend == "real" and not skip_confirm:
            self.get_logger().info(
                "真机夹爪即将运动。确认周围安全后，在此终端按 Enter。"
                "若要跳过确认，使用 skip_confirm:=true"
            )
            try:
                input("按 Enter 继续真机夹爪测试...")
            except EOFError as exc:
                raise GripperError(
                    "无法读取确认输入。请用 skip_confirm:=true 或在 TTY 中运行"
                ) from exc

        delay = float(
            self.get_parameter("pre_move_delay_sec")
            .get_parameter_value()
            .double_value
        )
        if delay > 0.0:
            self.get_logger().info(f"运动前等待 {delay:.1f}s")
            time.sleep(delay)

        for name, action in steps:
            self.get_logger().info(f"夹爪步骤：{name}")
            action()
            self._log_joint_snapshot(name)
            if hold > 0.0:
                time.sleep(hold)

        self.get_logger().info("夹爪测试完成")

    def _backend(self) -> str:
        """Resolve backend from backend= or mode=. Tasks must not do this."""
        backend = self._string_param("backend", "").strip().lower()
        if backend:
            return backend
        mode = self._string_param("mode", "").strip().lower()
        if mode in ("real", "hardware", "fairino"):
            return "real"
        if mode in ("sim", "gazebo", "simulation"):
            return "gazebo"
        return "gazebo"

    def _steps(
        self,
        gripper,
        info: dict,
        sequence: str,
        do_activate: bool,
        do_reset: bool,
    ):
        """Build the explicit command list. reset/activate are never implied."""
        steps = []
        if do_reset:
            steps.append(("RESET", gripper.reset))
        if do_activate:
            steps.append(("ACTIVATE", gripper.activate))
        if sequence in ("conservative", "small"):
            open_pos = info.get("open_position", 0.0)
            mid = info.get("conservative_position", 0.02)
            vel = info.get("conservative_velocity")
            force = info.get("conservative_force")
            steps.extend(
                (
                    (
                        f"OPEN {open_pos}",
                        lambda: gripper.move(
                            open_pos, velocity=vel, force=force
                        ),
                    ),
                    (
                        f"CONSERVATIVE {mid}",
                        lambda: gripper.move(mid, velocity=vel, force=force),
                    ),
                    (
                        f"OPEN {open_pos}",
                        lambda: gripper.move(
                            open_pos, velocity=vel, force=force
                        ),
                    ),
                )
            )
            return steps
        steps.extend(
            (
                ("OPEN", gripper.open),
                ("CLOSE", gripper.close),
                ("OPEN", gripper.open),
            )
        )
        return steps

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
        raise GripperError(
            "未收到 Gazebo /clock。请先启动仿真，并保持 use_sim_time:=true"
        )

    def _log_joint_snapshot(self, label: str) -> None:
        """Print a short /joint_states sample so arm control can be watched."""
        rclpy.spin_once(self, timeout_sec=0.05)
        msg = self._joint_state
        if msg is None:
            self.get_logger().info(f"/joint_states [{label}]: 尚未收到")
            return
        names = list(msg.name)
        positions = list(msg.position)
        arm = []
        for joint in ("j1", "j2", "j3", "j4", "j5", "j6"):
            if joint in names:
                arm.append(f"{joint}={positions[names.index(joint)]:.4f}")
        self.get_logger().info(
            f"/joint_states [{label}] n={len(names)} " + " ".join(arm)
        )

    def _on_joint_state(self, msg: JointState) -> None:
        """Cache the latest joint states."""
        self._joint_state = msg

    def _string_param(self, name: str, default: str) -> str:
        """Read a string ROS parameter."""
        value = self.get_parameter(name).get_parameter_value().string_value
        return default if value == "" else value

    def _bool_param(self, name: str) -> bool:
        """Read a bool ROS parameter."""
        return bool(self.get_parameter(name).get_parameter_value().bool_value)


def main(args=None) -> None:
    """ROS 2 entry point; exits after the gripper sequence finishes."""
    rclpy.init(args=args)
    node = GripperMotionTestNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("已中断")
    except Exception as exc:
        node.get_logger().error(f"夹爪测试失败：{exc}")
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
