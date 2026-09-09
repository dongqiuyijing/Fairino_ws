"""
Minimal Gazebo gripper test: open -> close -> open, plus one mid pose.

Does not move the FR3 arm. Gazebo must already be running.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from fr_control.constants import GRIPPER_CLOSED, GRIPPER_OPEN
from fr_control.gripper import GripperError, create_gripper


class GripperTestNode(Node):
    """Run a short open/close sequence on the Gazebo gripper."""

    def __init__(self) -> None:
        """Declare backend and timing parameters."""
        super().__init__("fr_gripper_test")
        self.declare_parameter("backend", "gazebo")
        self.declare_parameter("hold_seconds", 2.0)
        self.declare_parameter("mid_position", 0.05)

    def run(self) -> None:
        """Wait for sim clock, then execute the gripper sequence."""
        self._wait_for_clock()
        backend = (
            self.get_parameter("backend")
            .get_parameter_value()
            .string_value
        )
        hold = float(
            self.get_parameter("hold_seconds")
            .get_parameter_value()
            .double_value
        )
        mid = float(
            self.get_parameter("mid_position")
            .get_parameter_value()
            .double_value
        )
        gripper = create_gripper(self, backend=backend)

        steps = (
            ("OPEN", lambda: gripper.open()),
            ("CLOSE", lambda: gripper.close()),
            ("OPEN", lambda: gripper.open()),
            (f"MID {mid:.3f}", lambda: gripper.move(mid)),
            ("OPEN", lambda: gripper.open()),
        )
        for name, action in steps:
            self.get_logger().info(f"夹爪步骤：{name}")
            action()
            if hold > 0.0:
                time.sleep(hold)

        self.get_logger().info(
            f"夹爪测试完成：open={GRIPPER_OPEN} closed={GRIPPER_CLOSED}"
        )

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


def main(args=None) -> None:
    """ROS 2 entry point; exits after the gripper sequence finishes."""
    rclpy.init(args=args)
    node = GripperTestNode()
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
