"""
Gazebo-only grasp assist.

MoveIt attach and Gazebo physics are separate. This module only welds the
part in Gazebo after the gripper has closed. Task code must not use these
topics on a real robot: pass backend='none'.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
import time

from rclpy.node import Node
from std_msgs.msg import Empty

from fr_control.constants import GRASP_ATTACH_TOPIC, GRASP_DETACH_TOPIC


class SimGraspAssist(ABC):
    """Simulation grasp weld. Real hardware uses the no-op backend."""

    @abstractmethod
    def attach(self) -> None:
        """Weld the grasped part to the robot in simulation."""

    @abstractmethod
    def detach(self) -> None:
        """Release the simulated weld so the part is free again."""


class NoOpSimGrasp(SimGraspAssist):
    """Backend for a real robot: does not publish Gazebo topics."""

    def __init__(self, node: Node) -> None:
        """Store the node for log messages."""
        self._node = node

    def attach(self) -> None:
        """Skip Gazebo attach on real hardware."""
        self._node.get_logger().info("SimGrasp 后端 none：跳过 Gazebo attach")

    def detach(self) -> None:
        """Skip Gazebo detach on real hardware."""
        self._node.get_logger().info("SimGrasp 后端 none：跳过 Gazebo detach")


class GazeboDetachableJointAssist(SimGraspAssist):
    """
    Ignition DetachableJoint weld.

    This is a simulation crutch: Fortress starts attached, and small-part
    contact at 10 ms is often too unstable to lift by friction alone.
    """

    def __init__(
        self,
        node: Node,
        *,
        attach_topic: str = GRASP_ATTACH_TOPIC,
        detach_topic: str = GRASP_DETACH_TOPIC,
    ) -> None:
        """Create publishers for the existing ros_gz_bridge topics."""
        self._node = node
        self._attach = node.create_publisher(Empty, attach_topic, 10)
        self._detach = node.create_publisher(Empty, detach_topic, 10)

    def attach(self) -> None:
        """Publish attach after the gripper has already closed."""
        self._node.get_logger().info(
            "Gazebo DetachableJoint attach（仿真抓取辅助，非真机接口）"
        )
        self._burst(self._attach)

    def detach(self) -> None:
        """Publish detach so the part is not welded at task start."""
        self._node.get_logger().info("Gazebo DetachableJoint detach")
        self._burst(self._detach)

    def _burst(self, publisher, count: int = 4) -> None:
        """Publish several Empty messages so the bridge does not miss one."""
        for _ in range(count):
            publisher.publish(Empty())
            time.sleep(0.05)


def create_sim_grasp(node: Node, backend: str = "gazebo") -> SimGraspAssist:
    """Build a sim-grasp backend. Task nodes should call this factory."""
    name = str(backend).strip().lower()
    if name in ("gazebo", "sim", "simulation"):
        return GazeboDetachableJointAssist(node)
    if name in ("none", "off", "real", "hardware"):
        return NoOpSimGrasp(node)
    raise ValueError(
        f"未知 sim_grasp backend：{backend}。可选 gazebo 或 none"
    )
