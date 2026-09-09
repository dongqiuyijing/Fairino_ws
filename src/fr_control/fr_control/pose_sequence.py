"""
FR3 pose sequence task: Home -> Pose A -> Pose B -> Home.

Does not use the RViz green interactive marker.
Pose numbers live in config/poses.yaml; change data, not code.
"""

from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass
from typing import Any

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
import yaml

from fr_control.constants import (
    ARM_JOINTS,
    BASE_FRAME,
    EE_LINK,
    PLANNING_GROUP,
)
from fr_control.moveit_arm import MoveItArm, MoveItError, make_pose


@dataclass
class MotionStep:
    """One sequence step: joint target or Cartesian Pose."""

    name: str
    kind: str
    joints: list[float] | None = None
    pose: Pose | None = None
    frame_id: str = BASE_FRAME


class PoseSequenceNode(Node):
    """Load pose config and send the sequence to MoveIt."""

    def __init__(self) -> None:
        """Read poses.yaml and parse the motion sequence."""
        super().__init__("fr_pose_sequence")
        # Humble 会在 Node 构造时自动声明 use_sim_time；launch 再传入后
        # 这里不能 declare，否则会 ParameterAlreadyDeclaredException。
        self.declare_parameter("poses_file", "")

        poses_file = (
            self.get_parameter("poses_file")
            .get_parameter_value()
            .string_value
        )
        if not poses_file:
            poses_file = os.path.join(
                get_package_share_directory("fr_control"),
                "config",
                "poses.yaml",
            )
        self._config = _load_yaml(poses_file)
        self._steps = _parse_sequence(self._config)
        self.get_logger().info(f"已加载姿态文件：{poses_file}")

    def run(self) -> None:
        """Wait for sim clock and MoveIt, then run the sequence."""
        self._wait_for_clock()
        cfg = self._config
        arm = MoveItArm(
            self,
            group_name=str(cfg.get("move_group", PLANNING_GROUP)),
            ee_link=str(cfg.get("ee_link", EE_LINK)),
            base_frame=str(cfg.get("base_frame", BASE_FRAME)),
            joint_names=ARM_JOINTS,
            velocity_scale=float(cfg.get("velocity_scale", 0.2)),
            acceleration_scale=float(cfg.get("acceleration_scale", 0.2)),
            planning_time=float(cfg.get("planning_time", 5.0)),
            planning_attempts=int(cfg.get("planning_attempts", 10)),
            position_tolerance=float(cfg.get("position_tolerance", 0.005)),
            orientation_tolerance=float(
                cfg.get("orientation_tolerance", 0.05)
            ),
        )
        arm.wait_until_ready()

        settle = float(cfg.get("settle_seconds", 0.5))
        total = len(self._steps)
        for index, step in enumerate(self._steps, start=1):
            self.get_logger().info(
                f"步骤 {index}/{total}：{step.name} ({step.kind})"
            )
            if step.kind == "joints":
                arm.go_joints(step.joints)
            else:
                arm.go_pose(step.pose, frame_id=step.frame_id)
            if settle > 0.0:
                time.sleep(settle)

        self.get_logger().info("序列完成：Home → Pose A → Pose B → Home")

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


def _load_yaml(path: str) -> dict[str, Any]:
    """Load a YAML pose file."""
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"姿态文件格式错误：{path}")
    return data


def _parse_sequence(config: dict[str, Any]) -> list[MotionStep]:
    """Turn the YAML sequence list into MotionStep objects."""
    poses = config.get("poses") or {}
    names = config.get("sequence") or []
    if not names:
        raise ValueError("poses.yaml 中的 sequence 为空")
    steps: list[MotionStep] = []
    for name in names:
        spec = poses.get(name)
        if spec is None:
            raise KeyError(f"sequence 引用了未定义的姿态：{name}")
        steps.append(_parse_step(name, spec, config))
    return steps


def _parse_step(
    name: str,
    spec: dict[str, Any],
    config: dict[str, Any],
) -> MotionStep:
    """Parse one named pose entry."""
    kind = str(spec.get("type", "")).strip().lower()
    frame_id = str(spec.get("frame", config.get("base_frame", BASE_FRAME)))
    if kind == "joints":
        joints = [float(value) for value in spec["joints"]]
        if len(joints) != len(ARM_JOINTS):
            raise ValueError(f"{name} 的 joints 数量必须为 {len(ARM_JOINTS)}")
        return MotionStep(name=name, kind="joints", joints=joints)
    if kind == "pose":
        xyz = [float(value) for value in spec["position"]]
        if "orientation_xyzw" in spec:
            xyzw = [float(value) for value in spec["orientation_xyzw"]]
        elif "orientation_rpy" in spec:
            xyzw = list(_rpy_to_xyzw(*spec["orientation_rpy"]))
        else:
            raise ValueError(
                f"{name} 需要 orientation_xyzw 或 orientation_rpy"
            )
        return MotionStep(
            name=name,
            kind="pose",
            pose=make_pose(xyz, xyzw),
            frame_id=frame_id,
        )
    raise ValueError(f"{name} 的 type 必须是 joints 或 pose，实际为 {kind}")


def _rpy_to_xyzw(roll: float, pitch: float, yaw: float) -> tuple[float, ...]:
    """Convert RPY to the xyzw quaternion used by geometry_msgs."""
    cr = math.cos(float(roll) * 0.5)
    sr = math.sin(float(roll) * 0.5)
    cp = math.cos(float(pitch) * 0.5)
    sp = math.sin(float(pitch) * 0.5)
    cy = math.cos(float(yaw) * 0.5)
    sy = math.sin(float(yaw) * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def main(args=None) -> None:
    """ROS 2 entry point; exits after the sequence finishes."""
    rclpy.init(args=args)
    node = PoseSequenceNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("已中断")
    except Exception as exc:
        node.get_logger().error(f"序列失败：{exc}")
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
