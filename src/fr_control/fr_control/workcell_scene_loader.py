"""
Load fixed workcell collision geometry into the MoveIt Planning Scene.

Reads the same YAML used by Gazebo (stage4_world.py). Default obstacles
are the mounting column and table. Task objects are not loaded here.
"""

from __future__ import annotations

import time
from typing import Any, Sequence

from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from fr_control.constants import BASE_FRAME
from fr_control.grasp_poses import apply_transform, pose_inverse, pose_multiply
from fr_control.inspection_poses import as_vec3
from fr_control.moveit_arm import MoveItError
from fr_control.planning_scene import PlanningSceneClient
from fr_control.stage4_config import (
    block_pose,
    load_yaml,
    robot_base_pose,
    workcell_config_path,
)


# Default YAML keys for static box obstacles. Add new workcell boxes here
# (inspection platform, camera support, fences) without new add_* helpers.
DEFAULT_STATIC_KEYS = ("column", "table")


class WorkcellSceneLoader(Node):
    """Publish YAML-defined static boxes into MoveIt; do not move the robot."""

    def __init__(self) -> None:
        """Load YAML and create TF / PlanningScene clients."""
        super().__init__("workcell_scene_loader")
        self.declare_parameter("config_file", "")
        self.declare_parameter("include_object", False)
        self.declare_parameter("static_keys", list(DEFAULT_STATIC_KEYS))

        config_file = (
            self.get_parameter("config_file")
            .get_parameter_value()
            .string_value
        )
        if not config_file:
            config_file = workcell_config_path()
        self._config_file = config_file
        self._config = load_yaml(config_file)
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self.get_logger().info(f"已加载工作站配置：{config_file}")

    def load_scene(self) -> list[str]:
        """Wait for MoveIt, then add static boxes. Return loaded object ids."""
        self._wait_for_clock()
        cfg = self._config
        planning_frame = str(cfg.get("planning_frame", BASE_FRAME))
        t_world_base = robot_base_pose(cfg)
        world_frame = str(cfg.get("frames", {}).get("world", "world"))

        scene = PlanningSceneClient(self)
        scene.wait_until_ready(timeout_sec=90.0)
        self._wait_for_tf(planning_frame, world_frame)

        loaded: list[str] = []
        for key, block in self._static_box_blocks():
            object_id, world_pose, size, source_frame, touch_links = (
                _box_from_yaml(block)
            )
            pose = self._to_planning_frame(
                world_pose,
                source_frame,
                planning_frame,
                t_world_base,
            )
            _add_static_box(
                scene,
                object_id=object_id,
                pose=pose,
                size=size,
                frame_id=planning_frame,
                touch_links=touch_links,
            )
            loaded.append(object_id)
            self.get_logger().info(
                f"已加载 {key} id={object_id} "
                f"size=({size[0]:.3f}, {size[1]:.3f}, {size[2]:.3f}) "
                f"{source_frame} -> {planning_frame}"
            )

        names = "\n".join(f"  {name}" for name in loaded)
        self.get_logger().info(
            "[workcell_scene_loader] Loaded:\n"
            f"{names}\n\n"
            f"frame:\n  {planning_frame}"
        )
        return loaded

    def _static_box_blocks(self) -> list[tuple[str, dict[str, Any]]]:
        """Return (yaml_key, block) for each static box to load."""
        keys = list(
            self.get_parameter("static_keys")
            .get_parameter_value()
            .string_array_value
        )
        if not keys:
            keys = list(DEFAULT_STATIC_KEYS)
        include_object = (
            self.get_parameter("include_object")
            .get_parameter_value()
            .bool_value
        )
        if include_object and "object" not in keys:
            keys.append("object")

        blocks: list[tuple[str, dict[str, Any]]] = []
        missing: list[str] = []
        for key in keys:
            block = self._config.get(key)
            if not isinstance(block, dict):
                missing.append(key)
                continue
            blocks.append((key, block))
        if missing:
            raise MoveItError(
                "工作站 YAML 缺少静态物体定义："
                f"{missing}（配置：{self._config_file}）"
            )
        return blocks

    def _to_planning_frame(
        self,
        pose: Pose,
        source_frame: str,
        planning_frame: str,
        t_world_base: Pose,
    ) -> Pose:
        """Transform a Pose into the MoveIt planning frame via TF."""
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
                    "使用 YAML robot.base_pose"
                )
                return _fallback_world_base(
                    pose, source_frame, planning_frame, t_world_base
                )
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
        world_frame: str,
        timeout_sec: float = 30.0,
    ) -> None:
        """Wait until world -> planning_frame is available (or skip if same)."""
        if world_frame == planning_frame:
            return
        deadline = time.monotonic() + timeout_sec
        last_error = "timeout"
        while time.monotonic() < deadline and rclpy.ok():
            try:
                self._tf.lookup_transform(
                    planning_frame, world_frame, rclpy.time.Time()
                )
                return
            except TransformException as exc:
                last_error = str(exc)
                rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().warn(
            f"等待 TF {world_frame} -> {planning_frame} 超时（{last_error}），"
            "将在转换时回退到 YAML robot.base_pose"
        )

    def _wait_for_clock(self, timeout_sec: float = 20.0) -> None:
        """Wait until /clock is valid when use_sim_time is set."""
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
            "未收到 /clock。仿真请保持 use_sim_time:=true 并先启动 Gazebo"
        )


def _box_from_yaml(
    block: dict[str, Any],
) -> tuple[str, Pose, list[float], str, list[str]]:
    """Parse one YAML box obstacle: name, pose, size, source frame, ACM."""
    name = str(block["name"])
    pose_block = block["initial_pose"]
    pose = block_pose(pose_block)
    size = list(as_vec3(block["dimensions"]))
    frame = str(pose_block.get("frame", "world"))
    touch_links = [str(item) for item in block.get("touch_links", [])]
    return name, pose, size, frame, touch_links


def _add_static_box(
    scene: PlanningSceneClient,
    *,
    object_id: str,
    pose: Pose,
    size: Sequence[float],
    frame_id: str,
    touch_links: Sequence[str],
) -> None:
    """Replace one box collision object and apply optional allowed contacts."""
    scene.remove(object_id)
    scene.add_box(object_id, pose, size, frame_id)
    if touch_links:
        scene.allow_collisions(object_id, touch_links, allowed=True)


def _fallback_world_base(
    pose: Pose,
    source_frame: str,
    planning_frame: str,
    t_world_base: Pose,
) -> Pose:
    """Convert between world and base_link using YAML T_world_base."""
    if source_frame == "world" and planning_frame == "base_link":
        return pose_multiply(pose_inverse(t_world_base), pose)
    if source_frame == "base_link" and planning_frame == "world":
        return pose_multiply(t_world_base, pose)
    raise MoveItError(
        f"无法在 {source_frame} 与 {planning_frame} 之间转换位姿"
    )


def main(args=None) -> None:
    """ROS 2 entry point; keeps spinning after the static scene is applied."""
    rclpy.init(args=args)
    node = WorkcellSceneLoader()
    try:
        node.load_scene()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("已中断")
    except (MoveItError, TransformException, KeyError, TypeError) as exc:
        node.get_logger().error(f"工作站 Planning Scene 加载失败：{exc}")
        raise SystemExit(1) from exc
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
