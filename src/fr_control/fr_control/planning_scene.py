"""
MoveIt PlanningScene helpers for the known-object grasp test.

Adds the table and part as collision objects, allows gripper-part contact,
then attaches the part to the TCP after the gripper closes.
"""

from __future__ import annotations

from typing import Sequence
import time

from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
import rclpy

from fr_control.constants import (
    APPLY_SCENE_SERVICE,
    ATTACH_LINK,
    GET_SCENE_SERVICE,
    GRIPPER_TOUCH_LINKS,
)
from fr_control.moveit_arm import MoveItError


class PlanningSceneClient:
    """Thin client around ApplyPlanningScene / GetPlanningScene."""

    def __init__(self, node: Node) -> None:
        """Create service clients; call wait_until_ready before using."""
        self._node = node
        self._apply = node.create_client(
            ApplyPlanningScene, APPLY_SCENE_SERVICE
        )
        self._get = node.create_client(GetPlanningScene, GET_SCENE_SERVICE)
        self._obj_pub = node.create_publisher(
            CollisionObject, "/collision_object", 10
        )
        self._att_pub = node.create_publisher(
            AttachedCollisionObject, "/attached_collision_object", 10
        )

    def wait_until_ready(self, timeout_sec: float = 30.0) -> None:
        """Wait until move_group planning scene services are available."""
        self._node.get_logger().info("等待 PlanningScene 服务 ...")
        if not self._apply.wait_for_service(timeout_sec=timeout_sec):
            raise MoveItError(f"未找到 {APPLY_SCENE_SERVICE}")
        if not self._get.wait_for_service(timeout_sec=timeout_sec):
            raise MoveItError(f"未找到 {GET_SCENE_SERVICE}")
        self._node.get_logger().info("PlanningScene 服务已就绪")

    def add_box(
        self,
        object_id: str,
        pose: Pose,
        size: Sequence[float],
        frame_id: str,
    ) -> None:
        """Add or replace a box collision object."""
        obj = self._box_object(
            object_id,
            pose,
            size,
            frame_id,
            CollisionObject.ADD,
        )
        self._obj_pub.publish(obj)
        time.sleep(0.3)
        self._node.get_logger().info(
            f"PlanningScene 已添加 {object_id} "
            f"size=({size[0]:.3f}, {size[1]:.3f}, {size[2]:.3f})"
        )

    def allow_collisions(
        self,
        name: str,
        other_names: Sequence[str],
        allowed: bool = True,
    ) -> None:
        """Set ACM entries between name and each name in other_names."""
        acm = self._get_acm()
        for other in other_names:
            _set_acm_entry(acm, name, other, allowed)
        scene = PlanningScene()
        scene.is_diff = True
        scene.allowed_collision_matrix = acm
        self._apply_scene(scene)
        state = "允许" if allowed else "禁止"
        self._node.get_logger().info(
            f"PlanningScene {state} {name} 与 {list(other_names)} 碰撞"
        )

    def attach_box(
        self,
        object_id: str,
        pose_in_link: Pose,
        size: Sequence[float],
        *,
        link_name: str = ATTACH_LINK,
        touch_links: Sequence[str] = GRIPPER_TOUCH_LINKS,
    ) -> None:
        """Attach a box to a robot link and remove it from the world."""
        attached = AttachedCollisionObject()
        attached.link_name = link_name
        attached.object = self._box_object(
            object_id,
            pose_in_link,
            size,
            link_name,
            CollisionObject.ADD,
        )
        attached.touch_links = list(touch_links)
        attached.weight = 0.0
        self._att_pub.publish(attached)
        time.sleep(0.3)
        self._node.get_logger().info(
            f"PlanningScene 已将 {object_id} 附着到 {link_name}"
        )

    def detach(self, object_id: str, link_name: str = ATTACH_LINK) -> None:
        """Detach an object from the robot if it is currently attached."""
        attached = AttachedCollisionObject()
        attached.link_name = link_name
        attached.object.id = object_id
        attached.object.operation = CollisionObject.REMOVE
        self._att_pub.publish(attached)
        time.sleep(0.2)

    def remove(self, object_id: str) -> None:
        """Remove a world collision object if it exists."""
        obj = CollisionObject()
        obj.id = object_id
        obj.operation = CollisionObject.REMOVE
        self._obj_pub.publish(obj)
        time.sleep(0.2)

    def _box_object(
        self,
        object_id: str,
        pose: Pose,
        size: Sequence[float],
        frame_id: str,
        operation: int,
    ) -> CollisionObject:
        """Build a BOX CollisionObject."""
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [
            float(size[0]),
            float(size[1]),
            float(size[2]),
        ]
        header = Header()
        header.frame_id = frame_id
        header.stamp = self._node.get_clock().now().to_msg()
        obj = CollisionObject()
        obj.header = header
        obj.id = object_id
        # ROS 2 Python Pose() 默认四元数为 (0,0,0,0)。MoveIt 会丢弃该物体。
        obj.pose.orientation.w = 1.0
        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        obj.operation = operation
        return obj

    def _get_acm(self) -> AllowedCollisionMatrix:
        """Fetch the current allowed collision matrix."""
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        future = self._get.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            raise MoveItError("获取 PlanningScene ACM 失败")
        return future.result().scene.allowed_collision_matrix

    def _apply_scene(self, scene: PlanningScene) -> None:
        """Send a planning scene diff and require success."""
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self._apply.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            raise MoveItError("ApplyPlanningScene 无响应")
        if not future.result().success:
            raise MoveItError("ApplyPlanningScene 返回 success=false")


def _ensure_acm_index(acm: AllowedCollisionMatrix, name: str) -> int:
    """Return the matrix index for name, appending a row/column if needed."""
    if name in acm.entry_names:
        return acm.entry_names.index(name)
    n_old = len(acm.entry_names)
    acm.entry_names.append(name)
    for entry in acm.entry_values:
        entry.enabled.append(False)
    new_entry = AllowedCollisionEntry()
    new_entry.enabled = [False] * (n_old + 1)
    acm.entry_values.append(new_entry)
    return n_old


def _set_acm_entry(
    acm: AllowedCollisionMatrix,
    name_a: str,
    name_b: str,
    allowed: bool,
) -> None:
    """Set a symmetric ACM entry."""
    index_a = _ensure_acm_index(acm, name_a)
    index_b = _ensure_acm_index(acm, name_b)
    acm.entry_values[index_a].enabled[index_b] = allowed
    acm.entry_values[index_b].enabled[index_a] = allowed
