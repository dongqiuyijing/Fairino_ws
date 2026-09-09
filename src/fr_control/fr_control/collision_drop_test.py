"""
Independent Gazebo collision test: drop a box onto a stationary finger.

Does not command the gripper. Gazebo must already be running.
"""

from __future__ import annotations

import os
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from fr_control.gripper import GripperError


_DROP_SDF = """<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{name}">
    <static>false</static>
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name="link">
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>2.0e-5</ixx>
          <iyy>2.0e-5</iyy>
          <izz>2.0e-5</izz>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.020 0.020 0.020</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.020 0.020 0.020</size>
          </box>
        </geometry>
        <material>
          <ambient>0.05 0.45 0.90 1</ambient>
          <diffuse>0.10 0.70 1.00 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


class CollisionDropTestNode(Node):
    """Drop a 20 mm cube onto finger_l and check that it does not fall through."""

    def __init__(self) -> None:
        """Prepare TF and the Ignition partition used by sim.launch.py."""
        super().__init__("fr_collision_drop_test")
        self.declare_parameter("finger_link", "finger_l")
        self.declare_parameter("drop_height", 0.06)
        self.declare_parameter("hold_seconds", 0.8)
        self.declare_parameter("pass_z_margin", 0.008)
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        os.environ.setdefault("IGN_PARTITION", "fairino3_gazebo")
        os.environ.setdefault("GZ_PARTITION", "fairino3_gazebo")

    def run(self) -> None:
        """Spawn a cube above a stationary finger and record its pose."""
        self._wait_for_clock()
        finger = (
            self.get_parameter("finger_link")
            .get_parameter_value()
            .string_value
        )
        drop_height = float(
            self.get_parameter("drop_height").get_parameter_value().double_value
        )
        hold = float(
            self.get_parameter("hold_seconds").get_parameter_value().double_value
        )
        margin = float(
            self.get_parameter("pass_z_margin").get_parameter_value().double_value
        )
        pose = self._lookup_pose("world", finger)
        transform = self._lookup_transform("world", finger)
        corners = []
        for x in (-0.012, 0.0165):
            for y in (-0.020, 0.020):
                for z in (0.0, 0.0717):
                    corners.append(_transform_point(transform, x, y, z))
        max_z = max(corner[2] for corner in corners)
        center_x = 0.5 * (min(c[0] for c in corners) + max(c[0] for c in corners))
        center_y = 0.5 * (min(c[1] for c in corners) + max(c[1] for c in corners))
        spawn_z = max_z + drop_height
        name = "drop_block"
        self.get_logger().info(
            f"在静止 {finger} 碰撞盒上方生成 {name}："
            f"x={center_x:.4f} y={center_y:.4f} z={spawn_z:.4f} "
            f"(finger origin z={pose.pose.position.z:.4f} aabb_max_z={max_z:.4f})"
        )
        self._spawn_box(name, center_x, center_y, spawn_z)
        self._wait_sim(hold)
        final = self._model_pose(name)
        if final is None:
            raise GripperError(f"无法读取 {name} 位姿")
        rest_z = max_z + 0.008
        self.get_logger().info(
            f"{name} 最终 z={final[2]:.4f} "
            f"(手指碰撞最高点 z={max_z:.4f}，期望不低于 {rest_z:.4f})"
        )
        if final[2] < rest_z - margin:
            raise GripperError(
                f"方块穿过手指：final_z={final[2]:.4f} "
                f"finger_aabb_max_z={max_z:.4f}。"
                "请先修复 collision，不要继续调 controller。"
            )
        self.get_logger().info(
            "静止 collision 测试通过：方块被手指挡住，没有掉到手指下方"
        )

    def _spawn_box(self, name: str, x: float, y: float, z: float) -> None:
        """Create a dynamic cube in the running Ignition world."""
        sdf = _DROP_SDF.format(name=name, x=x, y=y, z=z)
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".sdf", delete=False
        ) as handle:
            handle.write(sdf)
            path = handle.name
        command = [
            "ign",
            "service",
            "-s",
            "/world/default/create",
            "--reqtype",
            "ignition.msgs.EntityFactory",
            "--reptype",
            "ignition.msgs.Boolean",
            "--timeout",
            "3000",
            "--req",
            f'sdf_filename: "{path}" name: "{name}" allow_renaming: true',
        ]
        result = subprocess.run(command, capture_output=True, text=True)
        if result.returncode != 0:
            raise GripperError(
                f"生成 {name} 失败：{result.stderr or result.stdout}"
            )
        self.get_logger().info(result.stdout.strip() or f"已请求生成 {name}")

    def _model_pose(self, name: str) -> tuple[float, float, float] | None:
        """Read a model pose from /world/default/pose/info."""
        command = [
            "ign",
            "topic",
            "-t",
            "/world/default/pose/info",
            "-e",
            "-n",
            "1",
        ]
        result = subprocess.run(
            command, capture_output=True, text=True, timeout=8
        )
        if result.returncode != 0:
            self.get_logger().warn(result.stderr.strip())
            return None
        return _parse_named_pose(result.stdout, name)

    def _lookup_transform(self, frame_id: str, link: str):
        """Wait until TF from frame_id to link is available."""
        deadline = time.monotonic() + 20.0
        last_error = "timeout"
        while time.monotonic() < deadline and rclpy.ok():
            try:
                return self._tf.lookup_transform(
                    frame_id, link, rclpy.time.Time()
                )
            except TransformException as exc:
                last_error = str(exc)
                rclpy.spin_once(self, timeout_sec=0.1)
        raise GripperError(f"等待 TF {frame_id} -> {link} 失败：{last_error}")

    def _lookup_pose(self, frame_id: str, link: str) -> PoseStamped:
        """Wait until TF from world to the finger link is available."""
        transform = self._lookup_transform(frame_id, link)
        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

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

    def _wait_sim(self, seconds: float) -> None:
        """Wait until the Gazebo clock advances by the given seconds."""
        start = self.get_clock().now()
        wall_deadline = time.monotonic() + max(60.0, float(seconds) * 120.0)
        while rclpy.ok() and time.monotonic() < wall_deadline:
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed >= seconds:
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        raise GripperError(f"等待仿真 {seconds:.2f}s 超时")


def _parse_named_pose(
    text: str, name: str
) -> tuple[float, float, float] | None:
    """Extract x y z of a named pose from ign topic text output."""
    lines = text.splitlines()
    for index, line in enumerate(lines):
        if f'name: "{name}"' in line or f"name: '{name}'" in line:
            chunk = "\n".join(lines[index : index + 40])
            values = {}
            for axis in ("x", "y", "z"):
                for item in chunk.splitlines():
                    stripped = item.strip()
                    if stripped.startswith(f"{axis}:"):
                        try:
                            values[axis] = float(stripped.split(":", 1)[1])
                        except ValueError:
                            continue
                        break
                if axis in values and len(values) == 3:
                    break
            if {"x", "y", "z"} <= values.keys():
                return values["x"], values["y"], values["z"]
    return None


def _transform_point(transform, x: float, y: float, z: float) -> tuple[float, float, float]:
    """Rotate and translate a point from a link frame into the parent frame."""
    q = transform.transform.rotation
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    ix = qw * x + qy * z - qz * y
    iy = qw * y + qz * x - qx * z
    iz = qw * z + qx * y - qy * x
    iw = -qx * x - qy * y - qz * z
    rx = ix * qw + iw * -qx + iy * -qz - iz * -qy
    ry = iy * qw + iw * -qy + iz * -qx - ix * -qz
    rz = iz * qw + iw * -qz + ix * -qy - iy * -qx
    t = transform.transform.translation
    return t.x + rx, t.y + ry, t.z + rz


def main(args=None) -> None:
    """ROS 2 entry point; exits after the drop test finishes."""
    rclpy.init(args=args)
    node = CollisionDropTestNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("已中断")
    except Exception as exc:
        node.get_logger().error(f"collision 测试失败：{exc}")
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
