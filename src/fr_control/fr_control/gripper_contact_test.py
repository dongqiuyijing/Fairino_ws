"""
Gripper contact test with the arm held still.

Open -> place a 20x40x20 mm block between the fingers -> close -> hold.
Records commanded vs actual joint state. Does not move the FR3 arm.
"""

from __future__ import annotations

import math
import os
import subprocess
import tempfile
import time

from geometry_msgs.msg import Pose, Quaternion
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

from fr_control.constants import (
    GRIPPER_CLOSED,
    GRIPPER_LEFT_JOINT,
    GRIPPER_RIGHT_JOINT,
)
from fr_control.gripper import GripperError, create_gripper, split_travel


_SUPPORT_SDF = """<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{name}">
    <static>true</static>
    <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>0.050 0.050 0.010</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.050 0.050 0.010</size>
          </box>
        </geometry>
        <material>
          <ambient>0.25 0.25 0.25 1</ambient>
          <diffuse>0.40 0.40 0.40 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

_CONTACT_SDF = """<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{name}">
    <static>false</static>
    <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
    <link name="link">
      <inertial>
        <mass>0.03</mass>
        <inertia>
          <ixx>5.0e-6</ixx>
          <iyy>2.0e-6</iyy>
          <izz>5.0e-6</izz>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.020 0.040 0.020</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.5</mu>
              <mu2>1.5</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.020 0.040 0.020</size>
          </box>
        </geometry>
        <material>
          <ambient>0.80 0.55 0.05 1</ambient>
          <diffuse>1.00 0.75 0.10 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


class GripperContactTestNode(Node):
    """Close the gripper on a block between the fingers and log stall evidence."""

    def __init__(self) -> None:
        """Declare backends, TF, and joint-state recording."""
        super().__init__("fr_gripper_contact_test")
        self.declare_parameter("backend", "gazebo")
        self.declare_parameter("hold_seconds", 0.8)
        self.declare_parameter("model_name", "contact_block")
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._joint_state: JointState | None = None
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )
        os.environ.setdefault("IGN_PARTITION", "fairino3_gazebo")
        os.environ.setdefault("GZ_PARTITION", "fairino3_gazebo")

    def run(self) -> None:
        """Open, place a block, close, and compare command vs actual."""
        self._wait_for_clock()
        backend = (
            self.get_parameter("backend").get_parameter_value().string_value
        )
        hold = float(
            self.get_parameter("hold_seconds").get_parameter_value().double_value
        )
        model = (
            self.get_parameter("model_name").get_parameter_value().string_value
        )
        model = f"{model}_{int(time.time())}"
        gripper = create_gripper(self, backend=backend)

        self.get_logger().info("1. OPEN")
        gripper.open(timeout_sec=8.0)
        self._spin(0.5)
        self._log_joints("after open")

        pose, rpy = self._grasp_pose()
        support_z = pose.position.z - 0.016
        self.get_logger().info(
            f"2. 在两指之间放置 {model}："
            f"x={pose.position.x:.4f} y={pose.position.y:.4f} "
            f"z={pose.position.z:.4f} rpy={rpy}"
        )
        self._spawn_sdf(
            "contact_support",
            _SUPPORT_SDF.format(
                name="contact_support",
                x=pose.position.x,
                y=pose.position.y,
                z=support_z,
                roll=rpy[0],
                pitch=rpy[1],
                yaw=rpy[2],
            ),
        )
        self._spawn_sdf(
            model,
            _CONTACT_SDF.format(
                name=model,
                x=pose.position.x,
                y=pose.position.y,
                z=pose.position.z,
                roll=rpy[0],
                pitch=rpy[1],
                yaw=rpy[2],
            ),
        )
        self._wait_sim(0.08)
        before = self._model_pose(model)

        cmd_left, cmd_right = split_travel(GRIPPER_CLOSED)
        self.get_logger().info(
            f"3. CLOSE  commanded left={cmd_left:.4f} right={cmd_right:.4f}"
        )
        gripper.close(timeout_sec=12.0)
        self._spin(0.5)
        after_close = self._log_joints("after close")

        self.get_logger().info(f"4. HOLD {hold:.1f}s")
        if hold > 0.0:
            self._wait_sim(hold)
        after_hold = self._log_joints("after hold")
        after = self._model_pose(model)

        left = after_hold.get(GRIPPER_LEFT_JOINT, {})
        right = after_hold.get(GRIPPER_RIGHT_JOINT, {})
        left_pos = float(left.get("position", 0.0))
        right_pos = float(right.get("position", 0.0))
        left_vel = float(left.get("velocity", 0.0))
        right_vel = float(right.get("velocity", 0.0))
        left_err = abs(left_pos - cmd_left)
        right_err = abs(right_pos - cmd_right)
        self.get_logger().info(
            "接触证据："
            f" left cmd={cmd_left:.4f} actual={left_pos:.4f} "
            f"err={left_err:.4f} vel={left_vel:.4f}"
            f" right cmd={cmd_right:.4f} actual={right_pos:.4f} "
            f"err={right_err:.4f} vel={right_vel:.4f}"
        )
        if before is not None:
            self.get_logger().info(
                f"{model} before=({before[0]:.4f}, {before[1]:.4f}, {before[2]:.4f})"
            )
        if after is not None:
            self.get_logger().info(
                f"{model} after=({after[0]:.4f}, {after[1]:.4f}, {after[2]:.4f})"
            )

        stalled = left_err > 0.004 and right_err > 0.004
        slow = abs(left_vel) < 0.01 and abs(right_vel) < 0.01
        if not stalled:
            raise GripperError(
                "夹爪到达了完全闭合目标，零件没有挡住手指。"
                f" left {left_pos:.4f}/{cmd_left:.4f}"
                f" right {right_pos:.4f}/{cmd_right:.4f}"
            )
        if not slow:
            self.get_logger().warn("接触后关节速度仍较大，可能还在振荡")
        if after is None:
            raise GripperError(f"无法读取 {model} 位姿")
        self.get_logger().info(
            "接触测试通过：commanded 与 actual 因零件阻挡而不同，关节已 stall"
        )
        _ = after_close

    def _grasp_pose(self) -> tuple[Pose, tuple[float, float, float]]:
        """Place the block at the midpoint of the two finger pads."""
        left = self._lookup("world", "finger_l")
        right = self._lookup("world", "finger_r")
        left_pad = _transform_point(left, 0.0165, 0.0, 0.03585)
        right_pad = _transform_point(right, 0.0165, 0.0, 0.03585)
        pose = Pose()
        pose.position.x = 0.5 * (left_pad[0] + right_pad[0])
        pose.position.y = 0.5 * (left_pad[1] + right_pad[1])
        pose.position.z = 0.5 * (left_pad[2] + right_pad[2])
        pose.orientation = left.transform.rotation
        rpy = _quat_to_rpy(pose.orientation)
        return pose, rpy

    def _spawn_sdf(self, name: str, sdf: str) -> None:
        """Create a model in the running Ignition world from an SDF string."""
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
        from fr_control.collision_drop_test import _parse_named_pose

        return _parse_named_pose(result.stdout, name)

    def _lookup(self, frame_id: str, link: str):
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

    def _log_joints(self, label: str) -> dict[str, dict[str, float]]:
        """Log and return the latest left/right slider state."""
        self._spin(0.2)
        snapshot: dict[str, dict[str, float]] = {}
        if self._joint_state is None:
            self.get_logger().warn(f"{label}: 还没有 /joint_states")
            return snapshot
        names = list(self._joint_state.name)
        positions = list(self._joint_state.position)
        velocities = list(self._joint_state.velocity or [])
        efforts = list(self._joint_state.effort or [])
        for joint in (GRIPPER_LEFT_JOINT, GRIPPER_RIGHT_JOINT):
            if joint not in names:
                continue
            index = names.index(joint)
            item = {"position": float(positions[index])}
            if index < len(velocities):
                item["velocity"] = float(velocities[index])
            if index < len(efforts):
                item["effort"] = float(efforts[index])
            snapshot[joint] = item
            extra = " ".join(f"{key}={value:.4f}" for key, value in item.items())
            self.get_logger().info(f"{label} {joint}: {extra}")
        return snapshot

    def _on_joint_state(self, msg: JointState) -> None:
        """Cache the latest joint state."""
        self._joint_state = msg

    def _spin(self, seconds: float) -> None:
        """Process callbacks for a short duration."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

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


def _transform_point(transform, x: float, y: float, z: float) -> tuple[float, float, float]:
    """Rotate and translate a point from a link frame into the parent frame."""
    rx, ry, rz = _rotate(transform.transform.rotation, x, y, z)
    t = transform.transform.translation
    return t.x + rx, t.y + ry, t.z + rz


def _rotate(q: Quaternion, x: float, y: float, z: float) -> tuple[float, float, float]:
    """Rotate a vector by a geometry_msgs quaternion."""
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    ix = qw * x + qy * z - qz * y
    iy = qw * y + qz * x - qx * z
    iz = qw * z + qx * y - qy * x
    iw = -qx * x - qy * y - qz * z
    return (
        ix * qw + iw * -qx + iy * -qz - iz * -qy,
        iy * qw + iw * -qy + iz * -qx - ix * -qz,
        iz * qw + iw * -qz + ix * -qy - iy * -qx,
    )


def _quat_to_rpy(q: Quaternion) -> tuple[float, float, float]:
    """Convert a geometry_msgs quaternion to roll, pitch, yaw."""
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    pitch = math.asin(sinp)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def main(args=None) -> None:
    """ROS 2 entry point; exits after the contact test finishes."""
    rclpy.init(args=args)
    node = GripperContactTestNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("已中断")
    except Exception as exc:
        node.get_logger().error(f"接触测试失败：{exc}")
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
