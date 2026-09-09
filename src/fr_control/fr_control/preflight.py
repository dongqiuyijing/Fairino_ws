"""
Detect leftover Gazebo / MoveIt sessions before sending trajectories.

Duplicate move_group or robot_state_publisher instances mix /clock and TF,
which shows up as TF_OLD_DATA and Execute CONTROL_FAILED.
"""

from __future__ import annotations

from collections import Counter

import rclpy
from rclpy.node import Node

from fr_control.moveit_arm import MoveItError


_CLEANUP = (
    "请只保留一套仿真：关掉多余终端中的 sim.launch.py 和 "
    "move_group.launch.py，然后重新启动。"
)


def check_unique_sim_graph(node: Node) -> None:
    """Raise MoveItError if more than one sim/MoveIt session is visible."""
    # Graph discovery is not instantaneous.
    for _ in range(8):
        rclpy.spin_once(node, timeout_sec=0.05)
    names = node.get_node_names()
    counts = Counter(names)
    problems: list[str] = []
    for name in (
        "move_group",
        "robot_state_publisher",
        "ros_gz_bridge",
        "controller_manager",
    ):
        n = counts.get(name, 0)
        if n > 1:
            problems.append(f"{name}×{n}")

    move_servers = _publisher_count(node, "/move_action/_action/status")
    exec_servers = _publisher_count(
        node, "/execute_trajectory/_action/status"
    )
    clocks = _publisher_count(node, "/clock")
    if move_servers > 1:
        problems.append(f"move_action 服务端×{move_servers}")
    if exec_servers > 1:
        problems.append(f"execute_trajectory 服务端×{exec_servers}")
    if clocks > 1:
        problems.append(f"/clock 发布者×{clocks}")

    if not problems:
        return
    raise MoveItError(
        "检测到多套仿真/MoveIt 同时在跑（"
        + "，".join(problems)
        + "）。这会导致 TF_OLD_DATA 和 Execute CONTROL_FAILED。"
        + _CLEANUP
    )


def check_tcp_height(
    z: float,
    *,
    min_z: float = 0.08,
) -> None:
    """Raise if the TCP is implausibly low for a standing FR3."""
    if z >= min_z:
        return
    raise MoveItError(
        f"当前 gripper_tcp 高度 z={z:.3f} m，机械臂像是倒下或 TF 已损坏。"
        "请重启 Gazebo（只启动一次 sim.launch.py），不要在旧仿真上继续。"
    )


def _publisher_count(node: Node, topic: str) -> int:
    """Count publishers on a topic; 0 if the graph has not advertised it."""
    try:
        return len(node.get_publishers_info_by_topic(topic))
    except Exception:
        return 0
