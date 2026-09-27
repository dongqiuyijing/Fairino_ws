"""Existing Fairino gripper service adapter; no SDK connection is created."""

from __future__ import annotations

from typing import Callable

from fairino_msgs.srv import GripperBridge


class GripperBridgeClient:
    """Submit open/close through the hardware plugin's existing ROS service."""

    def __init__(self, node, service_name: str, open_position: int, close_position: int, on_result: Callable[[str], None]) -> None:
        self._client = node.create_client(GripperBridge, service_name)
        self._open_position = open_position
        self._close_position = close_position
        self._on_result = on_result

    def send(self, command: str) -> bool:
        if command not in {"open", "close"}:
            raise ValueError("invalid gripper command")
        if not self._client.service_is_ready():
            self._on_result("gripper service unavailable")
            return False
        request = GripperBridge.Request()
        request.command = "move"
        request.gripper_id = 1
        request.position = self._open_position if command == "open" else self._close_position
        request.velocity = 20
        request.force = 20
        request.max_time_ms = 5000
        request.block = 1
        request.gripper_type = 0
        request.rot_num = 0.0
        request.rot_vel = 0
        request.rot_torque = 0
        future = self._client.call_async(request)
        future.add_done_callback(self._done)
        return True

    def _done(self, future) -> None:
        try:
            response = future.result()
            if response is None or response.error_code != 0:
                self._on_result(f"gripper failed: {getattr(response, 'message', 'no response')}")
            else:
                self._on_result("gripper command completed")
        except Exception as exc:  # ROS transport exceptions are surfaced to GUI status.
            self._on_result(f"gripper service exception: {exc}")
