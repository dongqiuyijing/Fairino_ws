"""
Leftover thin XML-RPC helper. Do not use it while real_bringup is running.

FairinoHardwareInterface already owns FRRobot / XML-RPC 20003 for ServoJ.
A second client calling MoveGripper on 20003 will time out and must not
be used from RealFairinoGripper. Real gripper commands go through
/fairino_gripper/command instead.
"""

from __future__ import annotations

import xmlrpc.client
from typing import Any


class GripperRpcError(RuntimeError):
    """Raised when the thin XML-RPC gripper client cannot talk to the controller."""


class _TimeoutTransport(xmlrpc.client.Transport):
    """xmlrpc transport with a connect/read timeout."""

    def __init__(self, timeout_sec: float) -> None:
        """Store the timeout used for each HTTP connection."""
        super().__init__()
        self._timeout_sec = float(timeout_sec)

    def make_connection(self, host):
        """Create the HTTP connection and apply the timeout."""
        connection = super().make_connection(host)
        connection.timeout = self._timeout_sec
        return connection


class FairinoGripperXmlRpc:
    """Gripper-only XML-RPC session to a FAIRINO controller."""

    def __init__(
        self,
        ip: str,
        port: int = 20003,
        timeout_sec: float = 3.0,
    ) -> None:
        """Remember connection parameters. Does not open a socket yet."""
        self._ip = str(ip)
        self._port = int(port)
        self._timeout_sec = float(timeout_sec)
        self._proxy: xmlrpc.client.ServerProxy | None = None

    @property
    def url(self) -> str:
        """XML-RPC endpoint used by FAIRINO controllers."""
        return f"http://{self._ip}:{self._port}"

    def connect(self) -> None:
        """Create the XML-RPC proxy. The first method call opens HTTP."""
        if self._proxy is not None:
            return
        transport = _TimeoutTransport(self._timeout_sec)
        self._proxy = xmlrpc.client.ServerProxy(
            self.url,
            transport=transport,
            allow_none=False,
        )

    def close(self) -> None:
        """Drop the proxy. Does not send CloseRPC or arm commands."""
        self._proxy = None

    def ping(self) -> Any:
        """Read-only connectivity check. Must not move the gripper."""
        errors: list[str] = []
        for method in ("GetGripperConfig", "GetControllerIP", "GetSDKVersion"):
            try:
                return method, self.call(method)
            except Exception as exc:  # noqa: BLE001
                errors.append(f"{method}: {exc}")
        raise GripperRpcError(
            "夹爪 XML-RPC 探测失败（未发送 Act/Move）："
            + "; ".join(errors)
        )

    def act_gripper(self, gripper_id: int, action: int) -> int:
        """Call ActGripper(index, action). 0=reset, 1=activate."""
        return self._code(self.call("ActGripper", int(gripper_id), int(action)))

    def move_gripper(
        self,
        gripper_id: int,
        pos: int,
        vel: int,
        force: int,
        max_time_ms: int,
        block: int,
        gripper_type: int,
        rot_num: float,
        rot_vel: int,
        rot_torque: int,
    ) -> int:
        """Call MoveGripper with the FAIRINO 10-argument signature."""
        return self._code(
            self.call(
                "MoveGripper",
                int(gripper_id),
                int(pos),
                int(vel),
                int(force),
                int(max_time_ms),
                int(block),
                int(gripper_type),
                float(rot_num),
                int(rot_vel),
                int(rot_torque),
            )
        )

    def get_motion_done(self) -> tuple[int, int | None, int | None]:
        """Call GetGripperMotionDone. Returns (code, fault, status)."""
        result = self.call("GetGripperMotionDone")
        if isinstance(result, (list, tuple)):
            code = int(result[0]) if result else -1
            fault = int(result[1]) if len(result) > 1 else None
            status = int(result[2]) if len(result) > 2 else None
            return code, fault, status
        return int(result), None, None

    def call(self, method: str, *args: Any) -> Any:
        """Invoke one XML-RPC method by name."""
        self.connect()
        if self._proxy is None:
            raise GripperRpcError("XML-RPC proxy 未创建")
        try:
            func = getattr(self._proxy, method)
            return func(*args)
        except xmlrpc.client.Fault as exc:
            raise GripperRpcError(
                f"XML-RPC Fault {method}: {exc.faultCode} {exc.faultString}"
            ) from exc
        except Exception as exc:
            raise GripperRpcError(f"XML-RPC 调用失败 {method}: {exc}") from exc

    @staticmethod
    def _code(result: Any) -> int:
        """Unwrap FAIRINO XML-RPC return values to an integer error code."""
        if result is None:
            raise GripperRpcError("XML-RPC 返回空")
        if isinstance(result, (list, tuple)):
            if not result:
                raise GripperRpcError("XML-RPC 返回空列表")
            return int(result[0])
        return int(result)
