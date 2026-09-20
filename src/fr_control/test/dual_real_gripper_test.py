#!/usr/bin/env python3
"""Read-only ping or guarded small-motion test for the two real FR3 grippers.

Requires the existing dual_bringup.launch.py to be running. Does NOT start an
SDK/RPC connection, move either arm, use the virtual gripper_joint, or call
ActGripper(reset/activate). The real grippers are separate GripperBridge
services owned by each real hardware plugin.

Default: ping only. With --run: A then B (or --arm a/b), each with its own
explicit operator confirmation: open(0) -> partial-close(20) -> open(0).
The test stops on the first error; it never silently continues to the next
gripper or retries an ambiguous/timeout motion.
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from fairino_msgs.srv import GripperBridge
from rclpy.node import Node

SERVICES = {
    "a": "/arm_a/fairino_gripper/command",
    "b": "/arm_b/fairino_gripper/command",
}
GRIPPER_ID = 1
OPEN_POSITION = 0
PARTIAL_CLOSE_POSITION = 20
VELOCITY = 10
FORCE = 15
MAX_TIME_MS = 5000
RESPONSE_TIMEOUT_SEC = 20.0


class TestError(RuntimeError):
    pass


def call(node: Node, client, service: str, command: str, *, position: int = 0):
    request = GripperBridge.Request()
    request.command = command
    request.gripper_id = GRIPPER_ID
    request.position = position
    request.velocity = VELOCITY
    request.force = FORCE
    request.max_time_ms = MAX_TIME_MS
    request.block = 1
    request.gripper_type = 0
    request.rot_num = 0.0
    request.rot_vel = 0
    request.rot_torque = 0

    future = client.call_async(request)
    rclpy.spin_until_future_complete(
        node, future, timeout_sec=RESPONSE_TIMEOUT_SEC
    )
    if not future.done():
        # The hardware may still be finishing the motion / restoring ServoJ.
        # Do not send another command when completion is unknown.
        raise TestError(f"{service}: {command} timeout; stop and inspect logs")
    result = future.result()
    if result is None:
        raise TestError(f"{service}: {command} returned no response")
    if int(result.error_code) != 0:
        raise TestError(
            f"{service}: {command} error_code={result.error_code}, "
            f"message={result.message}"
        )
    node.get_logger().info(
        f"{service}: {command}"
        + (f" position={position}" if command == "move" else "")
        + f" OK (service response: {result.message})"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--arm", choices=("a", "b", "both"), default="both",
        help="Select real gripper; default: both, tested sequentially",
    )
    parser.add_argument(
        "--run", action="store_true",
        help="Actually command a small open -> partial close -> open cycle; "
        "without --run only ping is sent",
    )
    args = parser.parse_args()
    arms = ("a", "b") if args.arm == "both" else (args.arm,)

    rclpy.init()
    node = Node("dual_real_gripper_test")
    try:
        clients = {}
        for arm in arms:
            name = SERVICES[arm]
            client = node.create_client(GripperBridge, name)
            node.get_logger().info(f"Checking Arm {arm.upper()} {name}")
            if not client.wait_for_service(timeout_sec=8.0):
                raise TestError(f"Arm {arm.upper()}: service unavailable: {name}")
            clients[arm] = client

        # Verify all selected endpoints before allowing any motion.
        for arm in arms:
            call(node, clients[arm], SERVICES[arm], "ping")
        node.get_logger().info("All selected gripper bridges responded to ping.")

        if not args.run:
            node.get_logger().info("CHECK ONLY: no motion / reset / activate sent.")
            return 0

        if not sys.stdin.isatty():
            raise TestError("--run requires an interactive terminal for confirmation")

        for arm in arms:
            name = SERVICES[arm]
            print(
                f"\nArm {arm.upper()}: real gripper will OPEN(0), "
                f"PARTIAL CLOSE({PARTIAL_CLOSE_POSITION}), OPEN(0).\n"
                "Clear the fingers and area; remove any held object. "
                "Confirm this arm is safe, enabled, and gripper is already "
                "activated. No robot arm movement is requested.\n"
                f"Type {arm.upper()} and press Enter to continue "
                "(anything else cancels): ",
                end="", flush=True,
            )
            if input().strip() != arm.upper():
                raise TestError(f"Operator cancelled before Arm {arm.upper()}")
            for position in (
                OPEN_POSITION, PARTIAL_CLOSE_POSITION, OPEN_POSITION
            ):
                call(node, clients[arm], name, "move", position=position)
                time.sleep(1.0)
            node.get_logger().info(
                f"Arm {arm.upper()} service commands passed. "
                "Visually verify that the fingers physically moved."
            )

        node.get_logger().info(
            "TEST COMPLETE. Service success is not independent physical "
            "finger-position feedback."
        )
        return 0
    except (TestError, KeyboardInterrupt, EOFError) as exc:
        node.get_logger().error(f"STOP: {exc}")
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
