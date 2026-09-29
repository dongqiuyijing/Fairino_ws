"""Arm A Stage 1 manual Cartesian ownership helpers.

The hardware plugin is the only ServoCart caller. This module decides which
controller switch to request and which single base-X velocity is legal.
"""

from __future__ import annotations

from typing import Any

STAGE1_ARM = "arm_a"
_GRIPPER_SUFFIX = "/fairino_gripper/command"


def manual_cartesian_config(config: Any) -> dict[str, Any]:
    if not isinstance(config, dict):
        return {}
    teleop = config.get("teleop", {})
    if not isinstance(teleop, dict):
        return {}
    block = teleop.get("manual_cartesian", {})
    return block if isinstance(block, dict) else {}


def stage1_configured(config: Any) -> bool:
    return bool(manual_cartesian_config(config).get("enabled", False))


def stage1_velocity_mm_s(config: Any) -> float:
    teleop = config.get("teleop", {}) if isinstance(config, dict) else {}
    if not isinstance(teleop, dict):
        return 10.0
    return float(teleop.get("default_linear_speed_mm_s", 10.0))


def manual_namespace(gripper_service: str) -> str:
    if gripper_service.endswith(_GRIPPER_SUFFIX):
        return gripper_service[: -len(_GRIPPER_SUFFIX)] + "/fairino_manual"
    return "/arm_a/fairino_manual"


def is_stage1_motion(arm: str, frame: str, axis: str, sign: int) -> bool:
    return arm == STAGE1_ARM and frame == "base" and axis == "x" and int(sign) == 1


def switch_lists(
    arm: str,
    entering_manual: bool,
    cartesian_arm_a: bool,
    auto_controller: str,
    teleop_controller: str,
) -> tuple[list[str], list[str]]:
    """Return (activate, deactivate). Arm A Stage 1 never activates the teleop controller."""
    if cartesian_arm_a and arm == STAGE1_ARM:
        if entering_manual:
            return [], [auto_controller]
        return [auto_controller], []
    if entering_manual:
        return [teleop_controller], [auto_controller]
    return [auto_controller], [teleop_controller]


def controller_pair_state(
    arm: str,
    auto_state: str | None,
    teleop_state: str | None,
    cartesian_arm_a: bool,
) -> str:
    if cartesian_arm_a and arm == STAGE1_ARM:
        if auto_state == "active" and teleop_state == "inactive":
            return "AUTO"
        if auto_state == "inactive" and teleop_state == "inactive":
            return "MANUAL"
        return "FAULT"
    if auto_state == "active" and teleop_state == "inactive":
        return "AUTO"
    if auto_state == "inactive" and teleop_state == "active":
        return "MANUAL"
    return "FAULT"
