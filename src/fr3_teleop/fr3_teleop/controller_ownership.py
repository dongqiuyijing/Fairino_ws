"""Controller-manager ownership protocol for a future Servo motion backend.

This module deliberately treats a successful strict controller switch as the
only grant of hardware command ownership. A GUI boolean is never a grant.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Callable


class Owner(str, Enum):
    NONE = "none"
    AUTO = "auto"
    MANUAL = "manual"


@dataclass(frozen=True)
class ControllerPair:
    auto: str
    manual: str


class ControllerLease:
    """Testable strict-switch state machine; callers supply ROS service calls."""

    def __init__(self, pairs: dict[str, ControllerPair], switch: Callable[[list[str], list[str]], bool], auto_busy: Callable[[str], bool]) -> None:
        self._pairs = pairs
        self._switch = switch
        self._auto_busy = auto_busy
        self.owner: dict[str, Owner] = {arm: Owner.AUTO for arm in pairs}

    def acquire_manual(self, arm: str) -> bool:
        if arm not in self._pairs or self.owner[arm] == Owner.MANUAL:
            return arm in self._pairs
        if self._auto_busy(arm):
            return False
        pair = self._pairs[arm]
        if not self._switch([pair.manual], [pair.auto]):
            return False
        self.owner[arm] = Owner.MANUAL
        return True

    def release_manual(self, arm: str) -> bool:
        if arm not in self._pairs or self.owner[arm] != Owner.MANUAL:
            return False
        pair = self._pairs[arm]
        if not self._switch([pair.auto], [pair.manual]):
            return False
        self.owner[arm] = Owner.AUTO
        return True
