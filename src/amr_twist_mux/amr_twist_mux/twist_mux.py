"""ROS-free priority arbitration for multiple velocity-command sources.

Several nodes now want to drive the robot - manual jog, docking, and the
waypoint follower - and they must not fight over ``/cmd_vel``. This picks the
highest-priority source that has published a recent command, like ``twist_mux``.
Keeping the selection logic ROS-free makes it unit-testable without a graph.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple


@dataclass
class TwistInput:
    """One velocity source's latest command and metadata."""

    name: str
    priority: int
    timeout_s: float
    linear_x: float = 0.0
    angular_z: float = 0.0
    stamp_s: Optional[float] = None  # monotonic time of last message, or None


def select_command(
    inputs: Sequence[TwistInput],
    now_s: float,
) -> Tuple[str, float, float]:
    """Return ``(name, linear_x, angular_z)`` for the winning source.

    A source is eligible if it has published within its ``timeout_s``. Among
    eligible sources the highest ``priority`` wins (ties resolve to the earliest
    in ``inputs`` for determinism). If none are eligible the output is a stop
    command from source ``"none"``.
    """
    eligible: List[TwistInput] = [
        item
        for item in inputs
        if item.stamp_s is not None and (now_s - item.stamp_s) <= item.timeout_s
    ]
    if not eligible:
        return ("none", 0.0, 0.0)

    best = eligible[0]
    for item in eligible[1:]:
        if item.priority > best.priority:
            best = item
    return (best.name, best.linear_x, best.angular_z)
