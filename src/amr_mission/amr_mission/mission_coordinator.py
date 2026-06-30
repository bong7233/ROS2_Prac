"""ROS-free mission state machine for battery-aware autonomy.

Coordinates the waypoint follower and the docking controller based on battery
level and dock status: patrol until the battery is low, return to the dock and
charge, then resume patrolling when charged. Keeping the decision logic ROS-free
makes it unit-testable without a running graph.

States
------
* ``PATROL``         - waypoint follower enabled, docking disabled.
* ``RETURN_TO_DOCK`` - docking enabled, waypoint follower disabled.
* ``CHARGING``       - both disabled while docked and charging.
"""
from __future__ import annotations

from dataclasses import dataclass

PATROL = "PATROL"
RETURN_TO_DOCK = "RETURN_TO_DOCK"
CHARGING = "CHARGING"


@dataclass
class MissionConfig:
    """Battery thresholds (fraction 0..1) that drive the mission."""

    low_battery_pct: float = 0.30
    full_battery_pct: float = 0.90


@dataclass
class MissionDecision:
    """The next mission state and the enable flags it implies."""

    state: str
    enable_navigation: bool
    enable_docking: bool


def decide(
    current_state: str,
    battery_pct: float,
    docked: bool,
    config: MissionConfig = MissionConfig(),
) -> MissionDecision:
    """Compute the next mission state and the controllers it enables."""
    if current_state == RETURN_TO_DOCK:
        if docked:
            return MissionDecision(CHARGING, enable_navigation=False, enable_docking=False)
        return MissionDecision(RETURN_TO_DOCK, enable_navigation=False, enable_docking=True)

    if current_state == CHARGING:
        if battery_pct >= config.full_battery_pct:
            return MissionDecision(PATROL, enable_navigation=True, enable_docking=False)
        return MissionDecision(CHARGING, enable_navigation=False, enable_docking=False)

    # PATROL (and any unknown state defaults to patrolling).
    if battery_pct <= config.low_battery_pct:
        return MissionDecision(
            RETURN_TO_DOCK, enable_navigation=False, enable_docking=True
        )
    return MissionDecision(PATROL, enable_navigation=True, enable_docking=False)
