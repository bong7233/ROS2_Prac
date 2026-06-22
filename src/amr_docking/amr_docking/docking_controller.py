"""ROS-free docking control law for ``amr_docking``.

Turns a docking error (range / lateral offset / bearing, as produced by
``amr_vision``) into a base velocity command. Keeping the law free of ROS
imports makes the behaviour unit-testable without a running graph, mirroring the
split already used by ``amr_vision``.

Behaviour
---------
* ``DOCKED``  - the detector reports aligned: stop and hold.
* ``SEARCH``  - no marker: stop (or rotate slowly if ``search_yaw_rate`` > 0).
* ``ALIGN``   - marker seen but bearing too large: turn in place toward it first.
* ``APPROACH``- marker roughly ahead: drive forward, slowing as range shrinks,
  while still correcting bearing.

All outputs respect ``max_linear_mps`` / ``max_angular_radps`` so the command is
safe to feed into the same ``/cmd_vel`` path as manual jog and Nav2 (it still
passes through the safety monitor).
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class DockingControllerConfig:
    """Tunable gains and limits for the docking control law."""

    max_linear_mps: float = 0.15
    max_angular_radps: float = 0.6
    range_gain: float = 0.3
    bearing_gain: float = 1.2
    align_first_bearing_rad: float = 0.20
    search_yaw_rate_radps: float = 0.0


@dataclass
class DockingCommand:
    """A velocity command plus the behaviour phase that produced it."""

    linear_x: float = 0.0
    angular_z: float = 0.0
    phase: str = "IDLE"


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def compute_docking_command(
    detected: bool,
    range_m: float,
    lateral_offset_m: float,
    bearing_rad: float,
    aligned: bool,
    config: DockingControllerConfig = DockingControllerConfig(),
) -> DockingCommand:
    """Compute the next docking velocity command.

    ``lateral_offset_m`` is accepted for symmetry with the docking error and to
    keep the interface stable, but the in-plane correction is driven by
    ``bearing_rad`` (the angle to the marker), which already captures the offset.
    """
    if aligned:
        return DockingCommand(0.0, 0.0, "DOCKED")

    if not detected:
        return DockingCommand(0.0, config.search_yaw_rate_radps, "SEARCH")

    angular_z = _clamp(config.bearing_gain * bearing_rad, config.max_angular_radps)

    if abs(bearing_rad) > config.align_first_bearing_rad:
        # Too far off-axis to drive safely; rotate to face the dock first.
        return DockingCommand(0.0, angular_z, "ALIGN")

    # Forward speed proportional to range (naturally slows on approach), tapered
    # by how well we are pointed at the marker, and capped at the limit.
    linear_x = min(config.range_gain * max(range_m, 0.0), config.max_linear_mps)
    linear_x *= math.cos(bearing_rad)
    return DockingCommand(linear_x, angular_z, "APPROACH")
