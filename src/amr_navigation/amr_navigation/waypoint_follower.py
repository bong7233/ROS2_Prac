"""ROS-free waypoint-following control law for ``amr_navigation``.

A simple pure-pursuit-style follower: given the robot pose and a list of (x, y)
waypoints in the odom frame, it turns toward the active waypoint and drives
forward, advancing through the list until done. Keeping it ROS-free makes the
behaviour unit-testable without a running graph, matching the split used by
``amr_docking`` and ``amr_vision``.

This exercises the otherwise-unused ``AUTO_READY`` / ``AUTO_RUNNING`` modes
without pulling in a full Nav2 stack. The command goes through the same
``/cmd_vel`` -> safety-monitor path as manual jog and docking.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple

Waypoint = Tuple[float, float]


@dataclass
class WaypointFollowerConfig:
    """Tunable gains and limits for the waypoint follower."""

    max_linear_mps: float = 0.4
    max_angular_radps: float = 1.0
    heading_gain: float = 1.5
    distance_gain: float = 0.8
    waypoint_tolerance_m: float = 0.15
    # If the heading error exceeds this, turn in place before driving forward.
    turn_in_place_rad: float = 0.5


@dataclass
class WaypointCommand:
    """A velocity command plus follower progress."""

    linear_x: float = 0.0
    angular_z: float = 0.0
    target_index: int = 0
    distance_to_target: float = 0.0
    done: bool = False


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def follow_waypoints(
    x: float,
    y: float,
    yaw: float,
    waypoints: Sequence[Waypoint],
    target_index: int,
    config: WaypointFollowerConfig = WaypointFollowerConfig(),
) -> WaypointCommand:
    """Compute the next command and the (possibly advanced) target index.

    Waypoints already within ``waypoint_tolerance_m`` are skipped, so the index
    advances as the robot arrives. When the list is exhausted the command is zero
    with ``done=True``.
    """
    index = target_index
    while index < len(waypoints):
        tx, ty = waypoints[index]
        dx = tx - x
        dy = ty - y
        distance = math.hypot(dx, dy)
        if distance > config.waypoint_tolerance_m:
            break
        index += 1
    else:
        return WaypointCommand(0.0, 0.0, index, 0.0, done=True)

    heading_error = normalize_angle(math.atan2(dy, dx) - yaw)
    angular_z = _clamp(config.heading_gain * heading_error, config.max_angular_radps)

    if abs(heading_error) > config.turn_in_place_rad:
        linear_x = 0.0
    else:
        linear_x = min(config.distance_gain * distance, config.max_linear_mps)
        linear_x *= math.cos(heading_error)

    return WaypointCommand(linear_x, angular_z, index, distance, done=False)
