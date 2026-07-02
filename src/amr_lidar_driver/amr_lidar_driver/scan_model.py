"""ROS-free 2D laser-scan model for the mock LiDAR.

Casts rays from a sensor pose inside a rectangular room (optionally with circular
obstacles) and returns the range for each beam. Keeping this free of ROS imports
makes the geometry unit-testable without a running graph, matching the split used
by ``amr_vision`` and ``amr_docking``.

The room is centered on the world origin with half-extents ``half_x``/``half_y``;
the sensor sits at ``(sensor_x, sensor_y)`` facing ``sensor_yaw``. Each beam angle
is measured in the sensor frame, so the returned list maps directly onto a
``sensor_msgs/LaserScan``.
"""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple

Obstacle = Tuple[float, float, float]  # (center_x, center_y, radius)


def beam_angles(angle_min: float, angle_max: float, count: int) -> List[float]:
    """Return the ``count`` beam angles spanning ``[angle_min, angle_max]``."""
    if count <= 1:
        return [angle_min]
    increment = (angle_max - angle_min) / (count - 1)
    return [angle_min + i * increment for i in range(count)]


def _room_exit_distance(
    px: float, py: float, dx: float, dy: float, half_x: float, half_y: float
) -> float:
    """Distance from an interior point to the rectangular wall along a ray."""
    tmax = math.inf
    if dx > 0.0:
        tmax = min(tmax, (half_x - px) / dx)
    elif dx < 0.0:
        tmax = min(tmax, (-half_x - px) / dx)
    if dy > 0.0:
        tmax = min(tmax, (half_y - py) / dy)
    elif dy < 0.0:
        tmax = min(tmax, (-half_y - py) / dy)
    return tmax


def _circle_distance(
    px: float, py: float, dx: float, dy: float, cx: float, cy: float, radius: float
) -> float:
    """Nearest positive ray-circle intersection distance, or ``inf``."""
    ox = px - cx
    oy = py - cy
    b = ox * dx + oy * dy
    c = ox * ox + oy * oy - radius * radius
    discriminant = b * b - c
    if discriminant < 0.0:
        return math.inf
    sqrt_disc = math.sqrt(discriminant)
    t_near = -b - sqrt_disc
    if t_near > 0.0:
        return t_near
    t_far = -b + sqrt_disc
    return t_far if t_far > 0.0 else math.inf


def compute_scan(
    sensor_x: float,
    sensor_y: float,
    sensor_yaw: float,
    angle_min: float,
    angle_max: float,
    count: int,
    range_min: float,
    range_max: float,
    half_x: float,
    half_y: float,
    obstacles: Sequence[Obstacle] = (),
) -> List[float]:
    """Compute per-beam ranges for the mock LiDAR."""
    ranges: List[float] = []
    for beam in beam_angles(angle_min, angle_max, count):
        world_angle = sensor_yaw + beam
        dx = math.cos(world_angle)
        dy = math.sin(world_angle)

        distance = _room_exit_distance(sensor_x, sensor_y, dx, dy, half_x, half_y)
        for cx, cy, radius in obstacles:
            distance = min(
                distance, _circle_distance(sensor_x, sensor_y, dx, dy, cx, cy, radius)
            )

        distance = min(distance, range_max)
        if distance < range_min:
            distance = range_min
        ranges.append(distance)
    return ranges
