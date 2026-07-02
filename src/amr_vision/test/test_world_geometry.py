"""Tests for the world->camera marker geometry used by the closed loop."""
import math

from amr_vision import marker_docking
from amr_vision.marker_docking import (
    optical_to_base_point,
    world_marker_to_camera_center,
)


def test_marker_straight_ahead():
    # Robot at origin facing +x, marker 2 m ahead, camera 0.4 m forward.
    center = world_marker_to_camera_center(
        0.0, 0.0, 0.0, 2.0, 0.0, camera_forward_offset_m=0.4
    )
    forward, left, up = optical_to_base_point(center)
    assert math.isclose(forward, 1.6, abs_tol=1e-9)
    assert math.isclose(left, 0.0, abs_tol=1e-9)
    assert math.isclose(up, 0.0, abs_tol=1e-9)


def test_marker_to_the_left():
    center = world_marker_to_camera_center(0.0, 0.0, 0.0, 2.0, 0.5)
    forward, left, _up = optical_to_base_point(center)
    assert math.isclose(forward, 2.0, abs_tol=1e-9)
    assert math.isclose(left, 0.5, abs_tol=1e-9)


def test_robot_rotation_is_accounted_for():
    # Robot facing +y (90 deg); marker due north at (0, 2) is straight ahead.
    center = world_marker_to_camera_center(0.0, 0.0, math.pi / 2, 0.0, 2.0)
    forward, left, _up = optical_to_base_point(center)
    assert math.isclose(forward, 2.0, abs_tol=1e-6)
    assert math.isclose(left, 0.0, abs_tol=1e-6)


def test_camera_height_offset():
    center = world_marker_to_camera_center(
        0.0, 0.0, 0.0, 1.0, 0.0, marker_height_m=0.5, camera_height_m=0.3
    )
    _forward, _left, up = optical_to_base_point(center)
    assert math.isclose(up, 0.2, abs_tol=1e-9)


def test_marker_behind_robot_has_negative_forward():
    center = world_marker_to_camera_center(0.0, 0.0, 0.0, -1.0, 0.0)
    forward, _left, _up = optical_to_base_point(center)
    assert forward < 0.0


def test_round_trip_matches_optical_to_base():
    # Driving forward should reduce the forward distance to a fixed marker.
    near = optical_to_base_point(
        world_marker_to_camera_center(1.0, 0.0, 0.0, 3.0, 0.0)
    )[0]
    far = optical_to_base_point(
        world_marker_to_camera_center(0.0, 0.0, 0.0, 3.0, 0.0)
    )[0]
    assert near < far
