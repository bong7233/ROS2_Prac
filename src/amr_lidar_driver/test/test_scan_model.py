"""Unit tests for the ROS-free laser-scan model."""
import math

from amr_lidar_driver import scan_model


def test_beam_count_matches_request():
    ranges = scan_model.compute_scan(
        0.0, 0.0, 0.0, -math.pi, math.pi, 360, 0.1, 10.0, 5.0, 5.0
    )
    assert len(ranges) == 360


def test_forward_beam_hits_front_wall():
    # Sensor at origin facing +x, room half-extent 5 -> wall at x=5.
    ranges = scan_model.compute_scan(
        0.0, 0.0, 0.0, 0.0, 0.0, 1, 0.1, 20.0, 5.0, 5.0
    )
    assert math.isclose(ranges[0], 5.0, abs_tol=1e-9)


def test_side_beam_hits_side_wall():
    ranges = scan_model.compute_scan(
        0.0, 0.0, 0.0, math.pi / 2, math.pi / 2, 1, 0.1, 20.0, 5.0, 3.0
    )
    assert math.isclose(ranges[0], 3.0, abs_tol=1e-9)


def test_obstacle_ahead_is_closer_than_wall():
    # Circle of radius 0.5 centered 2 m ahead -> near edge at 1.5 m.
    ranges = scan_model.compute_scan(
        0.0, 0.0, 0.0, 0.0, 0.0, 1, 0.1, 20.0, 5.0, 5.0, obstacles=[(2.0, 0.0, 0.5)]
    )
    assert math.isclose(ranges[0], 1.5, abs_tol=1e-6)


def test_ranges_are_clamped_to_max():
    ranges = scan_model.compute_scan(
        0.0, 0.0, 0.0, 0.0, 0.0, 1, 0.1, 2.0, 50.0, 50.0
    )
    assert ranges[0] == 2.0


def test_sensor_offset_changes_distance():
    # Moving toward the front wall shortens the forward range.
    near = scan_model.compute_scan(
        2.0, 0.0, 0.0, 0.0, 0.0, 1, 0.1, 20.0, 5.0, 5.0
    )[0]
    far = scan_model.compute_scan(
        0.0, 0.0, 0.0, 0.0, 0.0, 1, 0.1, 20.0, 5.0, 5.0
    )[0]
    assert math.isclose(near, 3.0, abs_tol=1e-9)
    assert near < far


def test_sensor_yaw_rotates_the_scan():
    # Facing +y, the first beam (sensor angle 0) hits the y wall at 4.
    ranges = scan_model.compute_scan(
        0.0, 0.0, math.pi / 2, 0.0, 0.0, 1, 0.1, 20.0, 5.0, 4.0
    )
    assert math.isclose(ranges[0], 4.0, abs_tol=1e-6)
