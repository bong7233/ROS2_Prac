"""Unit tests for the ROS-free docking control law."""
import math

from amr_docking.docking_controller import (
    DockingControllerConfig,
    compute_docking_command,
)

CONFIG = DockingControllerConfig()


def _cmd(detected=True, range_m=1.0, lateral=0.0, bearing=0.0, aligned=False):
    return compute_docking_command(
        detected, range_m, lateral, bearing, aligned, CONFIG
    )


def test_docked_stops_and_holds():
    cmd = _cmd(aligned=True, range_m=0.45)
    assert cmd.phase == "DOCKED"
    assert cmd.linear_x == 0.0
    assert cmd.angular_z == 0.0


def test_no_marker_stops_by_default():
    cmd = _cmd(detected=False)
    assert cmd.phase == "SEARCH"
    assert cmd.linear_x == 0.0
    assert cmd.angular_z == 0.0


def test_no_marker_rotates_when_search_enabled():
    config = DockingControllerConfig(search_yaw_rate_radps=0.3)
    cmd = compute_docking_command(False, 0.0, 0.0, 0.0, False, config)
    assert cmd.phase == "SEARCH"
    assert cmd.angular_z == 0.3
    assert cmd.linear_x == 0.0


def test_marker_to_the_left_turns_left():
    # Positive bearing is to the left -> positive (CCW) yaw.
    cmd = _cmd(bearing=0.5)
    assert cmd.angular_z > 0.0


def test_marker_to_the_right_turns_right():
    cmd = _cmd(bearing=-0.5)
    assert cmd.angular_z < 0.0


def test_large_bearing_aligns_before_driving():
    cmd = _cmd(range_m=2.0, bearing=0.5)
    assert cmd.phase == "ALIGN"
    assert cmd.linear_x == 0.0


def test_small_bearing_drives_forward():
    cmd = _cmd(range_m=2.0, bearing=0.02)
    assert cmd.phase == "APPROACH"
    assert cmd.linear_x > 0.0


def test_forward_speed_decreases_as_range_shrinks():
    far = _cmd(range_m=1.0, bearing=0.0)
    near = _cmd(range_m=0.3, bearing=0.0)
    assert far.linear_x > near.linear_x
    assert near.linear_x >= 0.0


def test_commands_respect_limits():
    cmd = _cmd(range_m=100.0, bearing=1.0)
    # Big bearing keeps us in ALIGN, so check the angular cap there.
    assert abs(cmd.angular_z) <= CONFIG.max_angular_radps + 1e-9
    forward = _cmd(range_m=100.0, bearing=0.0)
    assert forward.linear_x <= CONFIG.max_linear_mps + 1e-9


def test_angular_magnitude_grows_with_bearing():
    small = _cmd(range_m=2.0, bearing=0.05)
    large = _cmd(range_m=2.0, bearing=0.15)
    assert abs(large.angular_z) > abs(small.angular_z)
