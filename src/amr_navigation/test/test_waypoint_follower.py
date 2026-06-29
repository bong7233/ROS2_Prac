"""Unit and closed-loop tests for the ROS-free waypoint follower."""
import math

from amr_navigation.waypoint_follower import (
    WaypointFollowerConfig,
    follow_waypoints,
    normalize_angle,
)

CONFIG = WaypointFollowerConfig()


def test_empty_or_finished_is_done():
    cmd = follow_waypoints(0.0, 0.0, 0.0, [], 0, CONFIG)
    assert cmd.done
    assert cmd.linear_x == 0.0 and cmd.angular_z == 0.0


def test_drives_toward_waypoint_dead_ahead():
    cmd = follow_waypoints(0.0, 0.0, 0.0, [(1.0, 0.0)], 0, CONFIG)
    assert not cmd.done
    assert cmd.linear_x > 0.0
    assert abs(cmd.angular_z) < 1e-6
    assert math.isclose(cmd.distance_to_target, 1.0, abs_tol=1e-9)


def test_turns_in_place_when_waypoint_to_the_side():
    # Waypoint due north while facing +x -> 90 deg error, turn left, no drive.
    cmd = follow_waypoints(0.0, 0.0, 0.0, [(0.0, 1.0)], 0, CONFIG)
    assert cmd.angular_z > 0.0
    assert cmd.linear_x == 0.0


def test_turns_right_for_waypoint_on_the_right():
    cmd = follow_waypoints(0.0, 0.0, 0.0, [(0.0, -1.0)], 0, CONFIG)
    assert cmd.angular_z < 0.0


def test_advances_past_reached_waypoints():
    # First waypoint is already under tolerance -> should target the second.
    cmd = follow_waypoints(0.0, 0.0, 0.0, [(0.05, 0.0), (1.0, 0.0)], 0, CONFIG)
    assert cmd.target_index == 1
    assert not cmd.done


def test_done_when_last_waypoint_reached():
    cmd = follow_waypoints(1.0, 0.0, 0.0, [(1.0, 0.0)], 0, CONFIG)
    assert cmd.done


def test_limits_are_respected():
    cmd = follow_waypoints(0.0, 0.0, 0.0, [(100.0, 0.0)], 0, CONFIG)
    assert cmd.linear_x <= CONFIG.max_linear_mps + 1e-9
    cmd2 = follow_waypoints(0.0, 0.0, math.pi, [(0.0, 100.0)], 0, CONFIG)
    assert abs(cmd2.angular_z) <= CONFIG.max_angular_radps + 1e-9


def test_normalize_angle():
    assert math.isclose(normalize_angle(0.5), 0.5, abs_tol=1e-9)
    assert math.isclose(normalize_angle(2 * math.pi + 0.3), 0.3, abs_tol=1e-9)
    # +/-pi are both valid normalizations of an odd multiple of pi.
    assert math.isclose(abs(normalize_angle(3 * math.pi)), math.pi, abs_tol=1e-9)


def _simulate(waypoints, max_steps=4000, dt=0.05):
    x = y = yaw = 0.0
    index = 0
    for _ in range(max_steps):
        cmd = follow_waypoints(x, y, yaw, waypoints, index, CONFIG)
        index = cmd.target_index
        if cmd.done:
            return (x, y), index, True
        x += cmd.linear_x * math.cos(yaw) * dt
        y += cmd.linear_x * math.sin(yaw) * dt
        yaw += cmd.angular_z * dt
    return (x, y), index, False


def test_follows_a_square_loop_to_completion():
    waypoints = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]
    (x, y), index, done = _simulate(waypoints)
    assert done
    # Finished at (or very near) the last waypoint.
    assert math.hypot(x - 0.0, y - 0.0) < CONFIG.waypoint_tolerance_m + 0.1
