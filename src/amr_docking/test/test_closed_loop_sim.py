"""End-to-end closed-loop docking simulation, without ROS.

Wires the real perception (amr_vision) and the real control law together and
integrates the robot motion over time:

    render marker view -> detect docking error -> compute command -> move robot

If the perception geometry, the optical/base frame conventions, and the control
law all agree, the robot converges on the dock and the controller reports DOCKED.
This catches integration bugs that per-module tests cannot.
"""
import math

import pytest

from amr_docking.docking_controller import DockingControllerConfig, compute_docking_command

marker_docking = pytest.importorskip("amr_vision.marker_docking")

WIDTH = 640
HEIGHT = 480
MARKER_ID = 3
MARKER_LENGTH = 0.20
CAMERA_MATRIX = marker_docking.default_camera_matrix(WIDTH, HEIGHT, 70.0)
CAMERA_FORWARD_OFFSET = 0.40


def _simulate(marker_x, marker_y, max_steps=2000, dt=0.1):
    config = DockingControllerConfig()
    robot_x = robot_y = robot_yaw = 0.0
    last = None
    for _ in range(max_steps):
        center = marker_docking.world_marker_to_camera_center(
            robot_x, robot_y, robot_yaw, marker_x, marker_y,
            camera_forward_offset_m=CAMERA_FORWARD_OFFSET,
        )
        image = marker_docking.synthesize_marker_image(
            CAMERA_MATRIX, WIDTH, HEIGHT, MARKER_ID, MARKER_LENGTH,
            center_optical=center,
        )
        error = marker_docking.detect_docking_error(
            image, MARKER_ID, MARKER_LENGTH, CAMERA_MATRIX
        )
        aligned = marker_docking.is_aligned(error)
        command = compute_docking_command(
            error.detected, error.range_m, error.lateral_offset_m,
            error.bearing_rad, aligned, config,
        )
        last = (command, error, robot_x, robot_y)
        if command.phase == "DOCKED":
            return last, True

        robot_x += command.linear_x * math.cos(robot_yaw) * dt
        robot_y += command.linear_x * math.sin(robot_yaw) * dt
        robot_yaw += command.angular_z * dt
    return last, False


def test_robot_docks_to_marker_dead_ahead():
    (command, error, robot_x, _robot_y), docked = _simulate(2.0, 0.0)
    assert docked
    assert command.phase == "DOCKED"
    assert robot_x > 0.5  # actually drove forward


def test_robot_docks_to_offset_marker():
    # Marker ahead and to the left: requires ALIGN then APPROACH.
    (command, _error, robot_x, robot_y), docked = _simulate(2.0, 0.35)
    assert docked
    assert robot_x > 0.5
    assert robot_y > 0.0  # steered toward the marker side
