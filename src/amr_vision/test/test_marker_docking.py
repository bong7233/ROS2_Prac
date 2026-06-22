"""Unit tests for the ROS-free docking geometry.

Each test synthesizes a marker view at a known pose, then runs the full
detect -> solvePnP -> docking-error pipeline and checks the recovered geometry,
so the rendering and detection paths are validated together. These run without
ROS; they only need OpenCV and NumPy.
"""
import math

import numpy as np

from amr_vision import marker_docking
from amr_vision.marker_docking import DockingError

WIDTH = 640
HEIGHT = 480
MARKER_ID = 3
MARKER_LENGTH = 0.20
CAMERA_MATRIX = marker_docking.default_camera_matrix(WIDTH, HEIGHT, 70.0)


def _render(center_optical, yaw_rad=0.0, marker_id=MARKER_ID):
    return marker_docking.synthesize_marker_image(
        CAMERA_MATRIX,
        WIDTH,
        HEIGHT,
        marker_id,
        MARKER_LENGTH,
        center_optical=center_optical,
        yaw_rad=yaw_rad,
    )


def _detect(image):
    return marker_docking.detect_docking_error(
        image, MARKER_ID, MARKER_LENGTH, CAMERA_MATRIX
    )


def test_centered_marker_recovers_range_and_zero_offset():
    error = _detect(_render((0.0, 0.0, 1.0)))

    assert error.detected
    assert error.marker_id == MARKER_ID
    assert abs(error.range_m - 1.0) <= 0.05
    assert abs(error.lateral_offset_m) <= 0.02
    assert abs(error.bearing_rad) <= 0.02


def test_range_grows_with_distance():
    near = _detect(_render((0.0, 0.0, 1.0)))
    far = _detect(_render((0.0, 0.0, 2.0)))

    assert near.detected and far.detected
    assert abs(far.range_m - 2.0) <= 0.12
    assert far.range_m > near.range_m + 0.5


def test_marker_to_the_right_reports_negative_lateral():
    # +x in the optical frame is to the robot's right -> negative "left" offset.
    error = _detect(_render((0.2, 0.0, 1.0)))

    assert error.detected
    assert abs(error.lateral_offset_m - (-0.2)) <= 0.03
    assert error.bearing_rad < 0.0


def test_marker_to_the_left_reports_positive_lateral():
    error = _detect(_render((-0.2, 0.0, 1.0)))

    assert error.detected
    assert abs(error.lateral_offset_m - 0.2) <= 0.03
    assert error.bearing_rad > 0.0


def test_translation_is_robust_to_dock_face_tilt():
    # Range/lateral must stay reliable even when the marker is yawed away.
    for yaw_deg in (-20.0, 20.0):
        error = _detect(_render((0.0, 0.0, 1.0), yaw_rad=math.radians(yaw_deg)))
        assert error.detected
        assert abs(error.range_m - 1.0) <= 0.08
        assert abs(error.lateral_offset_m) <= 0.03


def test_aligned_when_close_and_centered():
    error = _detect(_render((0.0, 0.0, 0.45)))

    assert marker_docking.is_aligned(error)


def test_not_aligned_when_far():
    error = _detect(_render((0.0, 0.0, 1.5)))

    assert error.detected
    assert not marker_docking.is_aligned(error)


def test_wrong_marker_id_is_not_reported():
    image = _render((0.0, 0.0, 1.0), marker_id=MARKER_ID + 2)

    error = _detect(image)

    assert not error.detected


def test_blank_image_is_not_detected():
    blank = np.full((HEIGHT, WIDTH), 255, dtype=np.uint8)

    error = _detect(blank)

    assert error == DockingError(detected=False)


def test_optical_to_base_point_mapping():
    # Optical (x right, y down, z forward) -> base (x forward, y left, z up).
    forward, left, up = marker_docking.optical_to_base_point([0.3, -0.1, 2.0])

    assert math.isclose(forward, 2.0)
    assert math.isclose(left, -0.3)
    assert math.isclose(up, 0.1)
