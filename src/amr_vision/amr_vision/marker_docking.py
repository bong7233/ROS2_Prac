"""ROS-free docking-marker geometry for ``amr_vision``.

These functions turn an ArUco marker detection into an AMR docking error
expressed in the robot base frame (REP-103: ``x`` forward, ``y`` left,
``z`` up). Keeping the math free of ROS imports makes the perception pipeline
unit-testable without a running graph, which is also the only part of this
package that can be exercised in an environment without ROS installed.

Frames
------
* Camera optical frame (REP-103): ``x`` right, ``y`` down, ``z`` forward.
* Robot base frame (REP-103): ``x`` forward, ``y`` left, ``z`` up.

``solvePnP`` returns the marker pose in the camera optical frame; everything
reported to the rest of the stack is remapped into the base frame so the docking
controller can reason in robot terms.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

from amr_vision import aruco_compat


@dataclass
class DockingError:
    """Docking geometry derived from a single marker detection.

    Only translation-based signals are reported. The dock-face yaw of a single
    planar marker is subject to the well-known planar pose ambiguity and is left
    out deliberately; resolving it reliably (e.g. with a multi-marker dock board)
    is tracked as the next increment.
    """

    detected: bool = False
    marker_id: int = -1
    range_m: float = 0.0
    lateral_offset_m: float = 0.0
    bearing_rad: float = 0.0


def default_camera_matrix(
    width: int,
    height: int,
    horizontal_fov_deg: float = 70.0,
) -> np.ndarray:
    """Build a pinhole intrinsic matrix from image size and horizontal FOV."""
    fx = (width / 2.0) / math.tan(math.radians(horizontal_fov_deg) / 2.0)
    fy = fx
    cx = width / 2.0
    cy = height / 2.0
    return np.array(
        [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def marker_object_points(marker_length_m: float) -> np.ndarray:
    """Marker-frame corner coordinates matching OpenCV's corner ordering.

    Corners are top-left, top-right, bottom-right, bottom-left, with the marker
    centered at the origin in the ``z = 0`` plane (x right, y up).
    """
    half = marker_length_m / 2.0
    return np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )


def estimate_marker_pose(
    corners: np.ndarray,
    marker_length_m: float,
    camera_matrix: np.ndarray,
    dist_coeffs: Optional[np.ndarray] = None,
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """Estimate ``(rvec, tvec)`` of a square marker in the camera optical frame.

    Uses ``SOLVEPNP_ITERATIVE``, which handles the coplanar marker corners
    robustly. (``SOLVEPNP_IPPE_SQUARE`` is faster but was observed to return
    non-finite poses for some corner layouts, and ``estimatePoseSingleMarkers``
    was removed in OpenCV 4.7.) Returns ``None`` if the solve fails or yields a
    non-finite pose.
    """
    if dist_coeffs is None:
        dist_coeffs = np.zeros((5, 1), dtype=np.float64)

    object_points = marker_object_points(marker_length_m)
    image_points = np.asarray(corners, dtype=np.float64).reshape(4, 2)

    ok, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok or not (np.all(np.isfinite(rvec)) and np.all(np.isfinite(tvec))):
        return None
    return rvec, tvec


def optical_to_base_point(tvec: np.ndarray) -> Tuple[float, float, float]:
    """Map a point from the camera optical frame to the robot base frame."""
    x, y, z = np.asarray(tvec, dtype=np.float64).reshape(3)
    forward = float(z)
    left = float(-x)
    up = float(-y)
    return forward, left, up


def compute_docking_error(tvec: np.ndarray, marker_id: int) -> DockingError:
    """Convert a marker translation into a base-frame docking error.

    * ``range_m`` - forward distance from the camera to the marker.
    * ``lateral_offset_m`` - sideways offset, positive when the marker is to the
      robot's left.
    * ``bearing_rad`` - angle to the marker center, positive to the left.
    """
    forward, left, _up = optical_to_base_point(tvec)
    return DockingError(
        detected=True,
        marker_id=int(marker_id),
        range_m=forward,
        lateral_offset_m=left,
        bearing_rad=math.atan2(left, forward),
    )


def is_aligned(
    error: DockingError,
    max_range_m: float = 0.6,
    min_range_m: float = 0.15,
    max_lateral_m: float = 0.03,
    max_bearing_rad: float = 0.05,
) -> bool:
    """Return ``True`` when the robot is docked within the given tolerances."""
    if not error.detected:
        return False
    return (
        min_range_m <= error.range_m <= max_range_m
        and abs(error.lateral_offset_m) <= max_lateral_m
        and abs(error.bearing_rad) <= max_bearing_rad
    )


def detect_docking_error(
    image: np.ndarray,
    marker_id: int,
    marker_length_m: float,
    camera_matrix: np.ndarray,
    dist_coeffs: Optional[np.ndarray] = None,
    dictionary=None,
) -> DockingError:
    """End-to-end: detect ``marker_id`` in ``image`` and return a docking error.

    Returns ``DockingError(detected=False)`` if the target marker is absent.
    """
    if dictionary is None:
        dictionary = aruco_compat.get_dictionary()

    corners, ids = aruco_compat.detect_markers(image, dictionary)
    if ids is None:
        return DockingError(detected=False)

    ids_flat = np.asarray(ids).reshape(-1)
    for index, found_id in enumerate(ids_flat):
        if int(found_id) != marker_id:
            continue
        pose = estimate_marker_pose(
            corners[index], marker_length_m, camera_matrix, dist_coeffs
        )
        if pose is None:
            return DockingError(detected=False)
        _rvec, tvec = pose
        return compute_docking_error(tvec, marker_id)

    return DockingError(detected=False)


def synthesize_marker_image(
    camera_matrix: np.ndarray,
    width: int,
    height: int,
    marker_id: int,
    marker_length_m: float,
    center_optical: Tuple[float, float, float] = (0.0, 0.0, 1.0),
    yaw_rad: float = 0.0,
    dictionary=None,
    dist_coeffs: Optional[np.ndarray] = None,
    marker_pixels: int = 240,
    background: int = 255,
) -> np.ndarray:
    """Render a grayscale view of an upright marker facing the camera.

    ``center_optical`` is the marker center in the camera optical frame
    (x right, y down, z forward) and ``yaw_rad`` rotates the marker about the
    camera's vertical axis (positive turns the dock face to the robot's left).
    The four marker corners are placed directly in 3D camera coordinates, which
    keeps the marker upright and detectable regardless of OpenCV's pose
    conventions. The result can be fed straight back through
    :func:`detect_docking_error`, so the mock camera node and the unit tests
    share one rendering path.
    """
    if dictionary is None:
        dictionary = aruco_compat.get_dictionary()
    if dist_coeffs is None:
        dist_coeffs = np.zeros((5, 1), dtype=np.float64)

    marker = aruco_compat.generate_marker(dictionary, marker_id, marker_pixels)
    quiet = max(1, marker_pixels // 5)
    canvas_size = marker_pixels + 2 * quiet
    canvas = np.full((canvas_size, canvas_size), 255, dtype=np.uint8)
    canvas[quiet : quiet + marker_pixels, quiet : quiet + marker_pixels] = marker

    # Physical half-size of the full canvas (marker plus quiet zone) in meters.
    physical = marker_length_m * canvas_size / marker_pixels
    half = physical / 2.0
    center = np.asarray(center_optical, dtype=np.float64).reshape(3)

    # In-plane corner offsets for an upright marker, in canvas pixel order
    # (top-left, top-right, bottom-right, bottom-left). y is down to match image
    # rows, so the rendered marker is never mirrored.
    offsets = np.array(
        [
            [-half, -half],
            [half, -half],
            [half, half],
            [-half, half],
        ],
        dtype=np.float64,
    )
    cos_y = math.cos(yaw_rad)
    sin_y = math.sin(yaw_rad)
    corners_3d = np.empty((4, 3), dtype=np.float64)
    for i, (dx, dy) in enumerate(offsets):
        # Rotate the in-plane offset about the camera vertical (optical y) axis.
        corners_3d[i, 0] = center[0] + dx * cos_y
        corners_3d[i, 1] = center[1] + dy
        corners_3d[i, 2] = center[2] - dx * sin_y

    source = np.array(
        [
            [0.0, 0.0],
            [canvas_size - 1.0, 0.0],
            [canvas_size - 1.0, canvas_size - 1.0],
            [0.0, canvas_size - 1.0],
        ],
        dtype=np.float64,
    )
    projected, _ = cv2.projectPoints(
        corners_3d,
        np.zeros((3, 1), dtype=np.float64),
        np.zeros((3, 1), dtype=np.float64),
        camera_matrix,
        dist_coeffs,
    )
    destination = projected.reshape(4, 2)
    homography, _ = cv2.findHomography(source, destination)
    return cv2.warpPerspective(
        canvas,
        homography,
        (width, height),
        borderValue=background,
    )
