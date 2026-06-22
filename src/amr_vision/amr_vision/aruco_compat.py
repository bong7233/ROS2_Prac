"""Version-tolerant helpers around the OpenCV ArUco API.

OpenCV changed the ArUco interface between 4.6 (the ``python3-opencv`` shipped
with Ubuntu 24.04 / ROS 2 Jazzy) and 4.7+ (recent pip wheels). The detector
class, the parameter constructor, and the marker image generator were all
renamed. This module hides those differences with feature detection so the rest
of ``amr_vision`` can stay version agnostic and the same code runs in CI and on
a developer machine.
"""
from __future__ import annotations

from typing import List, Optional, Tuple

import cv2
import numpy as np

DEFAULT_DICTIONARY = "DICT_4X4_50"


def get_dictionary(name: str = DEFAULT_DICTIONARY):
    """Return a predefined ArUco dictionary by name (e.g. ``DICT_4X4_50``)."""
    try:
        dict_id = getattr(cv2.aruco, name)
    except AttributeError as exc:  # pragma: no cover - guards typos in config
        raise ValueError(f"unknown ArUco dictionary: {name}") from exc

    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(dict_id)
    return cv2.aruco.Dictionary_get(dict_id)  # OpenCV < 4.7 fallback


def generate_marker(dictionary, marker_id: int, side_pixels: int) -> np.ndarray:
    """Render a single marker as a grayscale ``side_pixels`` square image."""
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(dictionary, marker_id, side_pixels)
    return cv2.aruco.drawMarker(dictionary, marker_id, side_pixels)  # OpenCV < 4.7


def detect_markers(
    image: np.ndarray,
    dictionary,
) -> Tuple[List[np.ndarray], Optional[np.ndarray]]:
    """Detect markers and return ``(corners, ids)``.

    ``corners`` is a list of ``(1, 4, 2)`` float arrays (OpenCV's native shape)
    and ``ids`` is an ``(N, 1)`` int array, or ``None`` when nothing is found.
    The input may be grayscale or BGR; it is converted to gray internally.
    """
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    if hasattr(cv2.aruco, "ArucoDetector"):  # OpenCV >= 4.7
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        corners, ids, _ = detector.detectMarkers(gray)
    else:  # OpenCV 4.6 functional API
        params = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)

    return list(corners), ids
