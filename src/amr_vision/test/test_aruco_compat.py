"""Round-trip tests for the version-tolerant ArUco helpers."""
import numpy as np

from amr_vision import aruco_compat


def test_generate_and_detect_round_trip():
    dictionary = aruco_compat.get_dictionary()
    marker = aruco_compat.generate_marker(dictionary, marker_id=7, side_pixels=200)

    # Pad with a white quiet zone so the detector has the required border.
    quiet = 60
    scene = np.full(
        (marker.shape[0] + 2 * quiet, marker.shape[1] + 2 * quiet),
        255,
        dtype=np.uint8,
    )
    scene[quiet:-quiet, quiet:-quiet] = marker

    corners, ids = aruco_compat.detect_markers(scene, dictionary)

    assert ids is not None
    assert 7 in np.asarray(ids).reshape(-1).tolist()
    assert len(corners) == len(np.asarray(ids).reshape(-1))


def test_detect_returns_none_on_blank_image():
    dictionary = aruco_compat.get_dictionary()
    blank = np.full((240, 320), 255, dtype=np.uint8)

    corners, ids = aruco_compat.detect_markers(blank, dictionary)

    assert ids is None
    assert corners == []
