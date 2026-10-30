"""Native C++ vision and CUDA preprocessing parity tests."""

import os

import cv2
import numpy as np
import pytest

from upper_insert.vision import CudaPreprocessor, vision_locator as vision


def _scene(occluded=False):
    frame = np.full((720, 1280, 3), 180, np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(vision.ARUCO_DICT_ID)
    for marker, x in ((0, 180), (1, 760)):
        card = vision.draw_aruco_marker(dictionary, marker, 120)
        frame[240:360, x:x + 120] = cv2.cvtColor(card, cv2.COLOR_GRAY2BGR)
    if occluded:
        frame[240:265, 760:785] = 180
    return frame


def _ground_truth(x):
    return np.float32([[[x, 240], [x + 120, 240], [x + 120, 360], [x, 360]]])


@pytest.mark.parametrize('scene', ['both', 'occluded', 'missing', 'noise', 'perspective'])
@pytest.mark.parametrize('pass_idx', [None, 0, 1, 2, 3, 4, 5])
def test_native_recovery_preserves_candidate_selection_and_corners(scene, pass_idx):
    frame = _scene(scene == 'occluded')
    expected_corners = {0: _ground_truth(180), 1: _ground_truth(760)}
    if scene == 'missing':
        frame[:, 700:] = 180
        expected_corners = {0: _ground_truth(180)}
    elif scene == 'noise':
        frame = np.random.default_rng(42).integers(0, 256, frame.shape, dtype=np.uint8)
        expected_corners = {}
    elif scene == 'perspective':
        transform = cv2.getPerspectiveTransform(
            np.float32([[0, 0], [1279, 0], [1279, 719], [0, 719]]),
            np.float32([[20, 10], [1170, 50], [1250, 700], [70, 660]]),
        )
        frame = cv2.warpPerspective(frame, transform, (1280, 720), borderValue=(180, 180, 180))
        expected_corners = {
            m: cv2.perspectiveTransform(c, transform) for m, c in expected_corners.items()
        }
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    sat = vision._bgr_to_sat(frame)
    without_sat, with_sat = {}, {}
    vision._recover_occluded_markers_single_frame(
        gray, None, without_sat, frame=frame, pass_idx=pass_idx,
    )
    vision._recover_occluded_markers_single_frame(
        gray, sat, with_sat, frame=frame, pass_idx=pass_idx,
    )
    assert set(without_sat) == set(with_sat)
    for marker in without_sat:
        np.testing.assert_array_equal(without_sat[marker], with_sat[marker])
        np.testing.assert_allclose(without_sat[marker], expected_corners[marker], atol=1.5)
    if pass_idx is None:
        assert set(without_sat) == set(expected_corners)


def test_native_saturation_and_optional_cuda_preprocessor_match_opencv():
    frame = np.empty((256, 256, 3), np.uint8)
    frame[:, :, 1] = np.arange(256, dtype=np.uint8)[:, None]
    frame[:, :, 2] = np.arange(256, dtype=np.uint8)[None, :]
    cuda = CudaPreprocessor() if os.environ.get('UPPER_INSERT_TEST_CUDA') == '1' else None
    try:
        for blue in (0, 64, 128, 192, 255):
            frame[:, :, 0] = blue
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            expected_sat = cv2.multiply(
                hsv[:, :, 1], np.minimum(hsv[:, :, 2], 115), scale=1. / 115.,
            )
            np.testing.assert_array_equal(vision._bgr_to_sat(frame), expected_sat)
            if cuda is not None:
                gray, sat = cuda(frame)
                np.testing.assert_array_equal(gray, cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
                np.testing.assert_array_equal(sat, expected_sat)
    finally:
        if cuda is not None:
            cuda.close()
