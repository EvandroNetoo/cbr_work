"""Checks for color sampling outside the four detected AprilTag edges."""

import cv2
import numpy as np
import pytest

from vision.cube_color import (
    UNKNOWN, CubeColorCriteria, classify_cube_color, confirmed_color,
    outer_edge_strips,
)
from vision.constants import BLUE, RED


CORNERS = np.array([
    [120.0, 91.0], [199.0, 84.0],
    [204.0, 153.0], [116.0, 157.0],
])


def classify(image, corners=CORNERS):
    return classify_cube_color(
        image, corners, ((0, 15), (140, 179)), (70, 138), 80, 30)


@pytest.mark.parametrize('color,bgr', [
    (RED, (0, 0, 255)), (BLUE, (255, 0, 0)),
])
def test_reads_outer_bands_of_perspective_skewed_tag(color, bgr):
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    strips = list(outer_edge_strips(CORNERS))
    assert len(strips) == 4
    # The inner boundary touches the detected edge, with no white-margin gap.
    expected = CORNERS[0] + (CORNERS[1] - CORNERS[0]) * 0.15
    assert np.allclose(strips[0][0], expected, atol=0.5)
    for polygon in strips:
        cv2.fillConvexPoly(image, polygon, bgr)
    center = CORNERS.mean(axis=0)
    label = np.rint(center + (CORNERS - center) * 1.25).astype(np.int32)
    cv2.fillConvexPoly(image, label, (255, 255, 255))
    detected, confidence = classify(image)
    assert detected == color
    assert confidence == pytest.approx(1.0)


def test_geometry_uses_corners_without_pose_or_camera_matrix():
    forward = list(outer_edge_strips(CORNERS))
    reverse = list(outer_edge_strips(CORNERS[::-1]))
    assert len(forward) == len(reverse) == 4
    assert sorted(tuple(np.rint(p.mean(axis=0)).astype(int)) for p in forward) == (
        sorted(tuple(np.rint(p.mean(axis=0)).astype(int)) for p in reverse))


def test_unknown_without_visible_color_or_with_conflicting_bands():
    white = np.full((240, 320, 3), 255, dtype=np.uint8)
    assert classify(white) == (UNKNOWN, 0.0)
    image = np.zeros_like(white)
    strips = list(outer_edge_strips(CORNERS))
    cv2.fillConvexPoly(image, strips[0], (0, 0, 255))
    cv2.fillConvexPoly(image, strips[2], (255, 0, 0))
    assert classify(image)[0] == UNKNOWN


def test_conflicting_frames_are_unknown_and_two_matching_frames_confirm():
    assert confirmed_color([(RED, 0.9)]) == (UNKNOWN, 0.0, 1)
    assert confirmed_color([(RED, 0.9), (RED, 0.95)])[0] == RED
    assert confirmed_color([(RED, 0.9), (BLUE, 0.95)])[0] == UNKNOWN


def test_acceptance_thresholds_can_be_tuned():
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.fillConvexPoly(image, list(outer_edge_strips(CORNERS))[0],
                       (0, 0, 255))
    center = CORNERS.mean(axis=0)
    label = np.rint(center + (CORNERS - center) * 1.25).astype(np.int32)
    cv2.fillConvexPoly(image, label, (255, 255, 255))
    arguments = (image, CORNERS, ((0, 15), (140, 179)), (70, 138), 80, 30)
    assert classify_cube_color(*arguments)[0] == RED
    assert classify_cube_color(
        *arguments, CubeColorCriteria(min_colored_pixels=10000))[0] == UNKNOWN
    assert classify_cube_color(
        *arguments, CubeColorCriteria(min_colored_fraction=1.0))[0] == UNKNOWN
    assert confirmed_color(
        [(RED, 0.9), (RED, 0.9)],
        CubeColorCriteria(min_confirmed_frames=3))[0] == UNKNOWN
    votes = [(RED, 0.9), (RED, 0.9), (BLUE, 0.9)]
    assert confirmed_color(votes)[0] == UNKNOWN
    assert confirmed_color(
        votes, CubeColorCriteria(min_vote_share=0.60))[0] == RED


@pytest.mark.parametrize('override', [
    {'min_band_pixels': 0},
    {'min_colored_pixels': -1},
    {'min_colored_fraction': 1.1},
    {'min_colored_fraction': float('nan')},
    {'min_dominance': 0.5},
    {'min_confirmed_frames': 0},
    {'min_vote_share': 0.0},
])
def test_invalid_acceptance_thresholds_are_rejected(override):
    with pytest.raises(ValueError, match='cube color'):
        CubeColorCriteria(**override)
