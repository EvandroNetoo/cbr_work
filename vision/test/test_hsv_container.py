from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from vision.hsv_container import (
    Blob, PixelObservation, area_thresholds_for_height, detect_blobs,
    pixel_on_base_plane, update_tracks,
)


def test_pixel_area_thresholds_and_border_classification():
    mask = np.zeros((100, 140), np.uint8)
    cv2.rectangle(mask, (30, 20), (49, 39), 255, -1)  # 400 px, complete
    cv2.rectangle(mask, (0, 50), (9, 79), 255, -1)    # 300 px, clipped
    found = detect_blobs({1: mask}, mask.shape, 350, 250, border_margin=0)
    assert sorted((blob.area, blob.partial) for blob in found) == [
        (300, True), (400, False)]
    assert next(blob.center for blob in found if not blob.partial) == (39.5, 29.5)
    assert len(detect_blobs({1: mask}, mask.shape, 450, 350)) == 0


def test_three_distinct_frames_and_color_are_needed_for_matching():
    tracks = []
    for frame, x in [(1, 40), (2, 43), (3, 42)]:
        update_tracks(tracks, [PixelObservation(
            frame, Blob(1, (x, 30), 500, False), None, None)], 4)
    assert len(tracks) == 1
    assert len(tracks[0].observations) == 3
    update_tracks(tracks, [PixelObservation(
        3, Blob(1, (41, 30), 500, False), None, None)], 4)
    assert len(tracks) == 2  # Same frame cannot count twice in one track.
    update_tracks(tracks, [PixelObservation(
        4, Blob(2, (42, 30), 500, False), None, None)], 4)
    assert len(tracks) == 3


def test_rectified_pixel_intersects_known_base_plane():
    transform = SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=0.1, y=-0.2, z=0.4),
        rotation=SimpleNamespace(x=1.0, y=0.0, z=0.0, w=0.0),
    ))
    k = np.array([[200., 0., 50.], [0., 200., 40.], [0., 0., 1.]])
    point = pixel_on_base_plane((70., 60.), k, transform, 0.1)
    np.testing.assert_allclose(point, [0.13, -0.23, 0.1])
    assert pixel_on_base_plane((70., 60.), k, transform, 0.5) is None


@pytest.mark.parametrize('height, expected', [
    (0.0, (4000, 1800)),
    (0.05, (4000, 1800)),
    (0.050001, (4000, 2500)),
    (0.075, (4000, 2500)),
    (0.075001, (6500, 2500)),
    (0.10, (6500, 2500)),
    (0.100001, (6500, 3500)),
    (0.125, (6500, 3500)),
    (0.125001, (9000, 3500)),
])
def test_complete_and_partial_height_bands_are_independent(height, expected):
    assert area_thresholds_for_height(
        height, (4000, 6500, 9000), (1800, 2500, 3500)) == expected
