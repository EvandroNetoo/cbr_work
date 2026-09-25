"""Color-component container observations in the unwarped camera image."""

from dataclasses import dataclass, field
import math

import cv2
import numpy as np

from .geometry import rotation_from_quaternion


@dataclass(frozen=True)
class Blob:
    color: int
    center: tuple[float, float]
    area: int
    partial: bool


def area_thresholds_for_height(height_m, full_areas, partial_areas):
    """Select independent height bands for complete and clipped blobs."""
    full_index = 0 if height_m <= 0.075 else 1 if height_m <= 0.125 else 2
    partial_index = 0 if height_m <= 0.05 else 1 if height_m <= 0.10 else 2
    return full_areas[full_index], partial_areas[partial_index]


def detect_blobs(masks, image_shape, minimum_full, minimum_partial,
                 border_margin=0, max_area_fraction=0.85):
    """Return connected color regions; areas and centroids are mask pixels."""
    height, width = image_shape
    blobs = []
    for color, mask in masks.items():
        count, _, stats, centers = cv2.connectedComponentsWithStats(
            mask, connectivity=8)
        for label in range(1, count):
            x, y, box_width, box_height, area = map(int, stats[label])
            partial = (x <= border_margin or y <= border_margin or
                       x + box_width >= width - border_margin or
                       y + box_height >= height - border_margin)
            minimum = minimum_partial if partial else minimum_full
            if minimum <= area <= height * width * max_area_fraction:
                blobs.append(Blob(color, tuple(map(float, centers[label])),
                                  area, partial))
    return blobs


def pixel_on_base_plane(center, camera_matrix, camera_to_base, plane_z):
    """Intersect a rectified pixel ray with a known horizontal base plane."""
    fx, fy = float(camera_matrix[0, 0]), float(camera_matrix[1, 1])
    if fx <= 0 or fy <= 0 or not math.isfinite(plane_z):
        return None
    u, v = center
    ray = np.array([(u - camera_matrix[0, 2]) / fx,
                    (v - camera_matrix[1, 2]) / fy, 1.0])
    transform = camera_to_base.transform
    try:
        rotation = rotation_from_quaternion(transform.rotation)
    except ValueError:
        return None
    origin = np.array([transform.translation.x, transform.translation.y,
                       transform.translation.z], dtype=float)
    direction = rotation @ ray
    if abs(direction[2]) < 1e-8:
        return None
    distance = (plane_z - origin[2]) / direction[2]
    if distance <= 0 or not math.isfinite(distance):
        return None
    point = origin + distance * direction
    return point if np.all(np.isfinite(point)) else None


@dataclass
class PixelObservation:
    frame: int
    blob: Blob
    camera: object
    base: object


@dataclass
class PixelTrack:
    color: int
    observations: list[PixelObservation] = field(default_factory=list)

    @property
    def center(self):
        return np.median([item.blob.center for item in self.observations], axis=0)


def update_tracks(tracks, observations, tolerance_px):
    """Associate each color blob once per frame by its unwarped pixel center."""
    used = set()
    for observation in sorted(observations, key=lambda item: -item.blob.area):
        choices = [(float(np.linalg.norm(track.center - observation.blob.center)), i)
                   for i, track in enumerate(tracks)
                   if track.color == observation.blob.color
                   and track.observations[-1].frame != observation.frame
                   and i not in used]
        distance, index = min(choices, default=(math.inf, -1))
        if distance <= tolerance_px:
            tracks[index].observations.append(observation)
            used.add(index)
        else:
            tracks.append(PixelTrack(observation.blob.color, [observation]))
            used.add(len(tracks) - 1)
