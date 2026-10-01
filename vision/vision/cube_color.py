"""Classify cube color in narrow bands outside the four detected tag edges."""

from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np

from .constants import BLUE, RED

UNKNOWN = 0


@dataclass(frozen=True)
class CubeColorCriteria:
    min_band_pixels: int = 20
    min_colored_pixels: int = 12
    min_colored_fraction: float = 0.30
    min_dominance: float = 0.80
    min_confirmed_frames: int = 2
    min_vote_share: float = 0.75

    def __post_init__(self):
        for name in ('min_band_pixels', 'min_colored_pixels',
                     'min_confirmed_frames'):
            if getattr(self, name) < 1:
                raise ValueError(f'cube color {name} must be positive')
        if not (math.isfinite(self.min_colored_fraction)
                and 0 < self.min_colored_fraction <= 1):
            raise ValueError('cube color min_colored_fraction must be in (0, 1]')
        for name in ('min_dominance', 'min_vote_share'):
            value = getattr(self, name)
            if not math.isfinite(value) or not 0.5 < value <= 1:
                raise ValueError(f'cube color {name} must be in (0.5, 1]')


DEFAULT_CUBE_COLOR_CRITERIA = CubeColorCriteria()


def outer_edge_strips(corners):
    """Yield four bands starting at the detected edges of the 32 mm tag.

    Bands include the white label border and extend about 10 mm outward.
    Each edge uses its own image length to follow perspective skew.
    """
    corners = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    if not np.all(np.isfinite(corners)):
        return
    center = corners.mean(axis=0)
    for index in range(4):
        first = corners[index]
        second = corners[(index + 1) % 4]
        edge = second - first
        length = float(np.linalg.norm(edge))
        if length < 8.0:
            continue
        outward = np.array([-edge[1], edge[0]]) / length
        if np.dot(outward, center - (first + second) / 2.0) > 0:
            outward = -outward
        start = first + edge * 0.15
        end = second - edge * 0.15
        polygon = np.array([
            start,
            end,
            end + outward * length * 0.32,
            start + outward * length * 0.32,
        ])
        yield np.rint(polygon).astype(np.int32)


def classify_cube_color(bgr, corners, red_ranges, blue_range,
                        min_saturation, min_value,
                        criteria: CubeColorCriteria = DEFAULT_CUBE_COLOR_CRITERIA):
    """Return (color, confidence) using only small crops of the four bands."""
    red_total = blue_total = 0
    accepted_bands = 0
    image_height, image_width = bgr.shape[:2]
    for polygon in outer_edge_strips(corners):
        x, y, width, height = cv2.boundingRect(polygon)
        left, top = max(0, x), max(0, y)
        right = min(image_width, x + width)
        bottom = min(image_height, y + height)
        if right - left < 2 or bottom - top < 2:
            continue
        crop = bgr[top:bottom, left:right]
        mask = np.zeros(crop.shape[:2], np.uint8)
        cv2.fillConvexPoly(mask, polygon - (left, top), 255)
        pixels = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)[mask != 0]
        if len(pixels) < criteria.min_band_pixels:
            continue
        saturated = ((pixels[:, 1] >= min_saturation) &
                     (pixels[:, 2] >= min_value))
        hue = pixels[:, 0]
        red = sum(int(np.count_nonzero(
            saturated & (hue >= low) & (hue <= high)))
            for low, high in red_ranges)
        blue = int(np.count_nonzero(
            saturated & (hue >= blue_range[0]) &
            (hue <= blue_range[1])))
        colored = red + blue
        if (colored < criteria.min_colored_pixels or
                colored / len(pixels) < criteria.min_colored_fraction):
            continue
        if max(red, blue) / colored < criteria.min_dominance:
            return UNKNOWN, 0.0
        accepted_bands += 1
        red_total += red
        blue_total += blue
    colored_total = red_total + blue_total
    if not accepted_bands or not colored_total:
        return UNKNOWN, 0.0
    confidence = max(red_total, blue_total) / colored_total
    if confidence < criteria.min_dominance:
        return UNKNOWN, 0.0
    return (RED if red_total > blue_total else BLUE), float(confidence)


def confirmed_color(votes, criteria: CubeColorCriteria = DEFAULT_CUBE_COLOR_CRITERIA):
    """Aggregate independent frame decisions without changing pose selection."""
    if len(votes) < criteria.min_confirmed_frames:
        return UNKNOWN, 0.0, len(votes)
    scores = {RED: 0.0, BLUE: 0.0}
    counts = {RED: 0, BLUE: 0}
    for color, confidence in votes:
        scores[color] += confidence
        counts[color] += 1
    winner = RED if scores[RED] > scores[BLUE] else BLUE
    share = scores[winner] / max(sum(scores.values()), 1e-9)
    if (counts[winner] < criteria.min_confirmed_frames or
            share < criteria.min_vote_share):
        return UNKNOWN, 0.0, len(votes)
    mean_quality = scores[winner] / counts[winner]
    return winner, float(share * mean_quality), len(votes)
