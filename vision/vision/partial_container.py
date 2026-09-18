"""Fit a known tabletop rectangle when its image silhouette is clipped.

The fit is performed in the arm base frame on the known top plane. A projected
rectangle is clipped to the image before comparison with the observed contour;
using the cropped contour's bounding-box center would systematically move the
estimated container toward the visible part.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import cv2

import numpy as np


@dataclass(frozen=True)
class PartialContainerFit:
    """Result and uncertainty of one clipped-silhouette fit."""

    x: float
    y: float
    yaw: float
    overlap: float
    edge_error_px: float
    position_uncertainty_m: float
    yaw_uncertainty_deg: float


def rotation_from_quaternion(quaternion) -> np.ndarray:
    """Return a normalized 3x3 rotation matrix."""
    values = np.array([
        quaternion.x, quaternion.y, quaternion.z, quaternion.w,
    ], dtype=np.float64)
    norm = float(np.linalg.norm(values))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('invalid camera transform rotation')
    x, y, z, w = values / norm
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def _intersect_area(left: np.ndarray, right: np.ndarray) -> float:
    area, _ = cv2.intersectConvexConvex(
        left.astype(np.float32), right.astype(np.float32))
    return max(0.0, float(area))


def fit_partial_container(
    contour: np.ndarray,
    image_shape: tuple[int, int],
    camera_matrix: np.ndarray,
    camera_to_base,
    top_height_m: float,
    depth_m: float,
    width_m: float,
) -> PartialContainerFit | None:
    """Fit XY/yaw using the visible silhouette and the known external size."""
    if not all(math.isfinite(value) for value in (
        top_height_m, depth_m, width_m,
    )) or min(depth_m, width_m) <= 0.0:
        return None
    height, width = image_shape
    if height < 4 or width < 4:
        return None
    transform = camera_to_base.transform
    rotation = rotation_from_quaternion(transform.rotation)
    origin = np.array([
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    ], dtype=np.float64)
    fx, fy = float(camera_matrix[0, 0]), float(camera_matrix[1, 1])
    cx, cy = float(camera_matrix[0, 2]), float(camera_matrix[1, 2])
    if fx <= 0.0 or fy <= 0.0:
        return None

    visible = cv2.convexHull(contour.astype(np.float32)).reshape(-1, 2)
    visible_area = float(cv2.contourArea(visible))
    if visible_area <= 0.0:
        return None
    # Project sampled silhouette points onto the known top plane.
    pixels = visible
    rays_camera = np.column_stack((
        (pixels[:, 0] - cx) / fx,
        (pixels[:, 1] - cy) / fy,
        np.ones(len(pixels)),
    ))
    rays_base = rays_camera @ rotation.T
    vertical = rays_base[:, 2]
    if np.any(np.abs(vertical) < 1e-6):
        return None
    distances = (top_height_m - origin[2]) / vertical
    if np.any(distances <= 0.0) or not np.all(np.isfinite(distances)):
        return None
    world = origin + distances[:, None] * rays_base
    xy = world[:, :2]
    if not np.all(np.isfinite(xy)):
        return None

    world_box = cv2.boxPoints(cv2.minAreaRect(xy.astype(np.float32)))
    edge = world_box[1] - world_box[0]
    seed = math.atan2(float(edge[1]), float(edge[0]))
    # Both assignments of the visible rectangle's axes are possible after a
    # crop. Search nearby yaws and let the clipped silhouette select the fit.
    yaws = {
        round((seed + axis + math.radians(delta)) % math.pi, 8)
        for axis in (0.0, math.pi / 2.0)
        for delta in range(-25, 26, 5)
    }
    viewport = np.array([
        [0., 0.], [width - 1., 0.],
        [width - 1., height - 1.], [0., height - 1.],
    ], dtype=np.float32)
    scored: list[tuple[float, float, float, float, np.ndarray]] = []
    for yaw in sorted(yaws):
        direction = np.array([math.cos(yaw), math.sin(yaw)])
        side = np.array([-direction[1], direction[0]])
        along, across = xy @ direction, xy @ side
        ranges = []
        for values, extent in ((along, depth_m), (across, width_m)):
            low = float(np.max(values) - extent / 2.0)
            high = float(np.min(values) + extent / 2.0)
            if low > high:
                midpoint = (low + high) / 2.0
                low = high = midpoint
            ranges.append(np.linspace(low, high, 5))
        for along_center in ranges[0]:
            for across_center in ranges[1]:
                center = along_center * direction + across_center * side
                corners_xy = np.array([
                    center - depth_m/2*direction - width_m/2*side,
                    center + depth_m/2*direction - width_m/2*side,
                    center + depth_m/2*direction + width_m/2*side,
                    center - depth_m/2*direction + width_m/2*side,
                ])
                corners_base = np.column_stack((
                    corners_xy, np.full(4, top_height_m)))
                corners_camera = (corners_base - origin) @ rotation
                if np.any(corners_camera[:, 2] <= 1e-6):
                    continue
                projected = np.column_stack((
                    fx * corners_camera[:, 0] / corners_camera[:, 2] + cx,
                    fy * corners_camera[:, 1] / corners_camera[:, 2] + cy,
                )).astype(np.float32)
                projected_area, clipped = cv2.intersectConvexConvex(
                    projected, viewport)
                if projected_area <= 0.0 or clipped is None:
                    continue
                intersection = min(
                    _intersect_area(clipped.reshape(-1, 2), visible),
                    float(projected_area), visible_area,
                )
                union = projected_area + visible_area - intersection
                overlap = min(1.0, max(
                    0.0, intersection / union if union > 0.0 else 0.0))
                scored.append((overlap, float(center[0]), float(center[1]),
                               yaw, projected))
    if not scored:
        return None
    best = max(scored, key=lambda value: value[0])
    # Low-overlap fits remain useful as conservative table obstacles. The
    # consumer applies the configured threshold before using one as a target.
    # Ambiguous fits are reflected in the clearance used by table placement.
    near = [item for item in scored if item[0] >= best[0] - 0.04]
    full_area = float(cv2.contourArea(best[4]))
    visible_fraction = (
        min(1.0, visible_area / full_area) if full_area > 0 else 0.0)
    position_uncertainty = max(
        (math.hypot(item[1] - best[1], item[2] - best[2]) for item in near),
        default=0.0,
    ) + 0.005 + 0.03 * (1.0 - visible_fraction) + 0.10 * (
        1.0 - min(best[0], 1.0))
    yaw_uncertainty = max(
        (abs((item[3] - best[3] + math.pi / 2) % math.pi - math.pi / 2)
         for item in near),
        default=0.0,
    ) + math.radians(10.0 * (1.0 - visible_fraction))
    # Report pixel residual on visible contour edges, excluding the image edge
    # itself (which is a crop boundary rather than a physical container edge).
    projected_contour = best[4].reshape(-1, 1, 2)
    edge_distances = [
        cv2.pointPolygonTest(projected_contour, tuple(map(float, point)), True)
        for point in contour.reshape(-1, 2)
        if 2 < point[0] < width - 3 and 2 < point[1] < height - 3
    ]
    edge_error = math.sqrt(sum(d*d for d in edge_distances) /
                           len(edge_distances)) if edge_distances else 0.0
    return PartialContainerFit(
        x=best[1], y=best[2], yaw=best[3], overlap=best[0],
        edge_error_px=edge_error,
        position_uncertainty_m=position_uncertainty,
        yaw_uncertainty_deg=math.degrees(yaw_uncertainty),
    )
