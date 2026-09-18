"""Projection and planar geometry used by the container detector."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


@dataclass(frozen=True)
class ContainerGeometry:
    name: str
    physical_object: str
    depth: float
    width: float
    height: float
    inner_depth: float
    inner_width: float
    inner_height: float


@dataclass(frozen=True)
class Projection:
    outer: np.ndarray
    inner: np.ndarray
    bottom: np.ndarray
    visible_fraction: float
    expected_segmentable_area: float
    rim_mask: np.ndarray
    silhouette_mask: np.ndarray
    inner_mask: np.ndarray


def hsv_color_masks(
    bgr: np.ndarray,
    *,
    min_saturation: int = 80,
    min_value: int = 45,
    red_ranges: tuple[tuple[int, int], tuple[int, int]] = ((0, 12), (150, 179)),
    blue_range: tuple[int, int] = (70, 138),
    open_size: int = 3,
    close_size: int = 5,
) -> tuple[np.ndarray, np.ndarray]:
    """Return separate red/blue masks using the native OpenCV HSV scale."""
    hsv = cv2.cvtColor(np.asarray(bgr, dtype=np.uint8), cv2.COLOR_BGR2HSV)
    red = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for low, high in red_ranges:
        red = cv2.bitwise_or(red, cv2.inRange(
            hsv, (low, min_saturation, min_value), (high, 255, 255)))
    blue = cv2.inRange(
        hsv, (blue_range[0], min_saturation, min_value),
        (blue_range[1], 255, 255))
    def clean(mask: np.ndarray) -> np.ndarray:
        opening = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (open_size | 1, open_size | 1))
        closing = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (close_size | 1, close_size | 1))
        return cv2.morphologyEx(
            cv2.morphologyEx(mask, cv2.MORPH_OPEN, opening),
            cv2.MORPH_CLOSE, closing)
    return clean(red), clean(blue)


def rectangular_model_error(
    measured_dimensions: tuple[float, float], expected_dimensions: tuple[float, float],
) -> float:
    """Scale-free fit error for an axial rectangular physical hypothesis."""
    measured = sorted((float(value) for value in measured_dimensions), reverse=True)
    expected = sorted((float(value) for value in expected_dimensions), reverse=True)
    return max(abs(measured[index] - expected[index]) / expected[index]
               for index in range(2))


def opening_evidence_score(
    inner_area_px: float,
    colored_inner_area_px: float,
    cube_size_m: float,
    opening_depth_m: float,
    opening_width_m: float,
    maximum_cubes: int,
) -> float:
    """Discount at most the known projected area of cubes in the opening."""
    inner_area_px = max(1.0, float(inner_area_px))
    cube_projection = (
        inner_area_px * cube_size_m**2 / (opening_depth_m * opening_width_m))
    unexplained = max(
        0.0, float(colored_inner_area_px) - maximum_cubes*cube_projection)
    return max(0.0, min(1.0, 1.0 - unexplained/inner_area_px))


def load_geometry_profile(path: str, profile_name: str) -> ContainerGeometry:
    source = Path(path)
    root = yaml.safe_load(source.read_text(encoding='utf-8'))
    if root.get('schema_version') != 1 or not isinstance(root.get('profiles'), dict):
        raise ValueError(f'Invalid container geometry file: {source}')
    try:
        raw = root['profiles'][profile_name]
    except KeyError as error:
        raise ValueError(f'Unknown container geometry profile: {profile_name}') from error
    external = tuple(float(value) for value in raw['external_dimensions_m'])
    internal = tuple(float(value) for value in raw['internal_dimensions_m'])
    if len(external) != 3 or len(internal) != 3:
        raise ValueError('Container dimensions must contain depth, width and height')
    if not all(math.isfinite(value) and value > 0.0 for value in external + internal):
        raise ValueError('Container dimensions must be positive and finite')
    if internal[0] >= external[0] or internal[1] >= external[1] or internal[2] >= external[2]:
        raise ValueError('Internal dimensions must be smaller than external dimensions')
    return ContainerGeometry(
        profile_name, str(raw.get('physical_object', profile_name)),
        *external, *internal,
    )


def rotation_from_quaternion(quaternion) -> np.ndarray:
    values = np.array([
        quaternion.x, quaternion.y, quaternion.z, quaternion.w,
    ], dtype=np.float64)
    norm = float(np.linalg.norm(values))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid transform quaternion')
    x, y, z, w = values / norm
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def transform_parts(camera_to_base) -> tuple[np.ndarray, np.ndarray]:
    transform = camera_to_base.transform
    rotation = rotation_from_quaternion(transform.rotation)
    translation = np.array([
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    ], dtype=np.float64)
    if not np.all(np.isfinite(translation)):
        raise ValueError('Invalid transform translation')
    return rotation, translation


def pixels_to_plane(
    pixels: np.ndarray,
    camera_matrix: np.ndarray,
    camera_to_base,
    plane_z: float,
) -> np.ndarray | None:
    rotation, origin = transform_parts(camera_to_base)
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    pixels = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)
    rays_camera = np.column_stack((
        (pixels[:, 0] - cx) / fx,
        (pixels[:, 1] - cy) / fy,
        np.ones(len(pixels)),
    ))
    rays_base = rays_camera @ rotation.T
    denominators = rays_base[:, 2]
    if np.any(np.abs(denominators) < 1e-8):
        return None
    distances = (float(plane_z) - origin[2]) / denominators
    if np.any(distances <= 0.0) or not np.all(np.isfinite(distances)):
        return None
    return origin + distances[:, None] * rays_base


def rectangle_xy(center: tuple[float, float], depth: float, width: float, yaw: float):
    direction = np.array([math.cos(yaw), math.sin(yaw)], dtype=np.float64)
    side = np.array([-direction[1], direction[0]], dtype=np.float64)
    center_xy = np.asarray(center, dtype=np.float64)
    return np.array([
        center_xy - depth/2*direction - width/2*side,
        center_xy + depth/2*direction - width/2*side,
        center_xy + depth/2*direction + width/2*side,
        center_xy - depth/2*direction + width/2*side,
    ])


def project_base_points(
    points_base: np.ndarray,
    camera_matrix: np.ndarray,
    camera_to_base,
) -> np.ndarray | None:
    rotation, origin = transform_parts(camera_to_base)
    points_camera = (np.asarray(points_base, dtype=np.float64) - origin) @ rotation
    if np.any(points_camera[:, 2] <= 1e-6):
        return None
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    return np.column_stack((
        fx * points_camera[:, 0] / points_camera[:, 2] + cx,
        fy * points_camera[:, 1] / points_camera[:, 2] + cy,
    )).astype(np.float32)


def _polygon_mask(points: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
    output = np.zeros(shape, dtype=np.uint8)
    if points is not None and len(points) >= 3:
        cv2.fillConvexPoly(output, np.rint(points).astype(np.int32), 255)
    return output


def project_open_container(
    center: tuple[float, float],
    yaw: float,
    table_height: float,
    geometry: ContainerGeometry,
    camera_matrix: np.ndarray,
    camera_to_base,
    image_shape: tuple[int, int],
    rim_segmentable_fraction: float,
    side_segmentable_fraction: float,
) -> Projection | None:
    top_z = table_height + geometry.height
    outer_xy = rectangle_xy(center, geometry.depth, geometry.width, yaw)
    inner_xy = rectangle_xy(center, geometry.inner_depth, geometry.inner_width, yaw)
    outer = project_base_points(
        np.column_stack((outer_xy, np.full(4, top_z))),
        camera_matrix, camera_to_base)
    inner = project_base_points(
        np.column_stack((inner_xy, np.full(4, top_z))),
        camera_matrix, camera_to_base)
    bottom = project_base_points(
        np.column_stack((outer_xy, np.full(4, table_height))),
        camera_matrix, camera_to_base)
    if outer is None or inner is None or bottom is None:
        return None
    outer_mask = _polygon_mask(outer, image_shape)
    inner_mask = _polygon_mask(inner, image_shape)
    rim_mask = cv2.subtract(outer_mask, inner_mask)
    side_mask = np.zeros(image_shape, dtype=np.uint8)
    _, camera_origin = transform_parts(camera_to_base)
    for index in range(4):
        next_index = (index + 1) % 4
        edge = outer_xy[next_index] - outer_xy[index]
        outward = np.array([edge[1], -edge[0]])
        midpoint = (outer_xy[index] + outer_xy[next_index]) / 2.0
        if float(np.dot(outward, camera_origin[:2] - midpoint)) <= 0.0:
            continue  # Back faces cannot contribute visible colored plastic.
        face = np.array([
            outer[index], outer[next_index], bottom[next_index], bottom[index],
        ], dtype=np.float32)
        cv2.fillConvexPoly(side_mask, np.rint(face).astype(np.int32), 255)
    silhouette = cv2.bitwise_or(outer_mask, side_mask)
    full_outer_area = max(float(cv2.contourArea(outer)), 1.0)
    visible_outer_area = float(np.count_nonzero(outer_mask))
    visible_fraction = min(1.0, visible_outer_area / full_outer_area)
    expected = (
        float(np.count_nonzero(rim_mask)) * rim_segmentable_fraction
        + float(np.count_nonzero(side_mask & cv2.bitwise_not(outer_mask)))
        * side_segmentable_fraction
    )
    return Projection(
        outer, inner, bottom, visible_fraction, max(expected, 1.0),
        rim_mask, silhouette, inner_mask,
    )


def axial_angle_distance(left: float, right: float) -> float:
    return abs((left - right + math.pi / 2.0) % math.pi - math.pi / 2.0)


def yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def polygon_overlap_sat(left: np.ndarray, right: np.ndarray) -> bool:
    """Separating-axis test for convex 2-D polygons."""
    for polygon in (left, right):
        edges = np.roll(polygon, -1, axis=0) - polygon
        for edge in edges:
            axis = np.array([-edge[1], edge[0]], dtype=np.float64)
            norm = np.linalg.norm(axis)
            if norm < 1e-12:
                continue
            axis /= norm
            left_projection = left @ axis
            right_projection = right @ axis
            if left_projection.max() <= right_projection.min() + 1e-9:
                return False
            if right_projection.max() <= left_projection.min() + 1e-9:
                return False
    return True
