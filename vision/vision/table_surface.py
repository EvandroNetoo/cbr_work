"""Classify white table cells in the known horizontal plane."""

from __future__ import annotations

import math
import cv2
import numpy as np
from interfaces.msg import TableSurfaceGrid
from geometry_msgs.msg import TransformStamped
from .geometry import rotation_from_quaternion


class TableSurfaceMixin:
    def evaluate_white_table_grid(
        self,
        session: Session,
        bgr: np.ndarray,
        camera_matrix: np.ndarray,
        camera_to_base: TransformStamped,
        render_debug: bool = True,
    ) -> tuple[list[bool], list[bool], np.ndarray | None]:
        """Classify a base-frame grid without assuming any tool geometry."""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        saturation = hsv[:, :, 1]
        value = hsv[:, :, 2]
        unknown = np.logical_or(
            value < self.white_surface_min_value,
            value > self.white_surface_max_value,
        )
        white = np.logical_and.reduce((
            ~unknown,
            saturation <= self.white_surface_max_saturation,
        ))
        debug = bgr.copy() if render_debug else None
        if debug is not None:
            debug[unknown] = (0, 180, 255)
            debug[white] = (
                0.35 * debug[white] + 0.65 * np.array([40, 210, 40])
            ).astype(np.uint8)

        transform = camera_to_base.transform
        rotation = rotation_from_quaternion(transform.rotation)
        origin = np.array([
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ], dtype=np.float64)
        height, width = bgr.shape[:2]
        cell_count = session.table_grid_width * session.table_grid_height
        observed = np.zeros(cell_count, dtype=bool)
        confirmed = np.zeros(cell_count, dtype=bool)
        if cell_count == 0:
            return observed.tolist(), confirmed.tolist(), debug

        x_indices = np.tile(
            np.arange(session.table_grid_width), session.table_grid_height)
        y_indices = np.repeat(
            np.arange(session.table_grid_height), session.table_grid_width)
        centers = np.column_stack((
            session.table_search_x_min_m
            + x_indices * session.table_grid_resolution_m,
            session.table_search_y_min_m
            + y_indices * session.table_grid_resolution_m,
        ))
        half_cell = session.table_grid_resolution_m / 2.0
        # Project the four metric corners of every cell. The plane is rectified
        # below at a density derived from their projected pixel size.
        offsets = np.array([
            [-half_cell, -half_cell],
            [half_cell, -half_cell],
            [half_cell, half_cell],
            [-half_cell, half_cell],
        ], dtype=np.float64)
        corner_xy = centers[:, None, :] + offsets[None, :, :]
        samples_base = np.concatenate((
            corner_xy,
            np.full((*corner_xy.shape[:2], 1), session.work_surface_height_m),
        ), axis=2)
        samples_camera = (samples_base - origin) @ rotation
        depths = samples_camera[:, :, 2]
        finite = np.all(np.isfinite(samples_camera), axis=2)
        in_front = np.logical_and(finite, depths > 1e-6)
        safe_depths = np.where(in_front, depths, 1.0)
        pixels_x = (
            camera_matrix[0, 0] * samples_camera[:, :, 0] / safe_depths
            + camera_matrix[0, 2])
        pixels_y = (
            camera_matrix[1, 1] * samples_camera[:, :, 1] / safe_depths
            + camera_matrix[1, 2])
        in_image = np.logical_and.reduce((
            in_front,
            pixels_x >= 0.0,
            pixels_x <= width - 1,
            pixels_y >= 0.0,
            pixels_y <= height - 1,
        ))
        fully_visible = np.all(in_image, axis=1)
        projected_corners = np.stack((pixels_x, pixels_y), axis=2)
        # A planar work surface is a homography. Rectify the whole requested
        # region once, then reduce all cells in NumPy. The former implementation
        # allocated and filled one small OpenCV mask per cell (often >1800
        # allocations/frame), which dominated runtime on the ARM computer.
        grid_width = session.table_grid_width
        grid_height = session.table_grid_height
        outer_source = np.array([
            projected_corners[0, 0],
            projected_corners[grid_width - 1, 1],
            projected_corners[-1, 2],
            projected_corners[(grid_height - 1) * grid_width, 3],
        ], dtype=np.float32)
        edge_lengths = np.linalg.norm(
            projected_corners
            - np.roll(projected_corners, -1, axis=1), axis=2)
        samples_per_cell = int(np.clip(
            math.ceil(float(np.nanmax(edge_lengths))), 4, 16))
        rectified_width = grid_width * samples_per_cell
        rectified_height = grid_height * samples_per_cell
        outer_target = np.array([
            [0.0, 0.0],
            [rectified_width - 1.0, 0.0],
            [rectified_width - 1.0, rectified_height - 1.0],
            [0.0, rectified_height - 1.0],
        ], dtype=np.float32)
        homography = cv2.getPerspectiveTransform(outer_source, outer_target)
        warped_unknown = cv2.warpPerspective(
            unknown.astype(np.uint8), homography,
            (rectified_width, rectified_height),
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=1,
        )
        warped_white = cv2.warpPerspective(
            white.astype(np.uint8), homography,
            (rectified_width, rectified_height),
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )
        reduction_shape = (
            grid_height, samples_per_cell,
            grid_width, samples_per_cell)
        unknown_fraction = warped_unknown.reshape(reduction_shape).mean(
            axis=(1, 3)).reshape(-1)
        white_fraction = warped_white.reshape(reduction_shape).mean(
            axis=(1, 3)).reshape(-1)
        observed = np.logical_and(
            fully_visible,
            unknown_fraction <= self.white_surface_max_unknown_fraction)
        confirmed = np.logical_and(
            observed, white_fraction >= self.white_surface_min_fraction)
        return observed.tolist(), confirmed.tolist(), debug
