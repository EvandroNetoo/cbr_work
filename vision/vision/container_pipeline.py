"""HSV container mask handling and ROS result conversion."""

from __future__ import annotations

import copy
import math
import cv2
import numpy as np
from geometry_msgs.msg import Pose
from interfaces.msg import ContainerDetection, ContainerDetectionArray, ContainerStampedDetection
from .constants import RED, BLUE


class ContainerPipelineMixin:
    def _configure_hsv_container_detector(self) -> None:
        """Validate color masks, known top height, and pixel-area thresholds."""
        def positive(name: str) -> float:
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be positive and finite')
            return value

        def bounded(name: str, maximum: int) -> int:
            value = int(self.get_parameter(name).value)
            if not 0 <= value <= maximum:
                raise ValueError(f'{name} must be in [0, {maximum}]')
            return value

        self.external_height = positive('external_height_m')
        self.min_saturation = bounded('min_saturation', 255)
        self.min_value = bounded('min_value', 255)
        self.red_ranges = (
            (bounded('red_hue_low_1', 179), bounded('red_hue_high_1', 179)),
            (bounded('red_hue_low_2', 179), bounded('red_hue_high_2', 179)),
        )
        self.blue_range = (
            bounded('blue_hue_low', 179), bounded('blue_hue_high', 179))
        if any(low > high for low, high in (*self.red_ranges, self.blue_range)):
            raise ValueError('HSV hue range low must not exceed high')
        kernel_size = int(self.get_parameter('morphology_kernel_px').value)
        if kernel_size <= 0 or kernel_size % 2 == 0:
            raise ValueError('morphology_kernel_px must be positive and odd')
        self.morphology_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        self.container_border_margin_px = int(
            self.get_parameter('container_border_margin_px').value)
        if self.container_border_margin_px < 0:
            raise ValueError('container_border_margin_px must be nonnegative')
        self.max_contour_fraction = float(
            self.get_parameter('max_contour_area_fraction').value)
        if not 0.0 < self.max_contour_fraction <= 1.0:
            raise ValueError('max_contour_area_fraction must be in (0, 1]')
        self.container_warmup = float(
            self.get_parameter('container_warmup_sec').value)
        if not math.isfinite(self.container_warmup) or self.container_warmup < 0:
            raise ValueError('container_warmup_sec must be finite and nonnegative')
        self.hsv_min_areas = tuple(int(self.get_parameter(name).value) for name in (
            'hsv_container_min_area_le_7_5cm_px',
            'hsv_container_min_area_le_12_5cm_px',
            'hsv_container_min_area_gt_12_5cm_px'))
        self.hsv_min_partial_areas = tuple(int(self.get_parameter(name).value) for name in (
            'hsv_container_min_partial_area_le_5cm_px',
            'hsv_container_min_partial_area_le_10cm_px',
            'hsv_container_min_partial_area_gt_10cm_px'))
        self.hsv_min_frames = int(self.get_parameter(
            'hsv_container_min_confirmed_frames').value)
        self.hsv_center_tolerance = positive(
            'hsv_container_center_tolerance_px')
        if min(*self.hsv_min_areas, *self.hsv_min_partial_areas,
               self.hsv_min_frames) <= 0:
            raise ValueError('HSV area and frame limits must be positive')

    def container_color_masks(self, bgr: np.ndarray) -> dict[int, np.ndarray]:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        red = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for low, high in self.red_ranges:
            red = cv2.bitwise_or(red, cv2.inRange(
                hsv, (low, self.min_saturation, self.min_value),
                (high, 255, 255)))
        blue = cv2.inRange(
            hsv,
            (self.blue_range[0], self.min_saturation, self.min_value),
            (self.blue_range[1], 255, 255))
        output = {}
        for color, mask in ((RED, red), (BLUE, blue)):
            cleaned = cv2.morphologyEx(
                mask, cv2.MORPH_OPEN, self.morphology_kernel)
            cleaned = cv2.morphologyEx(
                cleaned, cv2.MORPH_CLOSE, self.morphology_kernel)
            output[color] = cleaned
        return output

    def hsv_blob_to_stamped(self, blob, header, pose, frame):
        item = ContainerStampedDetection()
        item.header = copy.deepcopy(header)
        item.header.frame_id = frame
        item.color = blob.color
        item.mask_area_px = float(blob.area)
        item.observation_count = 1
        item.partial = blob.partial
        item.pose = pose
        return item

    @staticmethod
    def copy_container_stamped(
        item: ContainerStampedDetection,
        pose: Pose | None = None,
        frame: str | None = None,
    ) -> ContainerStampedDetection:
        result = ContainerStampedDetection()
        result.header = copy.deepcopy(item.header)
        if frame is not None:
            result.header.frame_id = frame
        result.color = item.color
        result.mask_area_px = item.mask_area_px
        result.observation_count = item.observation_count
        result.position_spread_m = item.position_spread_m
        result.partial = item.partial
        result.pose = item.pose if pose is None else pose
        return result

    @staticmethod
    def container_detection_array(frame, image, items):
        output = ContainerDetectionArray()
        output.header.frame_id = frame
        output.header.stamp = image.header.stamp
        for item in items:
            detection = ContainerDetection()
            detection.color = item.color
            detection.mask_area_px = item.mask_area_px
            detection.observation_count = item.observation_count
            detection.position_spread_m = item.position_spread_m
            detection.partial = item.partial
            detection.pose = item.pose
            output.detections.append(detection)
        return output

    @staticmethod
    def _container_position(item: ContainerStampedDetection) -> np.ndarray:
        return np.array([
            item.pose.position.x,
            item.pose.position.y,
            item.pose.position.z,
        ], dtype=np.float64)

    def _confirmed_hsv_container_results(self, tracks):
        camera_results, base_results = [], []
        for track in tracks:
            observations = track.observations
            complete = [item for item in observations if not item.blob.partial]
            selected = complete if len(complete) >= self.hsv_min_frames else observations
            if len(selected) < self.hsv_min_frames:
                continue
            centers = np.array([item.blob.center for item in selected])
            center = np.median(centers, axis=0)
            selected = [item for item, distance in zip(
                selected, np.linalg.norm(centers - center, axis=1))
                if distance <= self.hsv_center_tolerance]
            if len(selected) < self.hsv_min_frames:
                continue
            for field_name, output in (('camera', camera_results),
                                       ('base', base_results)):
                representative = max(selected, key=lambda item: item.blob.area)
                result = self.copy_container_stamped(
                    getattr(representative, field_name))
                positions = np.array([
                    self._container_position(getattr(item, field_name))
                    for item in selected])
                position = np.median(positions, axis=0)
                (result.pose.position.x, result.pose.position.y,
                 result.pose.position.z) = map(float, position)
                result.observation_count = len(selected)
                result.mask_area_px = float(np.median([
                    item.blob.area for item in selected]))
                result.position_spread_m = float(max(
                    np.linalg.norm(positions - position, axis=1), default=0.0))
                result.partial = any(item.blob.partial for item in selected)
                output.append(result)
        return camera_results, base_results
