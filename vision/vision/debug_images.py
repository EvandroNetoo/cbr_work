"""Render AprilTag, table, and HSV container diagnostics."""

from __future__ import annotations

import copy
from dataclasses import dataclass
import cv2
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from interfaces.msg import AprilTagStampedDetection, TableSurfaceGrid
from sensor_msgs.msg import Image
from .constants import APRILTAGS, CONTAINERS_HSV, TABLE_SURFACE, COLOR_NAMES, DEBUG_COLORS
from .geometry import rotation_from_quaternion


def overlay_container_masks(image: np.ndarray,
                            masks: dict[int, np.ndarray],
                            alpha: float = 0.42) -> np.ndarray:
    """Tint exact HSV mask pixels and outline their connected regions."""
    debug = image.copy()
    for color, mask in masks.items():
        if mask.shape != image.shape[:2]:
            raise ValueError('container mask size differs from image size')
        selected = mask != 0
        if np.any(selected):
            tint = np.asarray(DEBUG_COLORS[color], dtype=np.float32)
            debug[selected] = np.rint(
                image[selected] * (1.0 - alpha) + tint * alpha
            ).astype(np.uint8)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(debug, contours, -1, (255, 255, 255), 1)
    return debug


@dataclass
class ContainerDebugFrame:
    """Annotated observation plus the calibration valid for that frame."""

    header: object
    image: np.ndarray
    camera_matrix: np.ndarray
    camera_to_base: TransformStamped | None
    container_masks: dict[int, np.ndarray] | None = None


class DebugImagesMixin:
    def publish_table_surface_debug_image(
        self, message: Image, debug: np.ndarray,
    ) -> None:
        output = self._bgr_image_message(message.header, debug)
        self.table_surface_debug_image_publisher.publish(output)

    def publish_hsv_container_debug_image(
            self, source, bgr, masks, blobs, session):
        debug = overlay_container_masks(bgr, masks)
        for blob in blobs:
            center = tuple(round(value) for value in blob.center)
            color = DEBUG_COLORS[blob.color]
            cv2.circle(debug, center, 6, color, 2)
            cv2.putText(debug,
                        f'{COLOR_NAMES[blob.color]} {blob.area}px' +
                        (' partial' if blob.partial else ''),
                        (center[0] + 8, center[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
        summary = self._detector_summary(session, CONTAINERS_HSV, 'LIVE')
        cv2.rectangle(debug, (0, 0), (debug.shape[1] - 1, 25),
                      (0, 0, 0), -1)
        self._draw_debug_text(
            debug, f'{summary} B{len(blobs)} MASK red/blue', (5, 17),
            scale=0.34)
        self.latest_container_debug_frame = ContainerDebugFrame(
            header=copy.deepcopy(source.header), image=debug.copy(),
            camera_matrix=np.array(session.latest_debug_frame.camera_matrix),
            camera_to_base=copy.deepcopy(
                session.latest_debug_frame.camera_to_base))
        self.container_debug_image_publisher.publish(
            self._bgr_image_message(source.header, debug))

    @staticmethod
    def _bgr_image_message(header, bgr: np.ndarray) -> Image:
        output = Image()
        output.header = header
        output.height, output.width = bgr.shape[:2]
        output.encoding = 'bgr8'
        output.is_bigendian = False
        output.step = output.width * 3
        output.data = bgr.tobytes()
        return output

    @staticmethod
    def _draw_debug_text(
        image: np.ndarray,
        text: str,
        origin: tuple[int, int],
        color: tuple[int, int, int] = (255, 255, 255),
        scale: float = 0.34,
    ) -> None:
        cv2.putText(
            image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
            (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(
            image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
            color, 1, cv2.LINE_AA)

    @staticmethod
    def _project_base_points(
        points_local: np.ndarray,
        pose: Pose,
        frame: ContainerDebugFrame,
    ) -> np.ndarray | None:
        if frame.camera_to_base is None:
            return None
        try:
            object_rotation = rotation_from_quaternion(pose.orientation)
            camera_rotation = rotation_from_quaternion(
                frame.camera_to_base.transform.rotation)
        except ValueError:
            return None
        position = np.array([
            pose.position.x, pose.position.y, pose.position.z,
        ], dtype=np.float64)
        origin = np.array([
            frame.camera_to_base.transform.translation.x,
            frame.camera_to_base.transform.translation.y,
            frame.camera_to_base.transform.translation.z,
        ], dtype=np.float64)
        points_base = points_local @ object_rotation.T + position
        points_camera = (points_base - origin) @ camera_rotation
        depths = points_camera[:, 2]
        if (
            not np.all(np.isfinite(points_camera))
            or np.any(depths <= 1e-6)
        ):
            return None
        matrix = frame.camera_matrix
        pixels = np.column_stack((
            matrix[0, 0] * points_camera[:, 0] / depths + matrix[0, 2],
            matrix[1, 1] * points_camera[:, 1] / depths + matrix[1, 2],
        ))
        return np.rint(pixels).astype(np.int32)

    def _draw_final_apriltags(
        self,
        debug: np.ndarray,
        frame: ContainerDebugFrame,
        detections: list[AprilTagStampedDetection],
    ) -> None:
        half = self.tag_size_m / 2.0
        tag_corners = np.array([
            [-half, -half, 0.0], [half, -half, 0.0],
            [half, half, 0.0], [-half, half, 0.0],
        ])
        axis_length = self.tag_size_m * 0.75
        axes = np.array([
            [0.0, 0.0, 0.0], [axis_length, 0.0, 0.0],
            [0.0, axis_length, 0.0], [0.0, 0.0, axis_length],
        ])
        line_y = 42
        for item in sorted(detections, key=lambda detection: detection.id):
            corners = self._project_base_points(tag_corners, item.pose, frame)
            projected_axes = self._project_base_points(axes, item.pose, frame)
            if corners is not None:
                cv2.polylines(
                    debug, [corners.reshape(-1, 1, 2)], True,
                    (0, 220, 0), 2, cv2.LINE_AA)
                center = tuple(np.rint(corners.mean(axis=0)).astype(int))
                self._draw_debug_text(
                    debug, f'T{item.id}', center, (0, 255, 0), 0.48)
            if projected_axes is not None:
                center = tuple(projected_axes[0])
                for endpoint, color in zip(
                    projected_axes[1:],
                    ((0, 0, 255), (0, 255, 0), (255, 0, 0)),
                ):
                    cv2.line(
                        debug, center, tuple(endpoint), color,
                        2, cv2.LINE_AA)
            position = item.pose.position
            self._draw_debug_text(
                debug,
                f'T{item.id} x={position.x:.3f} y={position.y:.3f} '
                f'z={position.z:.3f}m err={item.pose_error:.2f}px',
                (5, line_y),
            )
            line_y += 14
            self._draw_debug_text(
                debug,
                f'  margin={item.decision_margin:.1f} h={item.hamming}',
                (5, line_y),
            )
            line_y += 17

    def publish_final_debug_images(
        self, session: Session, result, status: str,
    ) -> None:
        """Publish retained summaries made from the exact action result."""
        if session.requested_detectors & APRILTAGS:
            frame = session.latest_debug_frames.get(
                APRILTAGS, session.latest_debug_frame)
            if frame is None:
                return
            debug = frame.image.copy()
            detections = list(result.best_apriltags_base)
            summary = self._detector_summary(session, APRILTAGS, status)
            cv2.rectangle(
                debug, (0, 0), (debug.shape[1] - 1, 25), (0, 0, 0), -1)
            self._draw_debug_text(
                debug, f'{summary} A{len(detections)}', (5, 17),
                scale=0.34)
            self._draw_final_apriltags(debug, frame, detections)
            self.debug_image_publisher.publish(
                self._bgr_image_message(frame.header, debug))
        if session.requested_detectors & CONTAINERS_HSV:
            frame = session.latest_debug_frames.get(
                CONTAINERS_HSV, session.latest_debug_frame)
            if frame is None:
                return
            debug = overlay_container_masks(
                frame.image, frame.container_masks or {})
            detections = list(result.best_containers_base)
            summary = self._detector_summary(session, CONTAINERS_HSV, status)
            cv2.rectangle(
                debug, (0, 0), (debug.shape[1] - 1, 25), (0, 0, 0), -1)
            self._draw_debug_text(
                debug, f'{summary} A{len(detections)} MASK red/blue',
                (5, 17), scale=0.34)
            for track in session.hsv_container_tracks:
                if len(track.observations) < self.hsv_min_frames:
                    continue
                center = tuple(round(float(value)) for value in track.center)
                cv2.drawMarker(debug, center, DEBUG_COLORS[track.color],
                               cv2.MARKER_CROSS, 18, 2)
                self._draw_debug_text(
                    debug,
                    f'{COLOR_NAMES[track.color]} n={len(track.observations)}',
                    (center[0] + 8, center[1] - 8), scale=0.38)
            self.latest_container_debug_frame = ContainerDebugFrame(
                header=copy.deepcopy(frame.header),
                image=debug.copy(),
                camera_matrix=frame.camera_matrix.copy(),
                camera_to_base=copy.deepcopy(frame.camera_to_base),
                container_masks={
                    color: mask.copy()
                    for color, mask in (frame.container_masks or {}).items()},
            )
            self.container_debug_image_publisher.publish(
                self._bgr_image_message(frame.header, debug))
        if session.requested_detectors & TABLE_SURFACE:
            frame = session.latest_debug_frames.get(
                TABLE_SURFACE, session.latest_debug_frame)
            if frame is None:
                return
            debug = frame.image.copy()
            if frame.camera_to_base is not None:
                _observed, _confirmed, rendered = self.evaluate_white_table_grid(
                    session, frame.image, frame.camera_matrix,
                    frame.camera_to_base, render_debug=True)
                if rendered is not None:
                    debug = rendered
            cells = list(result.table_surface_grid.cells)
            free = sum(cell == TableSurfaceGrid.FREE for cell in cells)
            summary = self._detector_summary(session, TABLE_SURFACE, status)
            cv2.rectangle(
                debug, (0, 0), (debug.shape[1] - 1, 25), (0, 0, 0), -1)
            self._draw_debug_text(
                debug, f'{summary} FREE{free}/{len(cells)}', (5, 17),
                scale=0.34)
            self.table_surface_debug_image_publisher.publish(
                self._bgr_image_message(frame.header, debug))

    def container_target_callback(self, target: PoseStamped) -> None:
        """Project the exact MoveIt TCP target over the cached camera frame."""
        cached = self.latest_container_debug_frame
        if cached is None or cached.camera_to_base is None:
            self.get_logger().warning(
                'Alvo do contêiner recebido sem frame/TF de visão armazenado.',
                throttle_duration_sec=2.0)
            return
        if target.header.frame_id != self.base_frame:
            self.get_logger().warning(
                'Alvo do contêiner fora do referencial base: '
                f'{target.header.frame_id!r}.',
                throttle_duration_sec=2.0)
            return

        transform = cached.camera_to_base.transform
        rotation = rotation_from_quaternion(transform.rotation)
        origin = np.array([
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ], dtype=np.float64)
        point_base = np.array([
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z,
        ], dtype=np.float64)
        if not np.all(np.isfinite(point_base)):
            self.get_logger().warning('Alvo do contêiner contém posição inválida.')
            return
        point_camera = rotation.T @ (point_base - origin)
        if not np.all(np.isfinite(point_camera)) or point_camera[2] <= 1e-6:
            self.get_logger().warning(
                'Alvo do contêiner está atrás da câmera no frame armazenado.')
            return

        matrix = cached.camera_matrix
        pixel = np.array([
            matrix[0, 0] * point_camera[0] / point_camera[2] + matrix[0, 2],
            matrix[1, 1] * point_camera[1] / point_camera[2] + matrix[1, 2],
        ])
        if not np.all(np.isfinite(pixel)):
            return
        x, y = map(int, np.rint(pixel))
        height, width = cached.image.shape[:2]
        if not (0 <= x < width and 0 <= y < height):
            self.get_logger().warning(
                f'Alvo MoveIt projetado fora da imagem: ({x}, {y}).')
            return

        debug = cached.image.copy()
        color = (255, 0, 255)
        cv2.circle(debug, (x, y), 4, color, -1, cv2.LINE_AA)
        cv2.drawMarker(
            debug, (x, y), color, cv2.MARKER_DIAMOND,
            20, 3, cv2.LINE_AA)
        label_origin = (min(x + 8, max(0, width - 112)), max(16, y - 8))
        cv2.putText(
            debug, 'MoveIt TCP target', label_origin,
            cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1, cv2.LINE_AA)
        self.container_debug_image_publisher.publish(
            self._bgr_image_message(cached.header, debug))

    def publish_detection_debug_image(
            self, source: Image, mono: np.ndarray, detections,
            session: Session, fps: float) -> None:
        """Publish the detector input annotated with raw AprilTag candidates."""
        debug = cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
        accepted = 0
        for detection in detections:
            is_accepted = (
                detection.hamming <= self.max_hamming
                and detection.decision_margin >= self.min_decision_margin
            )
            accepted += int(is_accepted)
            color = (0, 200, 0) if is_accepted else (0, 0, 255)
            corners = np.rint(np.asarray(detection.corners)).astype(np.int32)
            cv2.polylines(debug, [corners.reshape(-1, 1, 2)], True,
                          color, 2, cv2.LINE_AA)
            center = tuple(map(
                int, np.rint(np.asarray(detection.center)).reshape(2)))
            cv2.circle(debug, center, 3, color, -1, cv2.LINE_AA)
            label = (
                f'id={int(detection.tag_id)} '
                f'm={float(detection.decision_margin):.1f} '
                f'h={int(detection.hamming)}'
            )
            text_origin = (max(0, int(corners[:, 0].min())),
                           max(16, int(corners[:, 1].min()) - 5))
            cv2.putText(debug, label, text_origin, cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, color, 1, cv2.LINE_AA)

        detector_times = session.recent_detector_times.get(APRILTAGS, [])
        fps_text = f'{fps:.1f}' if len(detector_times) > 1 else '--'
        frames = session.detector_frame_counts.get(APRILTAGS, 0)
        transforms = session.detector_frames_with_base_transform.get(
            APRILTAGS, 0)
        summary = (
            f'LIVE {fps_text}fps F{frames} '
            f'TF{transforms}/{frames} R{len(detections)} A{accepted}')
        cv2.rectangle(debug, (0, 0), (debug.shape[1] - 1, 24),
                      (0, 0, 0), -1)
        cv2.putText(debug, summary, (6, 17), cv2.FONT_HERSHEY_SIMPLEX,
                    0.34, (255, 255, 255), 1, cv2.LINE_AA)

        output = Image()
        output.header = source.header
        output.height, output.width = debug.shape[:2]
        output.encoding = 'bgr8'
        output.is_bigendian = False
        output.step = output.width * 3
        output.data = debug.tobytes()
        self.debug_image_publisher.publish(output)
