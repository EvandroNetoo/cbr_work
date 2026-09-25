"""Validate and decode ROS image encodings."""

from __future__ import annotations

import cv2
import numpy as np
from sensor_msgs.msg import Image


class ImageEncodingMixin:
    @staticmethod
    def image_to_bgr8(message: Image) -> np.ndarray:
        height = int(message.height)
        width = int(message.width)
        step = int(message.step)
        if height <= 0 or width <= 0 or step <= 0:
            raise ValueError('image dimensions and step must be positive')
        buffer = np.frombuffer(message.data, dtype=np.uint8)
        if buffer.size < height * step:
            raise ValueError('image data is shorter than its declared step')
        rows = buffer[:height * step].reshape(height, step)
        encoding = message.encoding.lower()
        if encoding in {'mono8', '8uc1'}:
            mono = rows[:, :width].reshape(height, width)
            return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
        conversions = {
            'bgr8': (3, None),
            'rgb8': (3, cv2.COLOR_RGB2BGR),
            'bgra8': (4, cv2.COLOR_BGRA2BGR),
            'rgba8': (4, cv2.COLOR_RGBA2BGR),
        }
        if encoding not in conversions:
            raise ValueError(f'unsupported encoding: {message.encoding}')
        channels, conversion = conversions[encoding]
        image = rows[:, :width * channels].reshape(height, width, channels)
        return (
            image.copy() if conversion is None
            else cv2.cvtColor(image, conversion))

    @staticmethod
    def image_to_mono8(message: Image) -> np.ndarray:
        height, width, step = int(message.height), int(message.width), int(message.step)
        if height <= 0 or width <= 0 or step <= 0:
            raise ValueError('image dimensions and step must be positive')
        buffer = np.frombuffer(message.data, dtype=np.uint8)
        if buffer.size < height * step:
            raise ValueError('image data is shorter than its declared step')
        rows = buffer[:height * step].reshape(height, step)
        encoding = message.encoding.lower()
        if encoding in {'mono8', '8uc1'}:
            return rows[:, :width].copy()
        channels_and_code = {'rgb8': (3, 7), 'bgr8': (3, 6), 'rgba8': (4, 11), 'bgra8': (4, 10)}
        if encoding not in channels_and_code:
            raise ValueError(f'unsupported encoding: {message.encoding}')
        channels, code = channels_and_code[encoding]
        return cv2.cvtColor(rows[:, :width * channels].reshape(height, width, channels), code)
