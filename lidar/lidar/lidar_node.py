"""Fronteira ROS 2 do LiDAR físico da base CBR."""

from __future__ import annotations

import math
import time
import traceback

import rclpy
from rclpy.duration import Duration
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .lidar_driver import LidarConfig, LidarDriver


def _scan_start_age_ns(scan, sample_count: int, now_monotonic_ns: int) -> int:
    """Return the first-ray age while keeping the final ray at scan end."""
    end_age_ns = max(0, now_monotonic_ns - scan.end_monotonic_ns)
    if scan.rpm <= 0.0 or sample_count <= 1:
        return end_age_ns
    time_increment = (60.0 / scan.rpm) / 360.0
    sector_duration_ns = round(time_increment * (sample_count - 1) * 1e9)
    return end_age_ns + sector_duration_ns


class LidarNode(Node):
    def __init__(self, driver=None):
        super().__init__('lidar_node')
        self.declare_parameter('scan.topic', '/scan_front')
        self.declare_parameter('scan.frame_id', 'lidar_front_link')
        self.declare_parameter('scan.angle_start_deg', 307)
        self.declare_parameter('scan.angle_end_deg', 217)
        self.declare_parameter(
            'scan.valid_intervals_deg', [307, 67, 167, 196])
        self.declare_parameter('scan.range_min_m', 0.10)
        self.declare_parameter('scan.range_max_m', 3.0)
        self.declare_parameter('hardware.serial_port', 1)
        self.declare_parameter('hardware.serial_baud_rate', 115200)
        self.declare_parameter('hardware.serial_read_chunk_size', 64)
        self.declare_parameter('hardware.serial_timeout_sec', 0.10)
        self.declare_parameter('hardware.data_timeout_sec', 5.0)
        self.declare_parameter('hardware.relay_gpio_chip', '/dev/gpiochip1')
        self.declare_parameter('hardware.relay_pin', 266)
        self.declare_parameter('hardware.relay_active_low', True)
        self.declare_parameter('poll_rate_hz', 20.0)

        self._topic = str(self.get_parameter('scan.topic').value)
        self._frame_id = str(self.get_parameter('scan.frame_id').value)
        self._config = LidarConfig(
            serial_port=int(self.get_parameter('hardware.serial_port').value),
            serial_baud_rate=int(
                self.get_parameter('hardware.serial_baud_rate').value),
            serial_read_chunk_size=int(
                self.get_parameter('hardware.serial_read_chunk_size').value),
            serial_timeout_sec=float(
                self.get_parameter('hardware.serial_timeout_sec').value),
            data_timeout_sec=float(
                self.get_parameter('hardware.data_timeout_sec').value),
            relay_gpio_chip=str(
                self.get_parameter('hardware.relay_gpio_chip').value),
            relay_pin=int(self.get_parameter('hardware.relay_pin').value),
            relay_active_low=bool(
                self.get_parameter('hardware.relay_active_low').value),
            angle_start_deg=int(
                self.get_parameter('scan.angle_start_deg').value),
            angle_end_deg=int(self.get_parameter('scan.angle_end_deg').value),
            valid_intervals_deg=tuple(
                int(angle) for angle in
                self.get_parameter('scan.valid_intervals_deg').value),
            range_min_m=float(self.get_parameter('scan.range_min_m').value),
            range_max_m=float(self.get_parameter('scan.range_max_m').value),
        )
        self._config.validate()
        poll_rate = float(self.get_parameter('poll_rate_hz').value)
        if not math.isfinite(poll_rate) or poll_rate <= 0.0:
            raise ValueError('poll_rate_hz deve ser positivo e finito.')

        self._driver = driver or LidarDriver(self._config)
        self._publisher = self.create_publisher(
            LaserScan,
            self._topic,
            qos_profile_sensor_data,
        )
        self._timer = self.create_timer(1.0 / poll_rate, self._publish_if_ready)
        self.get_logger().info(
            f'LiDAR iniciado em SERIAL{self._config.serial_port}; '
            f'publicando {self._topic} no frame {self._frame_id}, setor '
            f'{self._config.angle_start_deg}°->{self._config.angle_end_deg}°, '
            f'intervalos válidos {self._config.valid_intervals_deg}.')

    def _publish_if_ready(self) -> None:
        scan = self._driver.take_scan()
        if scan is None:
            return

        message = LaserScan()
        now = self.get_clock().now()
        message.header.frame_id = self._frame_id

        ros_start_deg = ((self._config.angle_start_deg + 180) % 360) - 180
        message.angle_min = math.radians(ros_start_deg)
        message.angle_increment = math.radians(1.0)
        message.angle_max = message.angle_min + (
            (len(scan.ranges_m) - 1) * message.angle_increment)
        if scan.rpm > 0.0:
            message.scan_time = 60.0 / scan.rpm
            message.time_increment = message.scan_time / 360.0
        start_age_ns = _scan_start_age_ns(
            scan, len(scan.ranges_m), time.monotonic_ns())
        start = (
            now - Duration(nanoseconds=start_age_ns)
            if now.nanoseconds >= start_age_ns else now
        )
        message.header.stamp = start.to_msg()
        message.range_min = self._config.range_min_m
        message.range_max = self._config.range_max_m
        message.ranges = list(scan.ranges_m)
        message.intensities = []
        self._publisher.publish(message)

    def destroy_node(self):
        if getattr(self, '_driver', None) is not None:
            self._driver.close()
            self._driver = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = LidarNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        if node is not None:
            node.get_logger().fatal(f'Driver do LiDAR encerrado: {error}')
        else:
            get_logger('lidar_node').fatal(
                f'Falha ao inicializar o LiDAR: {error}')
        traceback.print_exc()
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    main()
