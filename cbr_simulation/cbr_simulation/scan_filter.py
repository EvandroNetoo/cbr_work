"""Apply the physical XV-11's chassis blind sectors to simulated scans."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


# Four corner columns of the lower frame occlude the scanner. The real XV-11
# driver removes these sectors before publishing /scan_front as well.
BLIND_SECTORS_DEG = ((42.0, 52.0), (133.0, 143.0),
                     (222.0, 232.0), (315.0, 325.0))


def remove_chassis_returns(scan: LaserScan) -> LaserScan:
    result = LaserScan()
    result.header = scan.header
    result.angle_min = scan.angle_min
    result.angle_max = scan.angle_max
    result.angle_increment = scan.angle_increment
    result.time_increment = scan.time_increment
    result.scan_time = scan.scan_time
    result.range_min = scan.range_min
    result.range_max = scan.range_max
    result.ranges = list(scan.ranges)
    result.intensities = list(scan.intensities)
    for index in range(len(result.ranges)):
        degrees = math.degrees(scan.angle_min + index * scan.angle_increment) % 360
        if any(start <= degrees <= end for start, end in BLIND_SECTORS_DEG):
            result.ranges[index] = float('inf')
            if index < len(result.intensities):
                result.intensities[index] = 0.0
    return result


class SimulatedXV11(Node):
    def __init__(self):
        super().__init__('simulated_xv11')
        self._publisher = self.create_publisher(
            LaserScan, '/scan_front', qos_profile_sensor_data)
        self._subscription = self.create_subscription(
            LaserScan, '/gazebo/scan_front', self._on_scan,
            qos_profile_sensor_data)

    def _on_scan(self, message):
        self._publisher.publish(remove_chassis_returns(message))


def main():
    rclpy.init()
    node = SimulatedXV11()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
