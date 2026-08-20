"""Publicador ROS 2 da IMU da Mariola."""

from __future__ import annotations

import math
import time
import traceback

import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from .imu_driver import GyroBiasCalibrator, ImuDriver, ImuSample


ORIENTATION_COVARIANCE = [
    0.001, 0.0, 0.0,
    0.0, 0.001, 0.0,
    0.0, 0.0, 0.1,
]
ANGULAR_VELOCITY_COVARIANCE = [
    0.0001, 0.0, 0.0,
    0.0, 0.0001, 0.0,
    0.0, 0.0, 0.0001,
]
LINEAR_ACCELERATION_COVARIANCE = [
    0.001, 0.0, 0.0,
    0.0, 0.001, 0.0,
    0.0, 0.0, 0.001,
]


def fill_imu_message(
    message: Imu,
    sample: ImuSample,
    corrected_angular_velocity: tuple[float, float, float],
    frame_id: str,
) -> None:
    """Preenche os campos independentes do relógio para facilitar testes."""
    qw, qx, qy, qz = sample.orientation_wxyz
    message.header.frame_id = frame_id
    message.orientation.w = qw
    message.orientation.x = qx
    message.orientation.y = qy
    message.orientation.z = qz
    message.orientation_covariance = ORIENTATION_COVARIANCE
    message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = (
        corrected_angular_velocity)
    message.angular_velocity_covariance = ANGULAR_VELOCITY_COVARIANCE
    message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = (
        sample.linear_acceleration_xyz)
    message.linear_acceleration_covariance = LINEAR_ACCELERATION_COVARIANCE


class ImuNode(Node):
    def __init__(self, driver=None) -> None:
        super().__init__('cbr_imu_node')
        self.declare_parameter('topic', '/imu/data')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('hardware.serial_port', 5)
        self.declare_parameter('hardware.serial_baud_rate', 115200)
        self.declare_parameter('hardware.serial_timeout_sec', 0.02)
        self.declare_parameter('max_consecutive_read_failures', 5)
        self.declare_parameter('calibration.warmup_sec', 0.5)
        self.declare_parameter('calibration.sample_count', 40)
        self.declare_parameter('calibration.stationary_max_rad_s', 0.05)
        self.declare_parameter('calibration.timeout_sec', 15.0)

        rate = float(self.get_parameter('publish_rate_hz').value)
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError('publish_rate_hz deve ser positivo e finito.')
        serial_timeout = float(self.get_parameter('hardware.serial_timeout_sec').value)
        if serial_timeout >= 1.0 / rate:
            raise ValueError('O timeout serial deve ser menor que o período de publicação.')

        self._topic = str(self.get_parameter('topic').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._max_failures = int(
            self.get_parameter('max_consecutive_read_failures').value)
        if self._max_failures <= 0:
            raise ValueError('max_consecutive_read_failures deve ser positivo.')
        self._warmup_sec = float(self.get_parameter('calibration.warmup_sec').value)
        self._calibration_timeout_sec = float(
            self.get_parameter('calibration.timeout_sec').value)
        if self._warmup_sec < 0.0 or self._calibration_timeout_sec <= self._warmup_sec:
            raise ValueError('Tempos de calibração inválidos.')

        self._driver = driver or ImuDriver(
            serial_port=int(self.get_parameter('hardware.serial_port').value),
            baud_rate=int(self.get_parameter('hardware.serial_baud_rate').value),
            timeout_sec=serial_timeout,
        )
        self._calibrator = GyroBiasCalibrator(
            sample_count=int(self.get_parameter('calibration.sample_count').value),
            stationary_max_rad_s=float(
                self.get_parameter('calibration.stationary_max_rad_s').value),
        )
        self._started_monotonic = time.monotonic()
        self._consecutive_failures = 0
        self._publisher = self.create_publisher(
            Imu, self._topic, qos_profile_sensor_data)
        self._timer = self.create_timer(1.0 / rate, self._read_and_publish)
        self.get_logger().info(
            f'IMU iniciada a {rate:g} Hz; mantenha o robô parado durante a calibração.')

    def _read_and_publish(self) -> None:
        try:
            sample = self._driver.read_sample()
            self._consecutive_failures = 0
        except Exception as error:
            self._consecutive_failures += 1
            self.get_logger().warning(
                f'Falha de leitura da IMU ({self._consecutive_failures}/'
                f'{self._max_failures}): {error}')
            if self._consecutive_failures >= self._max_failures:
                raise RuntimeError('Limite de falhas consecutivas da IMU excedido.') from error
            return

        elapsed = time.monotonic() - self._started_monotonic
        if self._calibrator.bias is None:
            if elapsed >= self._calibration_timeout_sec:
                raise RuntimeError(
                    'Não foi possível calibrar a IMU parada dentro do tempo limite.')
            if elapsed < self._warmup_sec:
                return
            if not self._calibrator.add(sample.angular_velocity_xyz):
                return
            self.get_logger().info(
                'Calibração angular concluída; bias xyz='
                f'{self._calibrator.bias}.')

        message = Imu()
        message.header.stamp = self.get_clock().now().to_msg()
        fill_imu_message(
            message,
            sample,
            self._calibrator.correct(sample.angular_velocity_xyz),
            self._frame_id,
        )
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
        node = ImuNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        logger = node.get_logger() if node is not None else get_logger('cbr_imu_node')
        logger.fatal(f'Driver da IMU encerrado: {error}')
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
