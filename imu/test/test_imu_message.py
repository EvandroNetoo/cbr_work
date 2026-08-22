from imu.imu_driver import ImuSample
from imu.imu_node import fill_imu_message
import pytest
from sensor_msgs.msg import Imu


def test_message_mapping_preserves_quaternion_acceleration_and_corrected_gyro():
    sample = ImuSample(
        orientation_wxyz=(0.9, 0.1, 0.2, 0.3),
        angular_velocity_xyz=(0.01, 0.02, 0.03),
        linear_acceleration_xyz=(1.0, 2.0, 9.7),
    )
    message = Imu()
    fill_imu_message(message, sample, (-0.01, 0.0, 0.004), 'imu_link')
    assert message.header.frame_id == 'imu_link'
    assert (
        message.orientation.w, message.orientation.x,
        message.orientation.y, message.orientation.z,
    ) == pytest.approx((0.9, 0.1, 0.2, 0.3))
    assert (
        message.angular_velocity.x, message.angular_velocity.y,
        message.angular_velocity.z,
    ) == pytest.approx((-0.01, 0.0, 0.004))
    assert (
        message.linear_acceleration.x, message.linear_acceleration.y,
        message.linear_acceleration.z,
    ) == pytest.approx((1.0, 2.0, 9.7))
    assert len(message.orientation_covariance) == 9
    assert len(message.angular_velocity_covariance) == 9
    assert len(message.linear_acceleration_covariance) == 9
