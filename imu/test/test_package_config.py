from pathlib import Path

import yaml


PACKAGE = Path(__file__).parents[1]


def test_default_configuration_is_low_rate_and_uses_serial5():
    config = yaml.safe_load((PACKAGE / 'config' / 'imu.yaml').read_text())
    parameters = config['imu_node']['ros__parameters']
    assert parameters['publish_rate_hz'] == 50.0
    assert parameters['topic'] == '/imu/data'
    assert parameters['frame_id'] == 'imu_link'
    assert parameters['hardware.serial_port'] == 5
    assert parameters['hardware.serial_timeout_sec'] == 0.015
    assert parameters['calibration.sample_count'] == 100


def test_node_has_one_timer_and_no_background_thread():
    source = (PACKAGE / 'imu' / 'imu_node.py').read_text()
    driver = (PACKAGE / 'imu' / 'imu_driver.py').read_text()
    assert source.count('create_timer(') == 1
    assert 'threading' not in source
    assert 'threading' not in driver


def test_ekf_fuses_only_planar_wheel_velocity_and_imu_yaw_rate():
    config = yaml.safe_load((PACKAGE / 'config' / 'ekf.yaml').read_text())
    parameters = config['ekf_filter_node']['ros__parameters']
    assert parameters['frequency'] == 20.0
    assert parameters['two_d_mode'] is True
    assert parameters['publish_tf'] is True
    assert parameters['base_link_frame'] == 'base_footprint'
    assert parameters['odom0'] == '/wheel/odom'
    assert parameters['imu0'] == '/imu/data'
    enabled_odom = [index for index, value in enumerate(
        parameters['odom0_config']) if value]
    enabled_imu = [index for index, value in enumerate(
        parameters['imu0_config']) if value]
    assert enabled_odom == [6, 7]
    assert enabled_imu == [11]


def test_localization_launch_preserves_public_odom_topic():
    source = (PACKAGE / 'launch' / 'imu_localization.launch.py').read_text()
    assert "executable='ekf_node'" in source
    assert "('odometry/filtered', '/odom')" in source
