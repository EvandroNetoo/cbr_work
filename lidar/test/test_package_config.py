from pathlib import Path

import yaml


PACKAGE = Path(__file__).parents[1]


def test_real_yaml_matches_robot_description_and_wiring():
    config = yaml.safe_load(
        (PACKAGE / 'config' / 'lidar.yaml').read_text(encoding='utf-8'))
    parameters = config['lidar_node']['ros__parameters']
    assert parameters['poll_rate_hz'] == 20.0
    assert parameters['scan.topic'] == '/scan_front'
    assert parameters['scan.frame_id'] == 'lidar_front_link'
    assert parameters['scan.angle_start_deg'] == 307
    assert parameters['scan.angle_end_deg'] == 217
    assert parameters['scan.valid_intervals_deg'] == [307, 67, 167, 196]
    assert parameters['hardware.serial_port'] == 1
    assert parameters['hardware.serial_baud_rate'] == 115200
    assert parameters['hardware.serial_read_chunk_size'] == 64
    assert parameters['hardware.data_timeout_sec'] == 5.0
    assert parameters['hardware.relay_gpio_chip'] == '/dev/gpiochip1'
    assert parameters['hardware.relay_pin'] == 266
    assert parameters['hardware.relay_active_low'] is True


def test_launch_is_fixed_to_package_yaml_without_cli_arguments():
    source = (PACKAGE / 'launch' / 'lidar.launch.py').read_text(encoding='utf-8')
    assert 'DeclareLaunchArgument' not in source
    assert "FindPackageShare('lidar')" in source
    assert "'config', 'lidar.yaml'" in source
    assert "prefix='/usr/bin/python3'" in source
