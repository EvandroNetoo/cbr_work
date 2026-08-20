from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]


def _display(config, name):
    displays = config['Visualization Manager']['Displays']
    return next(display for display in displays if display['Name'] == name)


def test_telemetry_launch_only_starts_rviz():
    source = (PACKAGE_ROOT / 'launch' / 'telemetry.launch.py').read_text()

    assert "package='rviz2'" in source
    assert "executable='rviz2'" in source
    assert "'config', 'telemetry.rviz'" in source
    assert 'DeclareLaunchArgument' not in source

    for forbidden in (
            'robot_state_publisher', 'ros2_control_node',
            'controller_manager', 'move_group', 'cbr_lidar', 'cbr_camera'):
        assert forbidden not in source


def test_telemetry_config_uses_live_robot_topics():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'telemetry.rviz').read_text())
    manager = config['Visualization Manager']

    assert manager['Global Options']['Fixed Frame'] == 'odom'

    robot = _display(config, 'RobotModel')
    assert robot['Enabled'] is True
    assert robot['Description Topic']['Value'] == '/robot_description'

    lidar = _display(config, 'LiDAR frontal')
    assert lidar['Enabled'] is True
    assert lidar['Topic']['Value'] == '/scan_front'
    assert lidar['Topic']['Reliability Policy'] == 'Best Effort'

    odometry = _display(config, 'Odometria')
    assert odometry['Enabled'] is True
    assert odometry['Topic']['Value'] == '/odom'

    tf_display = _display(config, 'TF (diagnostico)')
    assert tf_display['Enabled'] is False

    camera = _display(config, 'Camera retificada')
    assert camera['Enabled'] is False
    assert camera['Topic']['Value'] == '/camera/image_rect'


def test_rviz_config_is_installed_by_setup():
    setup_source = (PACKAGE_ROOT / 'setup.py').read_text()
    assert "glob('config/*.rviz')" in setup_source
