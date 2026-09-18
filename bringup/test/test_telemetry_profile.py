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
            'controller_manager', 'move_group', 'lidar', 'camera'):
        assert forbidden not in source


def test_telemetry_config_uses_live_robot_topics():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'telemetry.rviz').read_text())
    manager = config['Visualization Manager']

    assert manager['Global Options']['Fixed Frame'] == '/map'

    assert [panel['Class'] for panel in config['Panels']] == [
        'rviz_common/Displays']

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

    apriltag_debug = _display(config, 'Debug AprilTag')
    assert apriltag_debug['Enabled'] is True
    assert apriltag_debug['Topic']['Value'] == '/apriltags/debug_image'

    containers_debug = _display(config, 'Debug containers')
    assert containers_debug['Enabled'] is True
    assert containers_debug['Topic']['Value'] == '/containers/debug_image'

    motion_planning = _display(config, 'MotionPlanning')
    assert motion_planning['Class'] == 'moveit_rviz_plugin/MotionPlanning'
    assert motion_planning['Enabled'] is False
    assert motion_planning['Value'] is False
    assert motion_planning['Robot Description'] == 'robot_description'
    assert motion_planning['Planning Scene Topic'] == 'monitored_planning_scene'

    tool_classes = [tool['Class'] for tool in manager['Tools']]
    assert 'rviz_default_plugins/SetGoal' in tool_classes
    assert 'nav2_rviz_plugins/GoalTool' not in tool_classes


def test_rviz_config_is_installed_by_setup():
    setup_source = (PACKAGE_ROOT / 'setup.py').read_text()
    assert "glob('config/*.rviz')" in setup_source
