from pathlib import Path

import yaml


PACKAGE = Path(__file__).parents[1]
WORKSPACE_SOURCE = PACKAGE.parent


def test_action_interface_contains_goal_result_and_feedback_contract():
    action = (
        WORKSPACE_SOURCE / 'interfaces' / 'action' / 'MoveToDistance.action'
    ).read_text()
    assert 'uint32 distance_mm' in action
    assert 'uint32 tolerance_mm' in action
    assert 'builtin_interfaces/Duration timeout' in action
    assert 'bool has_valid_reading' in action
    assert 'float32 linear_velocity_mps' in action
    assert 'uint32 consecutive_read_failures' in action
    assert action.count('---') == 2

    follow_wall = (
        WORKSPACE_SOURCE / 'interfaces' / 'action' / 'FollowWall.action'
    ).read_text()
    assert 'uint32 wall_distance_mm' in follow_wall
    assert 'int32 travel_distance_mm' in follow_wall
    assert 'uint32 wall_tolerance_mm' in follow_wall
    assert 'uint32 travel_tolerance_mm' in follow_wall
    assert 'bool has_valid_odometry' in follow_wall
    assert 'float32 traveled_distance_mm' in follow_wall
    assert 'float32 linear_y_velocity_mps' in follow_wall
    assert follow_wall.count('---') == 2


def test_physical_defaults_match_the_validated_example():
    config = yaml.safe_load(
        (PACKAGE / 'config' / 'vl53_distance.yaml').read_text())
    parameters = config['vl53_distance_action']['ros__parameters']
    assert parameters['sensor.right.channel'] == 0
    assert parameters['sensor.right.offset_mm'] == 48
    assert parameters['sensor.left.channel'] == 1
    assert parameters['sensor.left.offset_mm'] == 106
    assert parameters['sensor.median_window'] == 3
    assert parameters['sensor.ranging_timeout_ms'] == 200
    assert parameters['max_consecutive_read_failures'] == 3
    assert parameters['follow_wall_action_name'] == '/vl53/follow_wall'
    assert parameters['odom_topic'] == '/odom'
    assert parameters['odom_start_timeout_sec'] > 0.0
    assert parameters['odom_freshness_timeout_sec'] > 0.0
    assert parameters['wheel_linear_speed_limit'] == 0.238
    assert parameters['kinematic_lever'] == 0.2225
    assert parameters['linear_pid.ki'] == 0.0
    assert parameters['linear_pid.kd'] == 0.0
    assert parameters['angular_pid.ki'] == 0.0
    assert parameters['angular_pid.kd'] == 0.0
    assert parameters['travel_pid.ki'] == 0.0
    assert parameters['travel_pid.kd'] == 0.0


def test_action_uses_watchdog_and_stops_on_every_terminal_path():
    source = (PACKAGE / 'vl53_distance' / 'action_server.py').read_text()
    assert "self._state != 'idle'" in source
    assert 'self._freshness_timeout' in source
    assert 'consecutive_failures >= self._failure_limit' in source
    assert 'self._invalidate_command(publish=True)' in source
    assert "name='vl53-distance-action-goal'" in source
    assert "'/cmd_vel'" in source
    assert "'/odom'" in source
    assert 'FollowWall' in source
    assert 'rightward_displacement_mm' in source


def test_both_physical_launches_start_action_after_base_controller():
    base_launch = (
        WORKSPACE_SOURCE / 'base_bringup' / 'launch' / 'real.launch.py'
    ).read_text()
    robot_launch = (
        WORKSPACE_SOURCE / 'bringup' / 'launch' / 'robot.launch.py'
    ).read_text()
    assert "FindPackageShare('vl53_distance')" in base_launch
    assert 'OnProcessExit(target_action=base, on_exit=after_base)' in base_launch
    assert "FindPackageShare('vl53_distance')" in robot_launch
    assert '[vl53_distance] + move_group_entities' in robot_launch


def test_driver_supports_shared_bus_and_explicit_close():
    source = (PACKAGE / 'vl53_distance' / 'vl53.py').read_text()
    assert 'bus=None' in source
    assert 'self._owns_bus = bus is None' in source
    assert 'ranging_timeout_ms=TIMEOUT_RANGING_MS' in source
    assert 'def close(self):' in source


def test_launch_prefers_python_from_active_virtualenv():
    source = (
        PACKAGE / 'launch' / 'vl53_distance.launch.py'
    ).read_text()
    assert "os.environ.get('VIRTUAL_ENV')" in source
    assert "os.path.join(virtualenv, 'bin', 'python')" in source
    assert 'return sys.executable' in source
    assert 'prefix=_python_executable()' in source
