from pathlib import Path

import pytest
import yaml


@pytest.fixture(scope='module')
def controllers():
    path = Path(__file__).parents[1] / 'config' / 'controllers.yaml'
    return yaml.safe_load(path.read_text())


def test_mecanum_geometry_frames_and_timeout(controllers):
    assert controllers['controller_manager']['ros__parameters'][
        'update_rate'] == 30
    parameters = controllers['base_controller']['ros__parameters']
    assert parameters['kinematics']['wheels_radius'] == pytest.approx(0.034)
    assert parameters['kinematics'][
        'sum_of_robot_center_projection_on_X_Y_axis'] == pytest.approx(0.2225)
    assert parameters['reference_timeout'] == pytest.approx(0.25)
    assert parameters['odom_frame_id'] == 'odom'
    assert parameters['base_frame_id'] == 'base_footprint'
    assert parameters['enable_odom_tf'] is False


def test_controller_uses_exact_four_wheel_joints(controllers):
    parameters = controllers['base_controller']['ros__parameters']
    wheels = {
        'front_left_wheel_joint', 'front_right_wheel_joint',
        'rear_left_wheel_joint', 'rear_right_wheel_joint'}
    configured = {
        value for key, value in parameters.items()
        if key.endswith('_wheel_command_joint_name')}
    assert configured == wheels
    assert controllers['joint_state_broadcaster']['ros__parameters']['joints'] == [
        'front_left_wheel_joint', 'front_right_wheel_joint',
        'rear_left_wheel_joint', 'rear_right_wheel_joint']


def test_base_stack_keeps_physical_config_in_yaml_and_driver_exposes_rollback():
    package = Path(__file__).parents[1]
    launch_source = (package / 'launch' / 'real.launch.py').read_text()
    driver_source = (
        package.parent / 'base_hardware' / 'launch' / 'driver.launch.py'
    ).read_text()
    assert 'DeclareLaunchArgument' not in launch_source
    assert "'deduplicate_commands', default_value='true'" in driver_source
    assert "'command_heartbeat_hz', default_value='5.0'" in driver_source
    assert "'hardware.expansion_serial_port'" not in driver_source
    assert "os.environ.get('VIRTUAL_ENV')" in driver_source
    assert "FindPackageShare('lidar')" in launch_source
    assert "FindPackageShare('imu')" in launch_source
    assert '--remap ~/odometry:=/wheel/odom' in launch_source


def test_readiness_waits_for_calibrated_imu():
    source = (
        Path(__file__).parents[1] / 'base_bringup' /
        'wait_for_wheel_states.py').read_text()
    assert "Imu, '/imu/data'" in source
    assert 'qos_profile_sensor_data' in source
    assert 'self._wheels_ready and self._imu_ready' in source
