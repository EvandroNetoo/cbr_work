from pathlib import Path


def test_robot_profile_is_headless_and_always_starts_vision():
    source = (Path(__file__).parents[1] / 'launch' / 'robot.launch.py').read_text()
    assert "FindPackageShare('camera')" in source
    assert "'rectify': 'true'" in source
    assert "'camera_framerate', default_value='15.0'" in source
    assert "'controller_update_rate', default_value='30'" in source
    assert "'arm_buffer_commands', default_value=CONFIG_DEFAULT" in source
    assert "'arm_deduplicate_commands', default_value=CONFIG_DEFAULT" in source
    assert "'arm_command_heartbeat_hz', default_value=CONFIG_DEFAULT" in source
    assert "'base_deduplicate_commands', default_value='true'" in source
    assert "'base_command_heartbeat_hz', default_value='5.0'" in source
    assert "FindPackageShare('apriltag')" in source
    assert "parameters=[{'timeout_sec': hardware_timeout}]" in source
    assert "FindPackageShare('base_hardware')" in source
    assert "FindPackageShare('lidar')" in source
    assert "FindPackageShare('imu')" in source
    assert '--remap ~/odometry:=/wheel/odom' in source
    assert "FindPackageShare('robot_description')" in source
    assert source.count("executable='ros2_control_node'") == 1
    assert "arguments=['joint_state_broadcaster'" in source
    assert 'enable_apriltag' not in source
    assert 'IfCondition' not in source
    assert 'rviz2' not in source
    assert 'joint_state_publisher_gui' not in source
    assert 'gazebo' not in source


def test_readiness_waits_for_imu_in_complete_profile():
    source = (
        Path(__file__).parents[1] / 'bringup' /
        'wait_for_hardware_states.py').read_text()
    assert "Imu, '/imu/data'" in source
    assert 'qos_profile_sensor_data' in source
    assert source.count('qos_profile_sensor_data)') == 3
    assert 'self._arm_ready and self._base_ready and self._imu_ready' in source
