from pathlib import Path


def test_robot_profile_is_headless_and_always_starts_vision():
    source = (Path(__file__).parents[1] / 'launch' / 'robot.launch.py').read_text()
    assert "FindPackageShare('cbr_camera')" in source
    assert "'rectify': 'true'" in source
    assert "'camera_framerate', default_value='15.0'" in source
    assert "'controller_update_rate', default_value='30'" in source
    assert "'arm_buffer_commands', default_value='true'" in source
    assert "'arm_deduplicate_commands', default_value='true'" in source
    assert "'arm_command_heartbeat_hz', default_value='5.0'" in source
    assert "'base_deduplicate_commands', default_value='true'" in source
    assert "'base_command_heartbeat_hz', default_value='5.0'" in source
    assert "FindPackageShare('cbr_apriltag')" in source
    assert "parameters=[{'timeout_sec': hardware_timeout}]" in source
    assert "FindPackageShare('cbr_base_hardware')" in source
    assert "FindPackageShare('cbr_lidar')" in source
    assert "FindPackageShare('cbr_robot_description')" in source
    assert source.count("executable='ros2_control_node'") == 1
    assert "arguments=['joint_state_broadcaster'" in source
    assert 'enable_apriltag' not in source
    assert 'IfCondition' not in source
    assert 'rviz2' not in source
    assert 'joint_state_publisher_gui' not in source
    assert 'gazebo' not in source
