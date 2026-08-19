from pathlib import Path


def test_robot_profile_is_headless_and_always_starts_vision():
    source = (Path(__file__).parents[1] / 'launch' / 'robot.launch.py').read_text()
    assert "FindPackageShare('cbr_camera')" in source
    assert "'rectify': 'true'" in source
    assert "FindPackageShare('cbr_apriltag')" in source
    assert "parameters=[{'timeout_sec': hardware_timeout}]" in source
    assert "FindPackageShare('cbr_base_hardware')" in source
    assert "FindPackageShare('cbr_robot_description')" in source
    assert source.count("executable='ros2_control_node'") == 1
    assert "arguments=['joint_state_broadcaster'" in source
    assert 'enable_apriltag' not in source
    assert 'IfCondition' not in source
    assert 'rviz2' not in source
    assert 'joint_state_publisher_gui' not in source
    assert 'gazebo' not in source
