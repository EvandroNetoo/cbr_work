from pathlib import Path


def test_robot_profile_is_headless_and_always_starts_vision():
    source = (Path(__file__).parents[1] / 'launch' / 'robot.launch.py').read_text()
    assert "FindPackageShare('cbr_camera')" in source
    assert "'rectify': 'true'" in source
    assert "FindPackageShare('cbr_apriltag')" in source
    assert "'hardware_state_timeout': hardware_state_timeout" in source
    assert 'enable_apriltag' not in source
    assert 'IfCondition' not in source
    assert 'rviz2' not in source
    assert 'joint_state_publisher_gui' not in source
    assert 'gazebo' not in source
