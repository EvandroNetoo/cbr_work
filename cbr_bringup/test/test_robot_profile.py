from pathlib import Path


def test_robot_profile_is_headless_and_apriltag_is_optional():
    source = (Path(__file__).parents[1] / 'launch' / 'robot.launch.py').read_text()
    assert 'enable_apriltag' in source
    assert "default_value='false'" in source
    assert 'rviz2' not in source
    assert 'joint_state_publisher_gui' not in source
    assert 'gazebo' not in source
