"""Static contracts for the embedded and workstation launch profiles."""

from pathlib import Path


LAUNCH_DIR = Path(__file__).parents[1] / 'launch'


def _source(name):
    return (LAUNCH_DIR / name).read_text()


def test_embedded_profiles_do_not_start_desktop_processes():
    banned = ('rviz2', 'gazebo', 'joint_state_publisher_gui', 'teleop')
    source = _source('real.launch.py')
    assert not any(term in source for term in banned)


def test_real_profile_waits_for_slow_physical_initialization():
    source = _source('real.launch.py')
    assert "DeclareLaunchArgument(\n            'hardware_state_timeout'" in source
    assert "default_value='45.0'" in source
    assert "LaunchConfiguration('hardware_state_timeout')" in source


def test_live_rviz_profile_has_no_robot_state_or_joint_state_publisher():
    source = _source('rviz.launch.py')
    assert "package='rviz2'" in source
    assert 'robot_state_publisher' not in source
    assert 'joint_state_publisher' not in source


def test_offline_model_profile_is_explicitly_gui_only():
    source = _source('model_demo.launch.py')
    assert 'robot_state_publisher' in source
    assert 'joint_state_publisher_gui' in source
    assert 'rviz2' in source


def test_removed_mixed_profiles_are_not_installed_from_source():
    assert not (LAUNCH_DIR / 'display.launch.py').exists()
    assert not (LAUNCH_DIR / 'keyboard_control.launch.py').exists()
