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


def test_readiness_subscriber_accepts_best_effort_hardware_state():
    source = (
        Path(__file__).parents[1] / 'so_arm_101_bringup' /
        'wait_for_joint_states.py').read_text()
    assert 'qos_profile_sensor_data' in source
    assert 'self._state_callback,\n            qos_profile_sensor_data,' in source


def test_simulation_demo_and_isolated_rviz_profiles_are_removed():
    for name in ('sim.launch.py', 'model_demo.launch.py', 'rviz.launch.py'):
        assert not (LAUNCH_DIR / name).exists()


def test_removed_mixed_profiles_are_not_installed_from_source():
    assert not (LAUNCH_DIR / 'display.launch.py').exists()
    assert not (LAUNCH_DIR / 'keyboard_control.launch.py').exists()
