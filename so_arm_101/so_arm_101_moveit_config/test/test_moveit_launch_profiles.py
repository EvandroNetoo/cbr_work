"""Static contracts for headless MoveIt deployment."""

from pathlib import Path


LAUNCH_DIR = Path(__file__).parents[1] / 'launch'


def test_real_planning_is_headless():
    source = (LAUNCH_DIR / 'real_planning.launch.py').read_text()
    assert 'generate_move_group_launch' in source
    assert 'generate_moveit_rviz_launch' not in source
    assert 'rviz2' not in source
    assert "'hardware_state_timeout': LaunchConfiguration(" in source


def test_old_real_moveit_profile_is_removed():
    assert not (LAUNCH_DIR / 'real_moveit.launch.py').exists()


def test_main_rviz_uses_composed_robot_without_starting_publishers():
    source = (LAUNCH_DIR / 'moveit_rviz.launch.py').read_text()
    assert 'get_combined_moveit_config' in source
    assert "'config', 'moveit_robot.rviz'" in source
    assert "package='rviz2'" in source
    assert 'robot_state_publisher' not in source
    assert 'move_group' not in source


def test_standalone_demo_keeps_arm_rviz_profile():
    demo = (LAUNCH_DIR / 'demo.launch.py').read_text()
    arm_rviz = (LAUNCH_DIR / 'arm_moveit_rviz.launch.py').read_text()
    assert "'launch', 'arm_moveit_rviz.launch.py'" in demo
    assert 'get_moveit_config' in arm_rviz
    assert 'generate_moveit_rviz_launch' in arm_rviz
