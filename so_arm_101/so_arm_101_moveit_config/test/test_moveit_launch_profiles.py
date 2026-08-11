"""Static contracts for headless MoveIt deployment."""

from pathlib import Path


LAUNCH_DIR = Path(__file__).parents[1] / 'launch'


def test_real_planning_is_headless():
    source = (LAUNCH_DIR / 'real_planning.launch.py').read_text()
    assert 'generate_move_group_launch' in source
    assert 'generate_moveit_rviz_launch' not in source
    assert 'rviz2' not in source


def test_old_real_moveit_profile_is_removed():
    assert not (LAUNCH_DIR / 'real_moveit.launch.py').exists()
