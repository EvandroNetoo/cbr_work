from pathlib import Path


PACKAGE = Path(__file__).parents[1]
SOURCE_ROOT = PACKAGE.parent


def test_ros_package_and_execute_action_are_registered():
    package_xml = (PACKAGE / 'package.xml').read_text()
    setup = (PACKAGE / 'setup.py').read_text()
    cmake = (SOURCE_ROOT / 'interfaces' / 'CMakeLists.txt').read_text()

    assert '<name>mission_manager</name>' in package_xml
    assert '<exec_depend>nav2_msgs</exec_depend>' in package_xml
    assert 'mission_manager_node = mission_manager.node:main' in setup
    assert 'action/ExecuteMission.action' in cmake


def test_package_stays_minimal_and_uses_existing_actions():
    source = (PACKAGE / 'mission_manager' / 'node.py').read_text()

    assert 'NavigateToPose' in source
    assert 'MoveToDistance' in source
    assert 'FollowWall' in source
    assert 'PrepareManipulator.Goal.NAVIGATION' in source
    assert 'self._current_step_index' in source
    assert 'self._current_location' in source
    assert 'world_state' not in source
    assert not (PACKAGE / 'mission_manager' / 'planners').exists()


def test_launch_resolves_installed_arena_and_plan_directory():
    source = (PACKAGE / 'launch' / 'mission_manager.launch.py').read_text()

    assert "FindPackageShare('mission_manager')" in source
    assert "'arena_file': arena" in source
    assert "'plans_directory': plans" in source


def test_execute_mission_contract_reports_step_and_completion():
    action = (
        SOURCE_ROOT / 'interfaces' / 'action' / 'ExecuteMission.action'
    ).read_text()

    assert 'string plan_id' in action
    assert 'uint32 completed_steps' in action
    assert 'string failed_step_id' in action
    assert 'uint32 current_step_index' in action
    assert 'string operation' in action
