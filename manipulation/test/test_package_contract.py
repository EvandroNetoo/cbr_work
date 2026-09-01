from pathlib import Path


PACKAGE = Path(__file__).parents[1]
SOURCE_ROOT = PACKAGE.parent


def test_actions_cover_pick_cargo_and_semantic_placements():
    actions = SOURCE_ROOT / 'interfaces' / 'action'
    for name in (
        'PickObject.action', 'StoreObject.action', 'RetrieveObject.action',
        'PlaceOnTable.action', 'PlaceInContainer.action', 'StackObject.action',
        'PlaceOnShelf.action', 'PlaceAtPose.action', 'PrepareManipulator.action',
    ):
        assert (actions / name).is_file()
    assert not (actions / 'PlaceObject.action').exists()

    table = (actions / 'PlaceOnTable.action').read_text()
    assert 'float32 ws_height_cm' in table
    assert 'bool analyze_apriltags' in table
    assert 'bool analyze_containers' in table

    container = (actions / 'PlaceInContainer.action').read_text()
    assert 'uint8 RED=1' in container
    assert 'uint8 BLUE=2' in container
    assert 'float32 ws_height_cm' in container

    stack = (actions / 'StackObject.action').read_text()
    assert 'int32 support_tag_id' in stack
    assert 'ws_height_cm' not in stack

    explicit = (actions / 'PlaceAtPose.action').read_text()
    assert 'geometry_msgs/PoseStamped release_pose' in explicit

    pick = (actions / 'PickObject.action').read_text()
    assert '\nint32 tag_id\n' in pick
    for name in (
        'StoreObject.action', 'RetrieveObject.action', 'PlaceOnTable.action',
        'PlaceInContainer.action', 'PlaceOnShelf.action', 'PlaceAtPose.action',
    ):
        assert '\nint32 tag_id\n' not in (actions / name).read_text()
    assert '\nint32 tag_id\n' not in stack

    result = (SOURCE_ROOT / 'interfaces' / 'msg' / 'ManipulationResult.msg').read_text()
    assert 'int32 object_tag_id' in result


def test_server_serializes_actions_and_propagates_cancellation():
    source = (PACKAGE / 'manipulation' / 'node.py').read_text()
    assert 'GoalResponse.REJECT' in source
    assert 'self._busy' in source
    assert 'self._motion.cancelar_objetivo_ativo()' in source
    assert 'MultiThreadedExecutor(num_threads=4)' in source
    assert 'spin_until_future_complete' not in source
    assert 'PlaceObject' not in source
    for callback in (
        '_execute_place_on_table', '_execute_place_in_container',
        '_execute_stack', '_execute_place_on_shelf', '_execute_place_at_pose',
    ):
        assert callback in source


def test_retrieve_uses_safe_waypoint_before_and_after_grasp():
    source = (PACKAGE / 'manipulation' / 'node.py').read_text()
    retrieve = source.split('def _execute_retrieve', 1)[1]
    retrieve = retrieve.split('def _validate_target_pose', 1)[0]

    pre_grip = retrieve.index("self._gripper('pre_grip'")
    approach_store = retrieve.index(
        'self._arm_state(slot.store_state', pre_grip
    )
    retrieve_pose = retrieve.index(
        'self._arm_state(slot.retrieve_state', approach_store
    )
    close_gripper = retrieve.index("self._gripper('grip'", retrieve_pose)
    retreat_store = retrieve.index(
        'self._arm_state(slot.store_state', close_gripper
    )
    go_home = retrieve.index('self._safe()', retreat_store)

    assert (
        pre_grip < approach_store < retrieve_pose < close_gripper
        < retreat_store < go_home
    )


def test_launch_installs_profiles_from_package_share():
    source = (PACKAGE / 'launch' / 'manipulation.launch.py').read_text()
    assert "FindPackageShare('manipulation')" in source
    assert "'profiles_file': profiles" in source
    assert "'cargo_slots_file': cargo" in source
