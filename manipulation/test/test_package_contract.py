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
    assert 'analyze_apriltags' not in table
    assert 'analyze_containers' not in table

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
    assert 'uint8 RECOVERY_OUT_OF_REACH=1' in pick
    assert 'uint8 RECOVERY_MOVEIT_UNREACHABLE=2' in pick
    assert 'geometry_msgs/PoseStamped detected_pose' in pick
    assert 'interfaces/AprilTagStampedDetection[] observed_detections' in pick
    assert '\nint32 tag_id\n' in pick
    for name in (
        'StoreObject.action', 'RetrieveObject.action', 'PlaceOnTable.action',
        'PlaceInContainer.action', 'StackObject.action', 'PlaceOnShelf.action',
        'PlaceAtPose.action',
    ):
        assert 'int32 object_tag_id' in (actions / name).read_text()

    prepare = (actions / 'PrepareManipulator.action').read_text()
    assert 'bool gripper_loaded' in prepare

    result = (SOURCE_ROOT / 'interfaces' / 'msg' / 'ManipulationResult.msg').read_text()
    assert 'int32 object_tag_id' in result
    assert 'bool effect_known' in result
    assert 'bool state_known' not in result


def test_server_serializes_actions_and_propagates_cancellation():
    source = (PACKAGE / 'manipulation' / 'node.py').read_text()
    assert 'GoalResponse.REJECT' in source
    assert 'self._busy' in source
    assert 'self._motion.cancelar_objetivo_ativo()' in source
    assert 'MultiThreadedExecutor(num_threads=4)' in source
    assert 'spin_until_future_complete' not in source
    assert 'monitorar_estados_continuamente=False' in source
    assert 'self._motion.iniciar_monitoramento_dos_estados()' in source
    assert 'self._motion.parar_monitoramento_dos_estados()' in source
    assert 'PlaceObject' not in source
    assert 'MissionStateClient' not in source
    assert '_inventory' not in source
    assert 'state_service' not in source
    assert 'create_publisher' not in source
    for callback in (
        '_execute_place_on_table', '_execute_place_in_container',
        '_execute_stack', '_execute_place_on_shelf', '_execute_place_at_pose',
    ):
        assert callback in source


def test_retrieve_uses_explicit_waypoints_before_and_after_grasp():
    source = (PACKAGE / 'manipulation' / 'node.py').read_text()
    retrieve = source.split('def _execute_retrieve', 1)[1]
    retrieve = retrieve.split('def _validate_target_pose', 1)[0]

    approach_safe = retrieve.index('self._arm_state(slot.safe_state')
    pre_grip = retrieve.index("self._gripper('pre_grip'", approach_safe)
    retrieve_pose = retrieve.index(
        'self._arm_state(slot.retrieve_state', pre_grip
    )
    close_gripper = retrieve.index("self._gripper('grip'", retrieve_pose)
    retreat_safe = retrieve.index(
        'self._arm_state(\n                    slot.safe_state', close_gripper
    )

    assert (
        approach_safe < pre_grip < retrieve_pose < close_gripper < retreat_safe
    )
    assert 'self._transfer_state(' not in retrieve
    assert 'slot.store_state' not in retrieve
    assert 'self._safe()' not in retrieve


def test_pick_returns_to_approach_before_detection_pose():
    source = (PACKAGE / 'manipulation' / 'node.py').read_text()
    pick = source.split('def _execute_pick', 1)[1]
    pick = pick.split('def _execute_store', 1)[0]

    close_gripper = pick.index("self._gripper('grip'")
    return_approach = pick.index(
        'restricoes_de_pre_pegada(approach_pose)', close_gripper
    )
    return_detection = pick.index('self._transfer_state(', return_approach)

    assert close_gripper < return_approach < return_detection
    assert 'executar_trajetoria_invertida' not in pick


def test_cartesian_deposit_finishes_in_apriltag_observation_pose():
    source = (PACKAGE / 'manipulation' / 'node.py').read_text()
    release = source.split('def _release_at_pose', 1)[1]
    release = release.split('def _execute_place_on_table', 1)[0]

    open_gripper = release.index("self._gripper('open'")
    retreat = release.index(
        'restricoes_de_pre_pegada(retreat_pose)', open_gripper
    )
    return_detection = release.index('self._transfer_state(', retreat)

    assert open_gripper < retreat < return_detection
    assert 'self._safe(False)' not in release


def test_container_sequence_has_no_operational_waypoints():
    source = (PACKAGE / 'manipulation' / 'node.py').read_text()
    container = source.split('def _execute_place_in_container', 1)[1]
    container = container.split('def _execute_stack', 1)[0]
    plan = container.index('planejar_validar_e_executar')
    opening = container.index("self._gripper('open'", plan)
    detection_pose = container.index('self._transfer_state(', opening)
    assert plan < opening < detection_pose
    assert 'restricoes_de_pre_pegada' not in container
    assert 'retreat_pose' not in container


def test_launch_installs_profiles_from_package_share():
    source = (PACKAGE / 'launch' / 'manipulation.launch.py').read_text()
    assert "FindPackageShare('manipulation')" in source
    assert "'profiles_file': profiles" in source
    assert "'cargo_slots_file': cargo" in source
