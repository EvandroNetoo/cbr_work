from pathlib import Path


PACKAGE = Path(__file__).parents[1]


def test_one_process_hosts_four_serialized_actions():
    source = (PACKAGE / "so_arm_101_moveit_config" / "manipulation_server.py").read_text()
    for name in ("pick_cube", "store_cube", "retrieve_cube", "place_cube"):
        assert f'"/manipulation/{name}"' in source
    assert "self._busy" in source
    assert "GoalResponse.REJECT" in source
    assert "MultiThreadedExecutor(num_threads=3)" in source


def test_right_slot_has_no_unsafe_guessed_pose():
    source = (PACKAGE / "so_arm_101_moveit_config" / "manipulation_server.py").read_text()
    assert '"right_store_state": ""' in source
    assert '"right_retrieve_state": ""' in source
    assert "Estado MoveIt do braço não configurado" in source


def test_legacy_script_is_finite_mission_client():
    source = (PACKAGE / "scripts" / "pegar_e_colocar.py").read_text()
    assert "ExecuteMission" in source
    assert '"/mission/execute"' in source
    assert "while True" not in source
