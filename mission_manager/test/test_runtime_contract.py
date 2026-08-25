from pathlib import Path


PACKAGE = Path(__file__).parents[1]


def test_manager_uses_actions_without_polling_timer():
    source = (PACKAGE / "mission_manager" / "node.py").read_text()
    for action in ("NavigateToPose", "PickCube", "StoreCube", "RetrieveCube", "PlaceCube"):
        assert f"ActionClient(" in source and action in source
    assert "create_timer" not in source
    assert '"/odom"' in source
    assert '"/initialpose"' in source
    assert "_wait_base_stopped" in source


def test_public_action_contract_has_structured_partial_result():
    interface = (PACKAGE.parent / "interfaces" / "action" / "ExecuteMission.action").read_text()
    assert "uint8 PARTIAL=1" in interface
    assert "interfaces/MissionStepResult[] steps" in interface
    assert "string mission_name" in interface
