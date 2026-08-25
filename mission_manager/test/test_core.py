from pathlib import Path

import pytest

from mission_manager.core import (
    GRIPPER, LEFT, RIGHT, Inventory, MissionConfigurationError, load_config,
)


PACKAGE = Path(__file__).parents[1]


def test_example_mission_is_valid_and_installed_contract_is_high_level():
    config = load_config(PACKAGE / "config" / "missions.yaml")
    assert config.enabled is False
    steps = config.missions["exemplo"].steps
    assert [step.kind for step in steps] == [
        "navigate", "pick", "pick", "navigate", "place", "place"
    ]
    assert config.placements["mesa_2_centro"] == "place_cube_center"


def test_inventory_stores_left_then_right_and_keeps_third_in_gripper():
    inventory = Inventory()
    inventory.add_to_gripper(1)
    inventory.store_gripper(inventory.free_slot())
    inventory.add_to_gripper(2)
    inventory.store_gripper(inventory.free_slot())
    inventory.add_to_gripper(3)
    assert inventory.slots == {LEFT: 1, RIGHT: 2}
    assert inventory.gripper == 3
    assert inventory.locations == {1: LEFT, 2: RIGHT, 3: GRIPPER}


def test_two_cubes_can_be_delivered_in_non_stack_order_with_rehandling(tmp_path):
    mission = """
initial_pose: {x: 0, y: 0, yaw: 0}
stations: {mesa: {x: 1, y: 2, yaw: 0}}
placements: {centro: place_cube_center}
missions:
  ok:
    steps:
      - navigate: mesa
      - pick: {tag_id: 1}
      - pick: {tag_id: 2}
      - place: {tag_id: 1, placement: centro}
      - place: {tag_id: 2, placement: centro}
"""
    path = tmp_path / "missions.yaml"
    path.write_text(mission)
    assert "ok" in load_config(path).missions


def test_full_three_cube_load_rejects_inaccessible_delivery_order(tmp_path):
    mission = """
initial_pose: {x: 0, y: 0, yaw: 0}
stations: {mesa: {x: 1, y: 2, yaw: 0}}
placements: {centro: place_cube_center}
missions:
  impossible:
    steps:
      - navigate: mesa
      - pick: {tag_id: 1}
      - pick: {tag_id: 2}
      - pick: {tag_id: 3}
      - place: {tag_id: 1, placement: centro}
"""
    path = tmp_path / "missions.yaml"
    path.write_text(mission)
    with pytest.raises(MissionConfigurationError, match="não há onde guardar"):
        load_config(path)


@pytest.mark.parametrize("bad_step, message", [
    ("- pick: {tag_id: 1}", "antes de navegar"),
    ("- navigate: inexistente", "estação 'inexistente'"),
    ("- dance: {}", "Ação desconhecida"),
])
def test_invalid_missions_fail_before_motion(tmp_path, bad_step, message):
    source = f"""
initial_pose: {{x: 0, y: 0, yaw: 0}}
stations: {{mesa: {{x: 0, y: 0, yaw: 0}}}}
placements: {{centro: place_cube_center}}
missions:
  bad:
    steps:
      {bad_step}
"""
    path = tmp_path / "bad.yaml"
    path.write_text(source)
    with pytest.raises(MissionConfigurationError, match=message):
        load_config(path)
