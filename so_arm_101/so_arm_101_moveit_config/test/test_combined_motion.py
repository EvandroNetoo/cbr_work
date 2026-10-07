"""The retrieve preparation uses one MoveIt goal covering both controller groups."""
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
import yaml

from so_arm_101_moveit_config.configuracao import (
    ESTADOS_DOS_GRUPOS, GRUPO_BRACO, GRUPO_GARRA, GRUPO_BRACO_GARRA,
    TOLERANCIA_DAS_JUNTAS_DE_ESTADOS, TOLERANCIA_DA_JUNTA_DA_GARRA,
)
from so_arm_101_moveit_config.movimento import ExecutorDoMoveIt


CONFIG = Path(__file__).parents[1] / 'config'


@pytest.mark.parametrize('side', ['left', 'right'])
def test_one_combined_goal_preserves_named_targets_and_joint_tolerances(side):
    executor = ExecutorDoMoveIt.__new__(ExecutorDoMoveIt)
    executor.no = SimpleNamespace(get_logger=lambda: SimpleNamespace(info=lambda _: None))
    executor._estado_articular_ja_atingido = lambda *_args: False
    calls = []
    executor.executar_objetivo = lambda *args, **kwargs: calls.append((args, kwargs))
    arm_state = f'safe_cube_{side}'
    original_arm = dict(ESTADOS_DOS_GRUPOS[GRUPO_BRACO][arm_state])
    original_gripper = dict(ESTADOS_DOS_GRUPOS[GRUPO_GARRA]['pre_grip'])
    executor.mover_braco_e_garra_para_estados(arm_state, 'pre_grip', 'prepare')
    assert len(calls) == 1
    args, kwargs = calls[0]
    assert args[0] == GRUPO_BRACO_GARRA
    assert len(args[1]) == 1
    joints = {joint.joint_name: joint for joint in args[1][0].joint_constraints}
    assert set(joints) == set(original_arm) | set(original_gripper)
    for targets, tolerance in ((original_arm, TOLERANCIA_DAS_JUNTAS_DE_ESTADOS),
                               (original_gripper, TOLERANCIA_DA_JUNTA_DA_GARRA)):
        for name, position in targets.items():
            assert joints[name].position == position
            assert joints[name].tolerance_above == tolerance
            assert joints[name].tolerance_below == tolerance
    assert ESTADOS_DOS_GRUPOS[GRUPO_BRACO][arm_state] == original_arm
    assert ESTADOS_DOS_GRUPOS[GRUPO_GARRA]['pre_grip'] == original_gripper


def test_combined_goal_is_skipped_only_when_both_groups_have_finished():
    executor = ExecutorDoMoveIt.__new__(ExecutorDoMoveIt)
    executor.no = SimpleNamespace(get_logger=lambda: SimpleNamespace(info=lambda _: None))
    executor._estado_articular_ja_atingido = lambda *_args: True
    executor.executar_objetivo = lambda *_args, **_kwargs: pytest.fail('already reached')
    executor.mover_braco_e_garra_para_estados('safe_cube_left', 'pre_grip', 'prepare')


def test_combined_group_has_planner_and_existing_controllers_cover_its_joints():
    srdf = ET.parse(CONFIG / 'so_arm_101.srdf').getroot()
    group = srdf.find(f"group[@name='{GRUPO_BRACO_GARRA}']")
    assert {sub.attrib['name'] for sub in group.findall('group')} == {GRUPO_BRACO, GRUPO_GARRA}
    planning = yaml.safe_load((CONFIG / 'ompl_planning.yaml').read_text())
    assert planning[GRUPO_BRACO_GARRA]['planner_configs'] == ['RRTConnect']
    controllers = yaml.safe_load((CONFIG / 'moveit_controllers.yaml').read_text())[
        'moveit_simple_controller_manager']
    covered = {joint for name in controllers['controller_names']
               for joint in controllers[name]['joints']}
    targets = set(ESTADOS_DOS_GRUPOS[GRUPO_BRACO]['safe_cube_left']) | set(
        ESTADOS_DOS_GRUPOS[GRUPO_GARRA]['pre_grip'])
    assert targets <= covered


@pytest.mark.parametrize('reached_arm,reached_gripper', [(True, False), (False, True)])
def test_combined_preparation_still_runs_when_one_group_is_pending(reached_arm, reached_gripper):
    executor = ExecutorDoMoveIt.__new__(ExecutorDoMoveIt)
    executor.no = SimpleNamespace(get_logger=lambda: SimpleNamespace(info=lambda _: None))
    executor._estado_articular_ja_atingido = lambda group, *_args: (
        reached_arm if group == GRUPO_BRACO else reached_gripper)
    calls = []
    executor.executar_objetivo = lambda *args, **kwargs: calls.append(args)
    executor.mover_braco_e_garra_para_estados('safe_cube_left', 'pre_grip', 'prepare')
    assert len(calls) == 1
    assert calls[0][0] == GRUPO_BRACO_GARRA
