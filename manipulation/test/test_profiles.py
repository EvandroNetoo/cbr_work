from pathlib import Path

import pytest
import yaml

from manipulation.errors import ConfigurationError
from manipulation.profiles import load_profiles


PACKAGE = Path(__file__).parents[1]


@pytest.mark.parametrize('name', ['stack', 'table', 'explicit_pose'])
@pytest.mark.parametrize('approach, retreat', [(0, 0), (0, 0.08), (0.03, 0)])
def test_placement_heights_accept_zero(tmp_path, name, approach, retreat):
    raw = yaml.safe_load((PACKAGE / 'config/profiles.yaml').read_text())
    raw['placements'][name]['approach_height_m'] = approach
    raw['placements'][name]['retreat_height_m'] = retreat
    path = tmp_path / 'profiles.yaml'
    path.write_text(yaml.safe_dump(raw))

    profile = load_profiles(path, PACKAGE / 'config/cargo_slots.yaml').placements[name]
    assert profile.approach_height_m == approach
    assert profile.retreat_height_m == retreat


@pytest.mark.parametrize('field', ['approach_height_m', 'retreat_height_m'])
@pytest.mark.parametrize('value', [-0.01, float('nan'), float('inf'), True, '0'])
def test_placement_heights_reject_invalid_values(tmp_path, field, value):
    raw = yaml.safe_load((PACKAGE / 'config/profiles.yaml').read_text())
    raw['placements']['stack'][field] = value
    path = tmp_path / 'profiles.yaml'
    path.write_text(yaml.safe_dump(raw))

    with pytest.raises(ConfigurationError, match=field):
        load_profiles(path, PACKAGE / 'config/cargo_slots.yaml')


@pytest.mark.parametrize('tolerance', [None, 0.0, 2.5, 15.0, 180.0])
def test_stack_tilt_tolerance_is_loaded_independently(tmp_path, tolerance):
    raw = yaml.safe_load((PACKAGE / 'config/profiles.yaml').read_text())
    raw['placements']['stack']['tilt_tolerance_deg'] = tolerance
    path = tmp_path / 'profiles.yaml'
    path.write_text(yaml.safe_dump(raw))

    profiles = load_profiles(path, PACKAGE / 'config/cargo_slots.yaml')

    assert profiles.placements['stack'].tilt_tolerance_deg == tolerance
    assert profiles.placements['table'].tilt_tolerance_deg is None
    assert profiles.placements['explicit_pose'].tilt_tolerance_deg is None


@pytest.mark.parametrize('tolerance', [-0.1, 180.1, True, '5', float('nan'), float('inf')])
def test_stack_tilt_tolerance_rejects_invalid_values(tmp_path, tolerance):
    raw = yaml.safe_load((PACKAGE / 'config/profiles.yaml').read_text())
    raw['placements']['stack']['tilt_tolerance_deg'] = tolerance
    path = tmp_path / 'profiles.yaml'
    path.write_text(yaml.safe_dump(raw))

    with pytest.raises(ConfigurationError, match='tilt_tolerance_deg'):
        load_profiles(path, PACKAGE / 'config/cargo_slots.yaml')


def test_stack_tilt_tolerance_can_be_omitted(tmp_path):
    raw = yaml.safe_load((PACKAGE / 'config/profiles.yaml').read_text())
    del raw['placements']['stack']['tilt_tolerance_deg']
    path = tmp_path / 'profiles.yaml'
    path.write_text(yaml.safe_dump(raw))

    profiles = load_profiles(path, PACKAGE / 'config/cargo_slots.yaml')
    assert profiles.placements['stack'].tilt_tolerance_deg is None


def _profiles():
    return load_profiles(
        PACKAGE / 'config' / 'profiles.yaml',
        PACKAGE / 'config' / 'cargo_slots.yaml',
    )


def test_pickup_has_tabletop_and_shelf_front_sources():
    profiles = _profiles()
    assert set(profiles.pickup) == {'tabletop', 'shelf_front'}
    assert profiles.pickup['shelf_front'].strategy == 'front'
    assert profiles.pickup['shelf_front'].link3_to_link4_deg == 90.0
    assert profiles.pickup['tabletop'].cube_size_m == pytest.approx(0.042)
    pickup = profiles.pickup['tabletop']
    assert pickup.attempts == 1
    assert pickup.reachability_filter_enabled is True
    assert pickup.reach_min_radius_m is not None
    assert pickup.reach_max_radius_m is not None
    assert pickup.reach_x_min_m is not None
    assert pickup.reach_x_max_m is not None


def test_expected_placement_profiles_are_enabled():
    profiles = _profiles()
    enabled = {
        name for name, profile in profiles.placements.items() if profile.enabled
    }
    assert enabled == {'table', 'explicit_pose', 'container', 'stack', 'shelf', 'precision_table'}


def test_container_profile_keeps_xy_and_height_offsets_explicit():
    profile = _profiles().placements['container']
    assert profile.calibrated_reference is True
    assert profile.reference_offset_xyz[:2] == (0.0, 0.0)
    assert profile.reference_offset_xyz[2] >= 0.0
    assert profile.link3_to_link4_max_deg == pytest.approx(-10.0)


def test_table_release_calibration_is_complete_or_empty():
    profile = _profiles().placements['table']
    calibration = (
        profile.tcp_release_offset_cm,
        profile.free_space_preferred_yaw_deg,
        profile.free_space_alternate_yaw_deg,
    )
    assert all(value is None for value in calibration) or all(
        value is not None for value in calibration
    )


def test_table_free_space_search_uses_safe_defaults_and_complete_bounds():
    profile = _profiles().placements['table']
    assert profile.free_space_half_extent_x_m == pytest.approx(0.07)
    assert profile.free_space_half_extent_y_m == pytest.approx(0.04)
    assert profile.free_space_min_padding_m == pytest.approx(0.02)
    assert profile.free_space_preferred_padding_m == pytest.approx(0.04)
    assert profile.free_space_preferred_yaw_deg == pytest.approx(-90.0)
    assert profile.free_space_alternate_yaw_deg == pytest.approx(0.0)
    assert profile.reach_min_radius_m is not None
    assert profile.reach_max_radius_m is not None
    assert profile.reach_min_radius_m < profile.reach_max_radius_m
    assert profile.search_step_m == pytest.approx(0.01)
    assert profile.search_y_max_m <= -0.10
    measured_bounds = (
        profile.search_x_min_m,
        profile.search_x_max_m,
        profile.search_y_min_m,
    )
    assert all(value is None for value in measured_bounds) or all(
        value is not None for value in measured_bounds
    )


def test_semantic_placement_profiles_are_explicit():
    profiles = _profiles()
    assert set(profiles.placements) == {
        'table', 'explicit_pose', 'container', 'stack', 'shelf', 'precision_table'
    }


def test_both_measured_cargo_slots_are_enabled():
    profiles = _profiles()
    assert set(profiles.cargo_slots) == {'left', 'right'}
    assert profiles.cargo_slots['left'].store_state == 'deposit_cube_left'
    assert profiles.cargo_slots['left'].safe_state == 'safe_cube_left'
    assert profiles.cargo_slots['left'].retrieve_state == 'pick_cube_left'
    assert profiles.cargo_slots['right'].store_state == 'deposit_cube_right'
    assert profiles.cargo_slots['right'].safe_state == 'safe_cube_right'
    assert profiles.cargo_slots['right'].retrieve_state == 'pick_cube_right'


def test_unknown_configuration_field_is_rejected(tmp_path):
    profiles = (PACKAGE / 'config' / 'profiles.yaml').read_text()
    profiles = profiles.replace('attempts: 1', 'attempts: 1\n    typo: true')
    profile_path = tmp_path / 'profiles.yaml'
    profile_path.write_text(profiles)

    with pytest.raises(ConfigurationError, match='campos desconhecidos'):
        load_profiles(profile_path, PACKAGE / 'config' / 'cargo_slots.yaml')


def test_preferred_free_space_padding_cannot_be_negative(tmp_path):
    profiles = (PACKAGE / 'config' / 'profiles.yaml').read_text()
    profiles = profiles.replace(
        'free_space_preferred_padding_m: 0.04',
        'free_space_preferred_padding_m: -0.01',
    )
    profile_path = tmp_path / 'profiles.yaml'
    profile_path.write_text(profiles)

    with pytest.raises(ConfigurationError, match='maior ou igual a zero'):
        load_profiles(profile_path, PACKAGE / 'config' / 'cargo_slots.yaml')


def test_minimum_free_space_padding_cannot_exceed_preferred(tmp_path):
    profiles = (PACKAGE / 'config' / 'profiles.yaml').read_text()
    profiles = profiles.replace(
        'free_space_min_padding_m: 0.02',
        'free_space_min_padding_m: 0.05',
    )
    profile_path = tmp_path / 'profiles.yaml'
    profile_path.write_text(profiles)

    with pytest.raises(ConfigurationError, match='maior ou igual'):
        load_profiles(profile_path, PACKAGE / 'config' / 'cargo_slots.yaml')


def test_free_space_yaw_options_must_be_different(tmp_path):
    profiles = (PACKAGE / 'config' / 'profiles.yaml').read_text()
    profiles = profiles.replace(
        'free_space_alternate_yaw_deg: 0.0',
        'free_space_alternate_yaw_deg: -90.0',
    )
    profile_path = tmp_path / 'profiles.yaml'
    profile_path.write_text(profiles)

    with pytest.raises(ConfigurationError, match='devem ser diferentes'):
        load_profiles(profile_path, PACKAGE / 'config' / 'cargo_slots.yaml')


def test_reach_minimum_radius_must_be_smaller_than_maximum(tmp_path):
    profiles = (PACKAGE / 'config' / 'profiles.yaml').read_text()
    profiles = profiles.replace(
        'reach_min_radius_m: 0.155',
        'reach_min_radius_m: 0.31',
    )
    profile_path = tmp_path / 'profiles.yaml'
    profile_path.write_text(profiles)

    with pytest.raises(ConfigurationError, match='deve ser menor'):
        load_profiles(profile_path, PACKAGE / 'config' / 'cargo_slots.yaml')


def test_container_joint_limit_must_be_a_valid_angle(tmp_path):
    profiles = (PACKAGE / 'config' / 'profiles.yaml').read_text()
    profiles = profiles.replace(
        'link3_to_link4_max_deg: -10.0',
        'link3_to_link4_max_deg: -180.0',
    )
    profile_path = tmp_path / 'profiles.yaml'
    profile_path.write_text(profiles)

    with pytest.raises(ConfigurationError, match='entre -180 e 180'):
        load_profiles(profile_path, PACKAGE / 'config' / 'cargo_slots.yaml')


def test_pickup_reachability_flag_must_be_boolean(tmp_path):
    profiles = (PACKAGE / 'config' / 'profiles.yaml').read_text()
    profiles = profiles.replace(
        'reachability_filter_enabled: true',
        'reachability_filter_enabled: "true"',
    )
    profile_path = tmp_path / 'profiles.yaml'
    profile_path.write_text(profiles)

    with pytest.raises(ConfigurationError, match='deve ser booleano'):
        load_profiles(profile_path, PACKAGE / 'config' / 'cargo_slots.yaml')


def test_shelf_profile_references_an_arm_state_with_all_joints():
    import xml.etree.ElementTree as ET

    profile = _profiles().placements['shelf']
    assert profile.strategy == 'named_state'
    srdf = ET.parse(
        PACKAGE.parent / 'so_arm_101' / 'so_arm_101_moveit_config'
        / 'config' / 'so_arm_101.srdf')
    state = srdf.find(
        f".//group_state[@name='{profile.named_state}'][@group='arm']")
    assert state is not None
    assert {joint.attrib['name'] for joint in state} == {
        'base_link_to_link1', 'link1_to_link2', 'link2_to_link3',
        'link3_to_link4', 'link4_to_link5',
    }


def test_precision_profile_loads_independent_calibration_and_requires_tag_strategy(tmp_path):
    raw = yaml.safe_load((PACKAGE / 'config/profiles.yaml').read_text())
    raw['placements']['precision_table'].update(
        calibrated_reference=False, reference_offset_xyz=[0.01, -0.02, 0.03])
    path = tmp_path / 'profiles.yaml'
    path.write_text(yaml.safe_dump(raw))
    precision = load_profiles(path, PACKAGE / 'config/cargo_slots.yaml').placements['precision_table']
    assert precision.calibrated_reference is False
    assert precision.reference_offset_xyz == (0.01, -0.02, 0.03)
    raw['placements']['precision_table']['strategy'] = 'perception'
    path.write_text(yaml.safe_dump(raw))
    with pytest.raises(ConfigurationError, match='precision_table.strategy'):
        load_profiles(path, PACKAGE / 'config/cargo_slots.yaml')
