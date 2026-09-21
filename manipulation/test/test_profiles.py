from pathlib import Path

import pytest

from manipulation.errors import ConfigurationError
from manipulation.profiles import load_profiles


PACKAGE = Path(__file__).parents[1]


def _profiles():
    return load_profiles(
        PACKAGE / 'config' / 'profiles.yaml',
        PACKAGE / 'config' / 'cargo_slots.yaml',
    )


def test_pickup_has_only_tabletop_source():
    profiles = _profiles()
    assert set(profiles.pickup) == {'tabletop'}
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
    assert enabled == {'table', 'explicit_pose', 'container', 'stack'}


def test_container_profile_keeps_xy_and_height_offsets_explicit():
    profile = _profiles().placements['container']
    assert profile.calibrated_reference is True
    assert profile.reference_offset_xyz[:2] == (0.0, 0.0)
    assert profile.reference_offset_xyz[2] >= 0.0


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
        'table', 'explicit_pose', 'container', 'stack', 'shelf'
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
