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


def test_expected_placement_profiles_are_enabled():
    profiles = _profiles()
    enabled = {
        name for name, profile in profiles.placements.items() if profile.enabled
    }
    assert enabled == {'table', 'explicit_pose', 'stack'}


def test_nominal_table_pose_calibration_is_complete_or_empty():
    profile = _profiles().placements['table']
    calibration = (
        profile.release_x_m,
        profile.release_y_m,
        profile.release_yaw_deg,
        profile.tcp_release_offset_cm,
    )
    assert all(value is None for value in calibration) or all(
        value is not None for value in calibration
    )


def test_table_free_space_search_uses_safe_defaults_and_complete_bounds():
    profile = _profiles().placements['table']
    assert profile.free_space_min_distance_m == pytest.approx(0.08)
    assert profile.search_step_m == pytest.approx(0.01)
    assert profile.search_y_max_m == pytest.approx(-0.10)
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
    assert profiles.cargo_slots['left'].retrieve_state == 'pick_cube_left'
    assert profiles.cargo_slots['right'].store_state == 'deposit_cube_right'
    assert profiles.cargo_slots['right'].retrieve_state == 'pick_cube_right'


def test_unknown_configuration_field_is_rejected(tmp_path):
    profiles = (PACKAGE / 'config' / 'profiles.yaml').read_text()
    profiles = profiles.replace('attempts: 2', 'attempts: 2\n    typo: true')
    profile_path = tmp_path / 'profiles.yaml'
    profile_path.write_text(profiles)

    with pytest.raises(ConfigurationError, match='campos desconhecidos'):
        load_profiles(profile_path, PACKAGE / 'config' / 'cargo_slots.yaml')
