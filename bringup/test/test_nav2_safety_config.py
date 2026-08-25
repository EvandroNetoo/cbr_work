"""Static contracts for the workstation Nav2 safety profile."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml


PACKAGE = Path(__file__).parents[1]
PARAMS_PATH = PACKAGE / 'config' / 'nav2_navigation.yaml'
BT_PATH = PACKAGE / 'behavior_trees' / 'navigate_to_pose_safe.xml'


@pytest.fixture(scope='module')
def nav2_params():
    return yaml.safe_load(PARAMS_PATH.read_text())


def test_goal_and_planner_use_precise_safe_approximation(nav2_params):
    controller = nav2_params['controller_server']['ros__parameters']
    planner = nav2_params['planner_server']['ros__parameters']
    bt = nav2_params['bt_navigator']['ros__parameters']

    assert controller['progress_checker']['required_movement_radius'] == 0.05
    assert controller['general_goal_checker']['xy_goal_tolerance'] == 0.05
    assert controller['general_goal_checker']['yaw_goal_tolerance'] == 0.10
    assert planner['expected_planner_frequency'] == 0.5
    assert planner['GridBased']['tolerance'] == 0.10
    assert planner['GridBased']['use_final_approach_orientation'] is False
    assert bt['default_nav_to_pose_bt_xml'] == 'navigate_to_pose_safe.xml'
    autonomy = (PACKAGE / 'launch' / 'autonomy.launch.py').read_text()
    assert "'default_nav_to_pose_bt_xml': PathJoinSubstitution" in autonomy
    assert '/home/' not in PARAMS_PATH.read_text()


def test_controller_uses_full_mecanum_motion_with_conservative_limits(
        nav2_params):
    controller = nav2_params['controller_server']['ros__parameters']['FollowPath']
    smoother = nav2_params['velocity_smoother']['ros__parameters']

    assert controller['motion_model'] == 'Omni'
    assert controller['vx_min'] == -0.23
    assert controller['vx_max'] == 0.23
    assert controller['vy_min'] == -0.23
    assert controller['vy_max'] == 0.23
    assert controller['ax_min'] == -0.35
    assert controller['ax_max'] == 0.35
    assert controller['ay_min'] == -0.35
    assert controller['ay_max'] == 0.35
    assert controller['CostCritic']['consider_footprint'] is True
    assert smoother['min_velocity'][:2] == [-0.23, -0.23]
    assert smoother['max_velocity'][:2] == [0.23, 0.23]
    assert smoother['max_accel'][:2] == [0.35, 0.35]
    assert smoother['max_decel'][:2] == [-0.35, -0.35]


def test_mppi_sampling_and_critics_match_slow_holonomic_base(nav2_params):
    controller = nav2_params['controller_server']['ros__parameters']['FollowPath']

    assert controller['vx_std'] == 0.08
    assert controller['vy_std'] == 0.08
    assert controller['wz_std'] == 0.30
    assert controller['wz_max'] == 0.60
    assert controller['regenerate_noises'] is False
    assert 'PathAngleCritic' not in controller['critics']
    assert 'TwirlingCritic' in controller['critics']
    assert controller['TwirlingCritic']['cost_weight'] == 2.0
    assert controller['GoalAngleCritic']['cost_weight'] == 10.0
    assert controller['GoalAngleCritic']['threshold_to_consider'] == 0.50
    assert controller['PathAlignCritic']['cost_weight'] == 10.0
    assert controller['PathAlignCritic']['threshold_to_consider'] == 0.35
    assert controller['GoalCritic']['threshold_to_consider'] == 0.45
    assert controller['PathFollowCritic']['threshold_to_consider'] == 0.45
    assert controller['PathAlignCritic']['offset_from_furthest'] == 3


def test_costmaps_use_measured_local_footprint_and_static_walls(nav2_params):
    local = nav2_params['local_costmap']['local_costmap']['ros__parameters']
    global_costmap = nav2_params['global_costmap'][
        'global_costmap']['ros__parameters']

    assert yaml.safe_load(local['footprint']) == [
        [0.119, 0.155], [0.119, -0.155],
        [-0.119, -0.155], [-0.119, 0.155],
    ]
    assert local['footprint_padding'] == 0.02
    assert local['plugins'] == [
        'static_layer', 'obstacle_layer', 'inflation_layer']
    assert local['inflation_layer'] == {
        'plugin': 'nav2_costmap_2d::InflationLayer',
        'cost_scaling_factor': 8.0,
        'inflation_radius': 0.25,
    }
    assert yaml.safe_load(global_costmap['footprint']) == [
        [0.119, 0.155], [0.119, -0.155],
        [-0.119, -0.155], [-0.119, 0.155],
    ]
    assert global_costmap['footprint_padding'] == 0.02
    assert 'robot_radius' not in global_costmap
    assert global_costmap['plugins'] == ['static_layer', 'inflation_layer']
    assert 'obstacle_layer' not in global_costmap
    assert global_costmap['inflation_layer'] == {
        'plugin': 'nav2_costmap_2d::InflationLayer',
        'cost_scaling_factor': 8.0,
        'inflation_radius': 0.25,
    }


def test_navigation_tree_has_only_contextual_clear_retries():
    root = ET.parse(BT_PATH).getroot()
    tags = {element.tag for element in root.iter()}

    assert tags.isdisjoint({'Spin', 'BackUp', 'DriveOnHeading', 'Wait'})
    assert root.find('.//RateController').attrib['hz'] == '0.5'
    assert len(root.findall('.//RecoveryNode')) == 2
    assert all(
        node.attrib['number_of_retries'] == '1'
        for node in root.findall('.//RecoveryNode'))
    assert len(root.findall('.//ClearEntireCostmap')) == 2
    assert root.find('.//GoalUpdated') is not None
