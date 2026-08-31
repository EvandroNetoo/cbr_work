"""Contracts for the low-participant Nav2 launch."""

from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
NAVIGATION_PARAMS = PACKAGE_ROOT / 'config' / 'nav2_navigation_light.yaml'


def test_navigation_is_composed_and_excludes_unused_incompatible_servers():
    source = (PACKAGE_ROOT / 'launch' / 'navigation.launch.py').read_text()

    assert "executable='component_container_isolated'" in source
    assert "plugin='nav2_controller::ControllerServer'" in source
    assert "plugin='nav2_planner::PlannerServer'" in source
    assert "plugin='nav2_bt_navigator::BtNavigator'" in source
    assert "plugin='nav2_velocity_smoother::VelocitySmoother'" in source
    assert "package='nav2_collision_monitor'" in source
    assert "executable='collision_monitor'" in source
    assert "package='nav2_lifecycle_manager'" in source
    assert "executable='lifecycle_manager'" in source
    assert 'opennav_docking' not in source
    assert 'nav2_route::RouteServer' not in source
    assert 'nav2_waypoint_follower::WaypointFollower' not in source
    assert 'behavior_server::BehaviorServer' not in source


def test_navigation_preserves_stamped_command_chain_and_tf_topics():
    source = (PACKAGE_ROOT / 'launch' / 'navigation.launch.py').read_text()

    assert "('/tf', 'tf')" in source
    assert "('/tf_static', 'tf_static')" in source
    assert "('cmd_vel', 'cmd_vel_nav')" in source
    assert 'parameters=node_parameters' in source


def test_navigation_has_installed_defaults_and_no_absolute_yaml_paths():
    source = (PACKAGE_ROOT / 'launch' / 'navigation.launch.py').read_text()
    params_text = NAVIGATION_PARAMS.read_text()
    params = yaml.safe_load(params_text)

    assert "'config', 'nav2_navigation_light.yaml'" in source
    assert "'config', 'navigate_to_pose_safe.xml'" in source
    assert 'default_value=default_params_file' in source
    assert "'default_nav_to_pose_bt_xml': navigate_to_pose_bt" in source
    assert 'default_nav_to_pose_bt_xml' not in params['bt_navigator']['ros__parameters']
    assert '/home/' not in params_text


def test_navigation_expands_local_fastdds_initial_peer_range():
    source = (PACKAGE_ROOT / 'launch' / 'navigation.launch.py').read_text()
    profile = (PACKAGE_ROOT / 'config' / 'fastdds_nav2.xml').read_text()

    assert "SetEnvironmentVariable(\n            'FASTRTPS_DEFAULT_PROFILES_FILE'" in source
    assert "SetEnvironmentVariable(\n            'FASTDDS_DEFAULT_PROFILES_FILE'" in source
    assert '<maxInitialPeersRange>64</maxInitialPeersRange>' in profile
    assert '<address>127.0.0.1</address>' in profile
    assert '<transport_id>cbr_nav2_shm</transport_id>' in profile


def test_navigation_profile_only_loads_the_safe_pose_navigator():
    params = yaml.safe_load(NAVIGATION_PARAMS.read_text())
    bt = params['bt_navigator']['ros__parameters']

    assert bt['navigators'] == ['navigate_to_pose']
    assert 'navigate_through_poses' not in bt


def test_navigation_denoises_obstacles_before_inflation():
    params = yaml.safe_load(NAVIGATION_PARAMS.read_text())

    for costmap_name in ('local_costmap', 'global_costmap'):
        costmap = params[costmap_name][costmap_name]['ros__parameters']

        assert costmap['plugins'] == [
            'static_layer',
            'obstacle_layer',
            'denoise_layer',
            'inflation_layer',
        ]
        assert costmap['denoise_layer'] == {
            'plugin': 'nav2_costmap_2d::DenoiseLayer',
            'enabled': True,
            'minimal_group_size': 3,
            'group_connectivity_type': 8,
        }


def test_light_profile_reduces_controller_and_costmap_load():
    params = yaml.safe_load(NAVIGATION_PARAMS.read_text())
    controller = params['controller_server']['ros__parameters']
    mppi = controller['FollowPath']
    local = params['local_costmap']['local_costmap']['ros__parameters']
    global_costmap = params['global_costmap']['global_costmap']['ros__parameters']

    assert controller['controller_frequency'] == 10.0
    assert mppi['batch_size'] == 800
    assert mppi['time_steps'] == 40
    assert local['resolution'] == 0.025
    assert local['publish_frequency'] == 1.0
    assert global_costmap['update_frequency'] == 1.0
    assert global_costmap['publish_frequency'] == 1.0
