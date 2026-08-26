"""Contracts for the low-participant Nav2 launch."""

from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[2]


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


def test_navigation_expands_local_fastdds_initial_peer_range():
    source = (PACKAGE_ROOT / 'launch' / 'navigation.launch.py').read_text()
    profile = (PACKAGE_ROOT / 'config' / 'fastdds_nav2.xml').read_text()

    assert "SetEnvironmentVariable(\n            'FASTRTPS_DEFAULT_PROFILES_FILE'" in source
    assert "SetEnvironmentVariable(\n            'FASTDDS_DEFAULT_PROFILES_FILE'" in source
    assert '<maxInitialPeersRange>64</maxInitialPeersRange>' in profile
    assert '<address>127.0.0.1</address>' in profile
    assert '<transport_id>cbr_nav2_shm</transport_id>' in profile


def test_navigation_profile_only_loads_the_safe_pose_navigator():
    params = yaml.safe_load(
        (WORKSPACE_ROOT / 'tools' / 'nav2_navigation.yaml').read_text())
    bt = params['bt_navigator']['ros__parameters']

    assert bt['navigators'] == ['navigate_to_pose']
    assert 'navigate_through_poses' not in bt
