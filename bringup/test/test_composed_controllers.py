from pathlib import Path

import yaml


def test_one_joint_state_broadcaster_contains_arm_and_wheels():
    package = Path(__file__).parents[1]
    controllers = yaml.safe_load(
        (package / 'config' / 'controllers.yaml').read_text())
    manager = controllers['controller_manager']['ros__parameters']
    assert manager['update_rate'] == 30
    assert manager['joint_state_broadcaster']['type'] == (
        'joint_state_broadcaster/JointStateBroadcaster')
    assert sum(
        isinstance(value, dict)
        and value.get('type') == 'joint_state_broadcaster/JointStateBroadcaster'
        for value in manager.values()) == 1
    joints = controllers['joint_state_broadcaster']['ros__parameters']['joints']
    assert len(joints) == 10
    assert len(set(joints)) == 10
    assert controllers['arm_controller']['ros__parameters'][
        'state_publish_rate'] == 30.0
    assert controllers['gripper_controller']['ros__parameters'][
        'state_publish_rate'] == 30.0


def test_composed_mecanum_publishes_expected_frames():
    package = Path(__file__).parents[1]
    controllers = yaml.safe_load(
        (package / 'config' / 'controllers.yaml').read_text())
    base = controllers['base_controller']['ros__parameters']
    assert base['base_frame_id'] == 'base_footprint'
    assert base['odom_frame_id'] == 'odom'
    assert base['enable_odom_tf'] is False
