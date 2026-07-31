"""Validate controller configuration against URDF joints."""
import os
import xml.etree.ElementTree as ET

import pytest
import yaml

BASE_DIR = os.path.join(os.path.dirname(__file__), '..')


@pytest.fixture(scope='module')
def controllers():
    with open(os.path.join(BASE_DIR, 'config', 'controllers.yaml')) as f:
        return yaml.safe_load(f)


@pytest.fixture(scope='module')
def urdf_joints():
    tree = ET.parse(os.path.join(BASE_DIR, 'urdf', 'so_101.urdf.xacro'))
    root = tree.getroot()
    return {el.attrib['name'] for el in root.findall('joint')}


def test_arm_controller_joints_exist_in_urdf(controllers, urdf_joints):
    arm_joints = controllers['arm_controller']['ros__parameters']['joints']
    for joint in arm_joints:
        assert joint in urdf_joints, f"arm_controller joint '{joint}' not in URDF"


def test_gripper_controller_joints_exist_in_urdf(controllers, urdf_joints):
    gripper_joints = controllers['gripper_controller']['ros__parameters']['joints']
    for joint in gripper_joints:
        assert joint in urdf_joints, f"gripper_controller joint '{joint}' not in URDF"


def test_controller_types_valid(controllers):
    cm = controllers['controller_manager']['ros__parameters']
    valid_types = {
        'joint_state_broadcaster/JointStateBroadcaster',
        'joint_trajectory_controller/JointTrajectoryController',
        'position_controllers/JointGroupPositionController',
        'forward_command_controller/ForwardCommandController',
        'velocity_controllers/JointGroupVelocityController',
        'effort_controllers/JointGroupEffortController',
    }
    for ctrl_name in ['joint_state_broadcaster', 'arm_controller', 'gripper_controller']:
        ctrl_type = cm[ctrl_name]['type']
        assert ctrl_type in valid_types, f"Unknown controller type: {ctrl_type}"


def test_arm_controller_has_5_joints(controllers):
    joints = controllers['arm_controller']['ros__parameters']['joints']
    assert len(joints) == 5, f"Expected 5 arm joints, got {len(joints)}"


def test_gripper_controller_commands_only_actuated_joint(controllers):
    joints = controllers['gripper_controller']['ros__parameters']['joints']
    assert joints == ['right_clamp']
