"""Offline validation of the MoveIt configuration files."""
import os
import xml.etree.ElementTree as ET

import yaml


BASE_DIR = os.path.join(os.path.dirname(__file__), '..')
CONFIG_DIR = os.path.join(BASE_DIR, 'config')


def test_srdf_has_arm_chain_and_virtual_joint():
    root = ET.parse(os.path.join(CONFIG_DIR, 'so_arm_101.srdf')).getroot()
    virtual = root.find("virtual_joint[@name='world_joint']")
    chain = root.find("group[@name='arm']/chain")
    assert virtual.attrib['child_link'] == 'base_link'
    assert chain.attrib == {
        'base_link': 'base_link', 'tip_link': 'link5_1'}


def test_moveit_arm_controller_matches_ros2_control():
    path = os.path.join(CONFIG_DIR, 'moveit_controllers.yaml')
    data = yaml.safe_load(open(path))
    controller = data['moveit_simple_controller_manager']['arm_controller']
    assert controller['type'] == 'FollowJointTrajectory'
    assert controller['joints'] == [
        'base_link_to_link1', 'link1_to_link2', 'link2_to_link3',
        'link3_to_link4', 'link4_to_link5',
    ]


def test_all_arm_joints_have_moveit_limits():
    data = yaml.safe_load(open(os.path.join(CONFIG_DIR, 'joint_limits.yaml')))
    limits = data['joint_limits']
    for name in (
        'base_link_to_link1', 'link1_to_link2', 'link2_to_link3',
        'link3_to_link4', 'link4_to_link5',
    ):
        assert limits[name]['has_velocity_limits'] is True
        assert limits[name]['max_velocity'] > 0.0
        assert limits[name]['has_acceleration_limits'] is True
        assert limits[name]['max_acceleration'] > 0.0


def test_kinematics_uses_jazzy_solver_key():
    data = yaml.safe_load(open(os.path.join(CONFIG_DIR, 'kinematics.yaml')))
    assert data['arm']['kinematics_solver'] == (
        'kdl_kinematics_plugin/KDLKinematicsPlugin')


def test_gripper_named_states_match_visual_motion():
    root = ET.parse(os.path.join(CONFIG_DIR, 'so_arm_101.srdf')).getroot()
    states = {}
    for state in root.findall("group_state[@group='gripper']"):
        states[state.attrib['name']] = float(
            state.find("joint[@name='right_clamp']").attrib['value'])
    assert states['open'] == 0.037
    assert states['closed'] == 0.0


def test_gripper_limit_tolerates_only_numeric_noise():
    data = yaml.safe_load(open(os.path.join(CONFIG_DIR, 'joint_limits.yaml')))
    gripper = data['joint_limits']['right_clamp']
    assert gripper['min_position'] == -1e-6
    assert gripper['max_position'] == 0.037
