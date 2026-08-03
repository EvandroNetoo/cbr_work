"""Offline validation of the MoveIt configuration files."""
import os
import xml.etree.ElementTree as ET

import yaml


BASE_DIR = os.path.join(os.path.dirname(__file__), '..')
CONFIG_DIR = os.path.join(BASE_DIR, 'config')
URDF_PATH = os.path.join(
    BASE_DIR, '..', 'so_arm_101_description', 'urdf',
    'so_101.urdf.xacro')


def test_srdf_matches_urdf_root_and_arm_chain():
    root = ET.parse(os.path.join(CONFIG_DIR, 'so_arm_101.srdf')).getroot()
    urdf = ET.parse(URDF_PATH).getroot()
    chain = root.find("group[@name='arm']/chain")
    world_joint = urdf.find("joint[@name='world_to_base']")
    assert root.find('virtual_joint') is None
    assert world_joint.attrib['type'] == 'fixed'
    assert world_joint.find('parent').attrib['link'] == 'world'
    assert world_joint.find('child').attrib['link'] == 'base_link'
    assert chain.attrib == {
        'base_link': 'base_link', 'tip_link': 'link5_1'}


def test_moveit_controllers_match_ros2_control():
    path = os.path.join(CONFIG_DIR, 'moveit_controllers.yaml')
    data = yaml.safe_load(open(path))
    controllers = data['moveit_simple_controller_manager']
    assert controllers['controller_names'] == [
        'arm_controller', 'gripper_controller']
    assert controllers['arm_controller']['type'] == 'FollowJointTrajectory'
    assert controllers['arm_controller']['joints'] == [
        'base_link_to_link1', 'link1_to_link2', 'link2_to_link3',
        'link3_to_link4', 'link4_to_link5',
    ]
    assert controllers['gripper_controller']['type'] == (
        'FollowJointTrajectory')
    assert controllers['gripper_controller']['joints'] == ['right_clamp']


def test_all_arm_joints_have_acceleration_overrides():
    data = yaml.safe_load(open(os.path.join(CONFIG_DIR, 'joint_limits.yaml')))
    limits = data['joint_limits']
    for name in (
        'base_link_to_link1', 'link1_to_link2', 'link2_to_link3',
        'link3_to_link4', 'link4_to_link5',
    ):
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


def test_gripper_group_contains_actuated_and_mimic_joints():
    root = ET.parse(os.path.join(CONFIG_DIR, 'so_arm_101.srdf')).getroot()
    joints = root.findall("group[@name='gripper']/joint")
    assert [joint.attrib['name'] for joint in joints] == [
        'right_clamp', 'left_clamp']

    end_effector = root.find("end_effector[@name='gripper']")
    assert end_effector.attrib['parent_group'] == 'arm'


def test_gripper_planning_limits_only_add_numeric_tolerance():
    data = yaml.safe_load(open(os.path.join(CONFIG_DIR, 'joint_limits.yaml')))
    right = data['joint_limits']['right_clamp']
    left = data['joint_limits']['left_clamp']
    urdf = ET.parse(URDF_PATH).getroot()
    right_urdf_limit = urdf.find(
        "joint[@name='right_clamp']/limit").attrib
    left_urdf_limit = urdf.find(
        "joint[@name='left_clamp']/limit").attrib

    assert right['min_position'] == -1e-6
    assert right['max_position'] == 0.037001
    assert left['min_position'] == -0.037001
    assert left['max_position'] == 1e-6

    assert float(right_urdf_limit['lower']) == 0.0
    assert float(right_urdf_limit['upper']) == 0.037
    assert float(right_urdf_limit['velocity']) == 0.05
    assert float(left_urdf_limit['lower']) == -0.037
    assert float(left_urdf_limit['upper']) == 0.0
