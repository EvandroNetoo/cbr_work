"""Offline validation of the MoveIt configuration files."""
import os
import xml.etree.ElementTree as ET

import pytest
import yaml

from so_arm_101_description.limits import load_joint_limits


BASE_DIR = os.path.join(os.path.dirname(__file__), '..')
CONFIG_DIR = os.path.join(BASE_DIR, 'config')
URDF_PATH = os.path.join(
    BASE_DIR, '..', 'so_arm_101_description', 'urdf',
    'so_101.urdf.xacro')


def test_srdf_matches_urdf_root_arm_chain_and_tcp():
    root = ET.parse(os.path.join(CONFIG_DIR, 'so_arm_101.srdf')).getroot()
    for element in root.iter():
        for key, value in element.attrib.items():
            element.attrib[key] = value.replace(
                '$(arg arm_base_link_name)', 'arm_base_link')
    urdf = ET.parse(URDF_PATH).getroot().find(
        '{http://www.ros.org/wiki/xacro}macro[@name="so_101_arm"]')
    for element in urdf.iter():
        for key, value in element.attrib.items():
            element.attrib[key] = value.replace(
                '${arm_base_link_name}', 'arm_base_link')
    chain = root.find("group[@name='arm']/chain")
    world_joint = urdf.find(".//joint[@name='world_to_base']")
    assert root.find('virtual_joint') is None
    assert world_joint.attrib['type'] == 'fixed'
    assert world_joint.find('parent').attrib['link'] == 'world'
    assert world_joint.find('child').attrib['link'] == 'arm_base_link'
    assert chain.attrib == {
        'base_link': 'arm_base_link', 'tip_link': 'gripper_tcp'}
    tcp_joint = urdf.find(".//joint[@name='link5_to_gripper_tcp']")
    assert tcp_joint.attrib['type'] == 'fixed'
    assert tcp_joint.find('parent').attrib['link'] == 'link5_1'
    assert tcp_joint.find('child').attrib['link'] == 'gripper_tcp'
    assert tcp_joint.find('origin').attrib == {
        'xyz': '0 -0.10 0', 'rpy': '0 0 0'}


def test_moveit_controllers_match_ros2_control():
    path = os.path.join(CONFIG_DIR, 'moveit_controllers.yaml')
    data = yaml.safe_load(open(path))
    controllers = data['moveit_simple_controller_manager']
    assert controllers['controller_names'] == ['arm_controller', 'gripper_controller']
    assert controllers['arm_controller']['type'] == 'FollowJointTrajectory'
    assert controllers['arm_controller']['joints'] == [
        'base_link_to_link1', 'link1_to_link2', 'link2_to_link3',
        'link3_to_link4', 'link4_to_link5',
    ]
    assert controllers['gripper_controller']['type'] == 'FollowJointTrajectory'
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


def test_kinematics_uses_position_only_kdl_for_five_dof_arm():
    data = yaml.safe_load(open(os.path.join(CONFIG_DIR, 'kinematics.yaml')))
    assert data['arm']['kinematics_solver'] == (
        'kdl_kinematics_plugin/KDLKinematicsPlugin')
    assert data['arm']['position_only_ik'] is True


def test_gripper_named_states_match_visual_motion():
    root = ET.parse(os.path.join(CONFIG_DIR, 'so_arm_101.srdf')).getroot()
    states = {}
    for state in root.findall("group_state[@group='gripper']"):
        states[state.attrib['name']] = float(
            state.find("joint[@name='right_clamp']").attrib['value'])
    assert states['open'] == 0.037
    assert states['pre_grip'] == 0.037
    assert states['closed'] == 0.0


def test_pick_cube_left_named_state_uses_requested_joint_values():
    root = ET.parse(os.path.join(CONFIG_DIR, 'so_arm_101.srdf')).getroot()
    state = root.find("group_state[@name='pick_cube_left'][@group='arm']")
    assert state is not None
    values = [float(joint.attrib['value']) for joint in state.findall('joint')]
    assert values == pytest.approx([
        1.7976891296,
        0.7504915784,
        -0.9250245036,
        -1.5184364492,
        1.3788101091,
    ])


@pytest.mark.parametrize(
    ('state_name', 'expected'),
    (
        (
            'deposit_cube_right',
            [-1.6580627894, 0.6981317008, -0.6632251158,
             -1.7627825445, 1.6580627894],
        ),
        (
            'pick_cube_right',
            [-1.7453292520, 0.7504915784, -0.9250245036,
             -1.5184364492, 1.6580627894],
        ),
    ),
)
def test_right_cargo_named_states_use_measured_joint_values(state_name, expected):
    root = ET.parse(os.path.join(CONFIG_DIR, 'so_arm_101.srdf')).getroot()
    state = root.find(f"group_state[@name='{state_name}'][@group='arm']")
    assert state is not None
    values = [float(joint.attrib['value']) for joint in state.findall('joint')]
    assert values == pytest.approx(expected)


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
    model_limits = load_joint_limits()
    assert right['min_position'] == -1e-6
    assert right['max_position'] == 0.037001
    assert left['min_position'] == -0.037001
    assert left['max_position'] == 1e-6
    assert model_limits['right_clamp']['min_position'] == 0.0
    assert model_limits['right_clamp']['max_position'] == 0.037
    assert model_limits['right_clamp']['max_velocity'] == 0.05
    assert model_limits['left_clamp']['min_position'] == -0.037
    assert model_limits['left_clamp']['max_position'] == 0.0
