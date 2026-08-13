"""Validate URDF structure without requiring ROS2."""
import os
import xml.etree.ElementTree as ET

import pytest
from so_arm_101_description.limits import load_joint_limits

URDF_PATH = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'so_101.urdf.xacro')
CAMERA_MODULE_PATH = os.path.join(
    os.path.dirname(__file__), '..', 'urdf', 'camera_mount.xacro'
)

EXPECTED_LINKS = [
    'base_link', 'link1_1', 'link2_1', 'link3_1',
    'link4_1', 'link5_1', 'clamp_1', 'clamp_2',
]

EXPECTED_JOINTS = {
    'base_link_to_link1': ('base_link', 'link1_1', 'revolute'),
    'link1_to_link2': ('link1_1', 'link2_1', 'revolute'),
    'link2_to_link3': ('link2_1', 'link3_1', 'revolute'),
    'link3_to_link4': ('link3_1', 'link4_1', 'revolute'),
    'link4_to_link5': ('link4_1', 'link5_1', 'revolute'),
    'right_clamp': ('link5_1', 'clamp_1', 'prismatic'),
    'left_clamp': ('link5_1', 'clamp_2', 'prismatic'),
}


@pytest.fixture(scope='module')
def urdf_root():
    tree = ET.parse(URDF_PATH)
    return tree.getroot()


@pytest.fixture(scope='module')
def model_limits():
    return load_joint_limits()


def test_xml_wellformed(urdf_root):
    assert urdf_root.tag == 'robot'
    assert urdf_root.attrib['name'] == 'so_101'


def test_all_links_present(urdf_root):
    links = {el.attrib['name'] for el in urdf_root.findall('link')}
    for name in EXPECTED_LINKS:
        assert name in links, f"Missing link: {name}"


def test_all_joints_present(urdf_root):
    joints = {el.attrib['name'] for el in urdf_root.findall('joint')}
    for name in EXPECTED_JOINTS:
        assert name in joints, f"Missing joint: {name}"


@pytest.mark.parametrize('joint_name,expected', EXPECTED_JOINTS.items())
def test_joint_parent_child(urdf_root, joint_name, expected):
    parent_link, child_link, joint_type = expected
    joint_el = None
    for el in urdf_root.findall('joint'):
        if el.attrib['name'] == joint_name:
            joint_el = el
            break
    assert joint_el is not None
    assert joint_el.attrib['type'] == joint_type
    assert joint_el.find('parent').attrib['link'] == parent_link
    assert joint_el.find('child').attrib['link'] == child_link


@pytest.mark.parametrize('link_name', EXPECTED_LINKS)
def test_link_has_inertial_visual_collision(urdf_root, link_name):
    link_el = None
    for el in urdf_root.findall('link'):
        if el.attrib['name'] == link_name:
            link_el = el
            break
    assert link_el is not None
    assert link_el.find('inertial') is not None, f"{link_name} missing inertial"
    assert link_el.find('visual') is not None, f"{link_name} missing visual"
    if link_name not in {'clamp_1', 'clamp_2'}:
        assert link_el.find('collision') is not None, (
            f'{link_name} missing collision'
        )


def test_camera_mount_is_visual_only_and_uses_measured_pose():
    module_root = ET.parse(CAMERA_MODULE_PATH).getroot()
    camera = module_root.find(".//link[@name='camera_link']")
    joint = module_root.find(
        ".//joint[@name='${parent_link}_to_camera_link']")
    assert camera is not None
    assert camera.find('collision') is None
    assert camera.find("visual/geometry/box").attrib['size'] == (
        '0.0635 0.0026 0.0087')
    assert joint is not None
    assert joint.find('origin').attrib['xyz'] == '-0.0105 -0.0435 -0.016'


def test_camera_is_attached_to_link4(urdf_root):
    camera_call = urdf_root.find(
        '{http://www.ros.org/wiki/xacro}cbr_camera_mount')
    assert camera_call is not None
    assert camera_call.attrib['parent_link'] == 'link4_1'


def test_left_gripper_joint_mimics_the_single_actuator(urdf_root, model_limits):
    """The passive finger must follow the one physical actuator."""
    joints = {
        el.attrib['name']: el for el in urdf_root.findall('joint')
    }
    right_clamp = joints['right_clamp']
    left_clamp = joints['left_clamp']

    assert left_clamp.find('axis').attrib['xyz'] == (
        right_clamp.find('axis').attrib['xyz']
    )
    assert model_limits['left_clamp']['min_position'] == -0.037
    assert model_limits['left_clamp']['max_position'] == 0.0

    for el in urdf_root.findall('joint'):
        if el.attrib['name'] == 'left_clamp':
            mimic = el.find('mimic')
            assert mimic is not None
            assert mimic.attrib['joint'] == 'right_clamp'
            assert float(mimic.attrib.get('multiplier', '1')) == -1.0
            return
    pytest.fail('left_clamp joint not found')


def test_gripper_velocity_is_visually_smooth(urdf_root, model_limits):
    """A 5 mm teleop step should take about 100 ms, not one sim frame."""
    joints = {
        el.attrib['name']: el for el in urdf_root.findall('joint')
    }
    for name in ('right_clamp', 'left_clamp'):
        assert model_limits[name]['max_velocity'] == pytest.approx(0.05)


def test_all_joints_have_dynamics(urdf_root):
    for el in urdf_root.findall('joint'):
        if el.attrib['type'] == 'fixed':
            continue
        name = el.attrib['name']
        assert el.find('dynamics') is not None, f"{name} missing dynamics"


def test_revolute_joints_have_safety_controller(urdf_root):
    for el in urdf_root.findall('joint'):
        if el.attrib['type'] == 'revolute':
            name = el.attrib['name']
            assert el.find('safety_controller') is not None, \
                f"{name} missing safety_controller"


def test_visual_mesh_files_exist(urdf_root):
    base = os.path.join(os.path.dirname(__file__), '..')
    for link_el in urdf_root.findall('link'):
        vis = link_el.find('visual')
        if vis is None:
            continue
        mesh = vis.find('.//mesh')
        if mesh is None:
            continue
        filename = mesh.attrib['filename']
        # package://so_arm_101_description/meshes/visual/X.stl -> meshes/visual/X.stl
        rel = filename.replace('package://so_arm_101_description/', '')
        path = os.path.join(base, rel)
        assert os.path.exists(path), f"Missing mesh: {path}"


def test_collision_mesh_files_exist(urdf_root):
    base = os.path.join(os.path.dirname(__file__), '..')
    for link_el in urdf_root.findall('link'):
        col = link_el.find('collision')
        if col is None:
            continue
        mesh = col.find('.//mesh')
        if mesh is None:
            continue
        filename = mesh.attrib['filename']
        rel = filename.replace('package://so_arm_101_description/', '')
        path = os.path.join(base, rel)
        assert os.path.exists(path), f"Missing mesh: {path}"
