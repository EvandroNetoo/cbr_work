from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro


@pytest.fixture(scope='module')
def robot():
    path = Path(__file__).parents[1] / 'urdf' / 'cbr_robot.urdf.xacro'
    return ET.fromstring(xacro.process_file(str(path)).toxml())


def test_composed_tree_has_unique_names_and_two_hardware_systems(robot):
    links = [element.attrib['name'] for element in robot.findall('link')]
    joints = [element.attrib['name'] for element in robot.findall('joint')]
    assert len(links) == len(set(links))
    assert len(joints) == len(set(joints))
    systems = robot.findall('ros2_control')
    assert [system.attrib['name'] for system in systems] == [
        'CBRMecanumSystem', 'so_101_system']


def test_arm_mount_and_base_link_rename(robot):
    mount = robot.find("joint[@name='mobile_base_to_arm']")
    assert mount.find('parent').attrib['link'] == 'upper_platform_link'
    assert mount.find('child').attrib['link'] == 'arm_base_link'
    assert mount.find('origin').attrib == {
        'xyz': '0.075 0.0 0.004', 'rpy': '0.0 0.0 1.57079632679'}
    assert robot.find("link[@name='arm_base_link']") is not None


def test_physical_plugins_and_ten_commanded_joints(robot):
    plugins = [
        system.find('hardware/plugin').text.strip()
        for system in robot.findall('ros2_control')]
    assert plugins == [
        'cbr_base_hardware_interface/MariolaSystem',
        'so_arm_101_hardware_interface/SO101System']
    controlled = {
        joint.attrib['name']
        for system in robot.findall('ros2_control')
        for joint in system.findall('joint')
    }
    assert len(controlled) == 10
    assert {
        'front_left_wheel_joint', 'front_right_wheel_joint',
        'rear_left_wheel_joint', 'rear_right_wheel_joint',
    }.issubset(controlled)


def test_imu_data_frame_is_aligned_with_robot_axes(robot):
    imu_joint = robot.find("joint[@name='imu_joint']")
    assert imu_joint.find('parent').attrib['link'] == 'base_link'
    assert imu_joint.find('child').attrib['link'] == 'imu_link'
    assert imu_joint.find('origin').attrib['rpy'] == '0.0 0.0 0.0'
