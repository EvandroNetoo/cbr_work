from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).parents[1]


def test_camera_configuration_is_explicit_and_uses_matching_calibration():
    config = yaml.safe_load((PACKAGE / 'config' / 'camera.yaml').read_text())
    parameters = config['/**']['ros__parameters']
    calibration = yaml.safe_load(
        (PACKAGE / 'config' / 'camera_info.yaml').read_text())
    assert parameters['video_device'].startswith('/dev/video')
    assert parameters['image_width'] > 0
    assert parameters['image_height'] > 0
    assert parameters['framerate'] == 15.0
    assert parameters['frame_id'] == 'camera_optical_frame'
    assert parameters['camera_info_url'] == (
        'package://camera/config/camera_info.yaml')
    assert calibration['camera_name'] == parameters['camera_name']
    assert calibration['image_width'] == parameters['image_width']
    assert calibration['image_height'] == parameters['image_height']
    assert calibration['camera_matrix']['data'][0] > 0.0
    assert calibration['camera_matrix']['data'][4] > 0.0


def test_camera_launch_does_not_start_apriltag_or_robot():
    source = (PACKAGE / 'launch' / 'camera.launch.py').read_text()
    assert "package='usb_cam'" in source
    assert "executable='rectify_node'" in source
    assert "default_value='false'" in source
    assert "'framerate', default_value='15.0'" in source
    assert "LaunchConfiguration('framerate')" in source
    assert 'apriltag' not in source.lower()
    assert 'moveit' not in source.lower()


def test_calibration_target_has_expected_physical_size():
    target = PACKAGE / 'calibration' / 'checkerboard_8x6_25mm_a4.svg'
    root = ET.parse(target).getroot()
    assert root.attrib['width'] == '297mm'
    assert root.attrib['height'] == '210mm'
    assert '225 by 175 mm' in root.find('{http://www.w3.org/2000/svg}desc').text
