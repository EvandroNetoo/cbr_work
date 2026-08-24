"""Embedded camera, rectification and AprilTag perception pipeline."""

import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _apriltag_python():
    configured = os.environ.get('CBR_APRILTAG_PYTHON')
    if configured and Path(configured).is_file():
        return configured
    if os.environ.get('VIRTUAL_ENV'):
        candidate = Path(os.environ['VIRTUAL_ENV']) / 'bin' / 'python'
        if candidate.is_file():
            return str(candidate)
    return sys.executable


def _critical(node, label):
    return RegisterEventHandler(OnProcessExit(
        target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason=f'{label} encerrou.'))]))


def generate_launch_description():
    enabled = IfCondition(LaunchConfiguration('enable_perception'))
    camera = Node(
        package='usb_cam', executable='usb_cam_node_exe', namespace='camera',
        name='driver', output='screen', condition=enabled,
        parameters=[
            PathJoinSubstitution([FindPackageShare('camera'), 'config', 'camera.yaml']),
            {'framerate': ParameterValue(
                LaunchConfiguration('camera_framerate'), value_type=float)}])
    rectify = Node(
        package='image_proc', executable='rectify_node', namespace='camera',
        name='rectify', output='screen', condition=enabled,
        remappings=[('image', 'image_raw')])
    detector = Node(
        package='apriltag', executable='apriltag_detector', name='apriltag_detector',
        output='screen', condition=enabled, prefix=_apriltag_python(),
        parameters=[
            PathJoinSubstitution([FindPackageShare('apriltag'), 'config', 'apriltag.yaml']),
            {'image_topic': '/camera/image_rect',
             'camera_info_topic': '/camera/camera_info', 'base_frame': 'base_link'}])
    return LaunchDescription([
        DeclareLaunchArgument('enable_perception', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('camera_framerate', default_value='15.0'),
        camera, rectify, detector,
        _critical(camera, 'Câmera'), _critical(rectify, 'Retificação'),
        _critical(detector, 'Detector AprilTag')])
