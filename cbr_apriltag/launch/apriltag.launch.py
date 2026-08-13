"""Launch AprilTag using the virtualenv that owns pupil_apriltags."""

import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def _default_python_executable() -> str:
    """Find the Python containing the optional pupil_apriltags wheel."""
    configured = os.environ.get('CBR_APRILTAG_PYTHON')
    if configured and Path(configured).is_file():
        return configured

    active_virtualenv = os.environ.get('VIRTUAL_ENV')
    if active_virtualenv:
        candidate = Path(active_virtualenv) / 'bin' / 'python'
        if candidate.is_file():
            return str(candidate)

    try:
        from ament_index_python.packages import get_package_share_directory

        share_directory = Path(
            get_package_share_directory('cbr_apriltag')).resolve()
        for parent in (share_directory, *share_directory.parents):
            candidate = parent / '.venv' / 'bin' / 'python'
            if candidate.is_file():
                return str(candidate)
    except Exception:
        pass

    return sys.executable


def generate_launch_description() -> LaunchDescription:
    config = PathJoinSubstitution([FindPackageShare('cbr_apriltag'), 'config', 'apriltag.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/image_rect'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument(
            'python_executable',
            default_value=_default_python_executable(),
            description='Python interpreter containing pupil_apriltags.'),
        Node(
            package='cbr_apriltag',
            executable='apriltag_detector',
            name='apriltag_detector',
            output='screen',
            prefix=LaunchConfiguration('python_executable'),
            parameters=[config, {
                'image_topic': LaunchConfiguration('image_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'base_frame': LaunchConfiguration('base_frame'),
            }],
        ),
    ])
