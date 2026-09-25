"""Launch unified scene analysis using the pupil_apriltags virtualenv."""

import os
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _default_python_executable() -> str:
    """Find the Python containing the optional pupil_apriltags wheel."""
    configured = os.environ.get('CBR_VISION_PYTHON')
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
            get_package_share_directory('vision')).resolve()
        for parent in (share_directory, *share_directory.parents):
            candidate = parent / '.venv' / 'bin' / 'python'
            if candidate.is_file():
                return str(candidate)
    except Exception:
        pass

    return sys.executable


def generate_launch_description() -> LaunchDescription:
    config = PathJoinSubstitution([
        FindPackageShare('vision'), 'config', 'vision.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_rect'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('simulation', default_value='false'),
        DeclareLaunchArgument(
            'python_executable',
            default_value=_default_python_executable(),
            description='Python interpreter containing pupil_apriltags.'),
        Node(
            package='vision',
            executable='scene_analyzer',
            name='scene_analyzer',
            output='screen',
            prefix=LaunchConfiguration('python_executable'),
            parameters=[LaunchConfiguration('config_file'), {
                'image_topic': LaunchConfiguration('image_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'base_frame': LaunchConfiguration('base_frame'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'manage_camera_capture': ParameterValue(
                    PythonExpression(["'", LaunchConfiguration('simulation'),
                                      "' == 'false'"]), value_type=bool),
                'manage_vision_led': ParameterValue(
                    PythonExpression(["'", LaunchConfiguration('simulation'),
                                      "' == 'false'"]), value_type=bool),
            }],
        ),
    ])
