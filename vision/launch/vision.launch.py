import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _python_executable():
    configured = os.environ.get('CBR_VISION_PYTHON')
    if configured and Path(configured).is_file():
        return configured
    virtualenv = os.environ.get('VIRTUAL_ENV')
    if virtualenv and (Path(virtualenv) / 'bin/python').is_file():
        return str(Path(virtualenv) / 'bin/python')
    try:
        share = Path(__import__(
            'ament_index_python.packages', fromlist=['get_package_share_directory']
        ).get_package_share_directory('vision')).resolve()
        for parent in (share, *share.parents):
            candidate = parent / '.venv' / 'bin' / 'python'
            if candidate.is_file():
                return str(candidate)
    except Exception:
        pass
    return sys.executable


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('vision'), 'config', 'vision.yaml'])
    geometry = PathJoinSubstitution([
        FindPackageShare('vision'), 'config', 'container_geometry_profiles.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/image_rect'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('output_frame', default_value='arm_base_link'),
        DeclareLaunchArgument(
            'python_executable', default_value=_python_executable(),
            description='Python interpreter containing pupil_apriltags.'),
        Node(
            package='vision', executable='vision', name='vision', output='screen',
            prefix=LaunchConfiguration('python_executable'),
            parameters=[config, {
                'geometry_profiles_file': geometry,
                'image_topic': LaunchConfiguration('image_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'output_frame': LaunchConfiguration('output_frame'),
            }],
        ),
    ])
