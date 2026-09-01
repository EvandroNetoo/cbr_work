"""Start the semantic manipulation action server."""

import os
import sys

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _python_executable() -> str:
    virtualenv = os.environ.get('VIRTUAL_ENV')
    if virtualenv:
        candidate = os.path.join(virtualenv, 'bin', 'python')
        if os.path.isfile(candidate):
            return candidate
    return sys.executable


def generate_launch_description() -> LaunchDescription:
    config = PathJoinSubstitution([
        FindPackageShare('manipulation'), 'config', 'manipulation.yaml'
    ])
    profiles = PathJoinSubstitution([
        FindPackageShare('manipulation'), 'config', 'profiles.yaml'
    ])
    cargo = PathJoinSubstitution([
        FindPackageShare('manipulation'), 'config', 'cargo_slots.yaml'
    ])
    return LaunchDescription([
        Node(
            package='manipulation',
            executable='manipulation_server',
            name='manipulation_server',
            output='screen',
            prefix=_python_executable(),
            parameters=[config, {
                'profiles_file': profiles,
                'cargo_slots_file': cargo,
            }],
        ),
    ])
