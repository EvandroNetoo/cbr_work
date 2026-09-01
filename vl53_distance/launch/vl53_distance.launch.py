"""Inicia o servidor da action de distância com sua configuração física."""

import os
import sys

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _python_executable():
    virtualenv = os.environ.get('VIRTUAL_ENV')
    if virtualenv:
        candidate = os.path.join(virtualenv, 'bin', 'python')
        if os.path.isfile(candidate):
            return candidate
    return sys.executable


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('vl53_distance'), 'config', 'vl53_distance.yaml'])
    return LaunchDescription([
        Node(
            package='vl53_distance',
            executable='vl53_distance_action',
            name='vl53_distance_action',
            output='screen',
            prefix=_python_executable(),
            parameters=[config],
        ),
    ])
