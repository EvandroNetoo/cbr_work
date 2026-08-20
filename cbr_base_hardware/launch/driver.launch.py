"""Start the Mariola base driver with its fixed YAML configuration."""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _python_executable():
    virtualenv = os.environ.get('VIRTUAL_ENV')
    if virtualenv:
        candidate = os.path.join(virtualenv, 'bin', 'python')
        if os.path.isfile(candidate):
            return candidate
    return sys.executable


def generate_launch_description():
    driver = Node(
        package='cbr_base_hardware',
        executable='cbr_base_hardware_node',
        output='screen',
        prefix=_python_executable(),
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('cbr_base_hardware'), 'config', 'hardware.yaml']),
            {
                'deduplicate_commands': ParameterValue(
                    LaunchConfiguration('deduplicate_commands'), value_type=bool),
                'command_heartbeat_hz': ParameterValue(
                    LaunchConfiguration('command_heartbeat_hz'), value_type=float),
            },
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'deduplicate_commands', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('command_heartbeat_hz', default_value='5.0'),
        driver,
        RegisterEventHandler(OnProcessExit(
            target_action=driver,
            on_exit=[EmitEvent(event=Shutdown(
                reason='O driver físico da base encerrou.'))],
        )),
    ])
