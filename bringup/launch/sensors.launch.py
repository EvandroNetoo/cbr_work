"""Base-coupled physical sensors, independently selectable and non-fatal."""

import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _project_python():
    configured = os.environ.get('CBR_SENSOR_PYTHON')
    if configured and Path(configured).is_file():
        return configured
    if os.environ.get('VIRTUAL_ENV'):
        candidate = Path(os.environ['VIRTUAL_ENV']) / 'bin' / 'python'
        if candidate.is_file():
            return str(candidate)
    share = Path(get_package_share_directory('bringup')).resolve()
    for parent in (share, *share.parents):
        candidate = parent / '.venv' / 'bin' / 'python'
        if candidate.is_file():
            return str(candidate)
    return sys.executable


def _shutdown_if_failed(node, label):
    def callback(event, context):
        fatal = LaunchConfiguration('sensor_failures_are_fatal').perform(context) == 'true'
        if fatal and event.returncode != 0:
            return [EmitEvent(event=Shutdown(
                reason=f'{label} encerrou com código {event.returncode}.'))]
        return []
    return RegisterEventHandler(OnProcessExit(target_action=node, on_exit=callback))


def _launch_sensors(context):
    del context
    python = _project_python()
    lidar = Node(
        package='lidar', executable='lidar_node', output='screen',
        prefix=python, condition=IfCondition(LaunchConfiguration('enable_lidar')),
        parameters=[PathJoinSubstitution([FindPackageShare('lidar'), 'config', 'lidar.yaml'])])
    imu = Node(
        package='imu', executable='imu_node', output='screen',
        prefix=python, condition=IfCondition(LaunchConfiguration('enable_imu')),
        parameters=[PathJoinSubstitution([FindPackageShare('imu'), 'config', 'imu.yaml'])])
    return [lidar, imu, _shutdown_if_failed(lidar, 'LiDAR'), _shutdown_if_failed(imu, 'IMU')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_lidar', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_imu', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('sensor_failures_are_fatal', default_value='false',
                              choices=['true', 'false']),
        OpaqueFunction(function=_launch_sensors)])
