"""Base-coupled physical sensors: LiDAR and IMU."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _critical(node, label):
    return RegisterEventHandler(OnProcessExit(
        target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason=f'{label} encerrou.'))]))


def generate_launch_description():
    enabled = IfCondition(LaunchConfiguration('enable_base'))
    lidar = Node(
        package='lidar', executable='lidar_node', output='screen',
        prefix='/usr/bin/python3', condition=enabled,
        parameters=[PathJoinSubstitution([FindPackageShare('lidar'), 'config', 'lidar.yaml'])])
    imu = Node(
        package='imu', executable='imu_node', output='screen',
        prefix='/usr/bin/python3', condition=enabled,
        parameters=[PathJoinSubstitution([FindPackageShare('imu'), 'config', 'imu.yaml'])])
    return LaunchDescription([
        DeclareLaunchArgument('enable_base', default_value='true', choices=['true', 'false']),
        lidar, imu, _critical(lidar, 'LiDAR'), _critical(imu, 'IMU')])
