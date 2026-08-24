"""Planar wheel/IMU localization for the physical base."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ekf = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        output='screen', condition=IfCondition(LaunchConfiguration('enable_base')),
        parameters=[PathJoinSubstitution([FindPackageShare('imu'), 'config', 'ekf.yaml'])],
        remappings=[('odometry/filtered', '/odom')])
    return LaunchDescription([
        DeclareLaunchArgument('enable_base', default_value='true', choices=['true', 'false']),
        ekf,
        RegisterEventHandler(OnProcessExit(
            target_action=ekf,
            on_exit=[EmitEvent(event=Shutdown(reason='EKF encerrou.'))]))])
