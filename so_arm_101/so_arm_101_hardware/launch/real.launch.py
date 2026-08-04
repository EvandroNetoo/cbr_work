"""Compatibility wrapper for the low-level driver launch.

Use ``driver.launch.py`` when only the LeRobot node is needed. The complete
hardware stack is provided by ``so_arm_101_bringup real.launch.py``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware_share = FindPackageShare('so_arm_101_hardware')
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value=''),
        DeclareLaunchArgument('robot_id', default_value='so101_follower'),
        DeclareLaunchArgument(
            'calibration_file',
            default_value=PathJoinSubstitution([
                hardware_share, 'config', 'so101_follower.json'])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                hardware_share,
                'launch', 'driver.launch.py'])),
            launch_arguments={
                'port': LaunchConfiguration('port'),
                'robot_id': LaunchConfiguration('robot_id'),
                'calibration_file': LaunchConfiguration('calibration_file'),
            }.items()),
    ])
