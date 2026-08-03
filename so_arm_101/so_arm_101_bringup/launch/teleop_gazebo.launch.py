"""Compatibility alias for keyboard_control.launch.py."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    headless = LaunchConfiguration('headless')
    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='false', choices=['true', 'false'],
            description='Run Gazebo without its graphical client'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('so_arm_101_bringup'), 'launch',
                'keyboard_control.launch.py'])),
            launch_arguments={'headless': headless}.items()),
    ])
