"""Start the physical ros2_control stack, MoveIt and MoveIt RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
)

from so_arm_101_moveit_config.configuration import get_moveit_config


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare('so_arm_101_bringup')
    moveit_config = get_moveit_config()

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            bringup_share, 'launch', 'real.launch.py'])),
        launch_arguments={
            'port': LaunchConfiguration('port'),
            'robot_id': LaunchConfiguration('robot_id'),
            'calibration_file': LaunchConfiguration('calibration_file'),
        }.items())

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value=''),
        DeclareLaunchArgument('robot_id', default_value='so101_follower'),
        DeclareLaunchArgument(
            'calibration_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('so_arm_101_hardware'),
                'config', 'so101_follower.json'])),
        hardware,
        *generate_move_group_launch(moveit_config).entities,
        *generate_moveit_rviz_launch(moveit_config).entities,
    ])
