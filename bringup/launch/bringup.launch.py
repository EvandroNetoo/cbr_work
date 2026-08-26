"""Canonical production entry point for the physical CBR robot."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _enabled(context, name):
    return LaunchConfiguration(name).perform(context) == 'true'


def _validate_profile(context):
    base = _enabled(context, 'enable_base')
    arm = _enabled(context, 'enable_arm')
    perception = _enabled(context, 'enable_perception')
    moveit = _enabled(context, 'enable_moveit')
    lidar = _enabled(context, 'enable_lidar')
    imu = _enabled(context, 'enable_imu')
    if moveit and not arm:
        raise RuntimeError('enable_moveit:=true requires enable_arm:=true')
    if perception and not arm:
        raise RuntimeError(
            'enable_perception:=true requires enable_arm:=true because the camera TF is on the arm')
    if (lidar or imu) and not base:
        raise RuntimeError('LiDAR and IMU are base-coupled; enable_base must be true')
    if not base and not arm:
        raise RuntimeError('At least one of enable_base or enable_arm must be true')
    return []


def _include(filename, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('bringup'), 'launch', filename])),
        launch_arguments=arguments.items())


def generate_launch_description():
    common = {
        'enable_base': LaunchConfiguration('enable_base'),
        'enable_arm': LaunchConfiguration('enable_arm'),
    }
    return LaunchDescription([
        DeclareLaunchArgument('enable_base', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_arm', default_value='true', choices=['true', 'false']),
        # Ainda não há Nav2/SLAM integrado; não ocupe GPIO/CPU sem consumidor.
        DeclareLaunchArgument('enable_lidar', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_imu', default_value='true', choices=['true', 'false']),
        # Visão é cara e o hardware USB é opcional; habilite para a missão que a usa.
        DeclareLaunchArgument('enable_perception', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_moveit', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('sensor_failures_are_fatal', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('perception_failure_is_fatal', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('localization_failure_is_fatal', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('moveit_failure_is_fatal', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('port', default_value='__from_config__'),
        DeclareLaunchArgument('robot_id', default_value='__from_config__'),
        DeclareLaunchArgument('calibration_file', default_value='__from_config__'),
        DeclareLaunchArgument('camera_framerate', default_value='15.0'),
        DeclareLaunchArgument('controller_update_rate', default_value='30'),
        DeclareLaunchArgument('controller_manager_timeout', default_value='120.0'),
        DeclareLaunchArgument('hardware_state_timeout', default_value='45.0'),
        DeclareLaunchArgument('controller_readiness_timeout', default_value='120.0'),
        OpaqueFunction(function=_validate_profile),
        _include('hardware.launch.py', {
            **common,
            'port': LaunchConfiguration('port'),
            'robot_id': LaunchConfiguration('robot_id'),
            'calibration_file': LaunchConfiguration('calibration_file'),
            'controller_update_rate': LaunchConfiguration('controller_update_rate'),
            'controller_manager_timeout': LaunchConfiguration('controller_manager_timeout'),
            'hardware_state_timeout': LaunchConfiguration('hardware_state_timeout'),
        }),
        _include('sensors.launch.py', {
            'enable_lidar': LaunchConfiguration('enable_lidar'),
            'enable_imu': LaunchConfiguration('enable_imu'),
            'sensor_failures_are_fatal': LaunchConfiguration('sensor_failures_are_fatal'),
        }),
        _include('localization.launch.py', {
            'enable_base': LaunchConfiguration('enable_base'),
            'localization_failure_is_fatal': LaunchConfiguration(
                'localization_failure_is_fatal'),
        }),
        _include('perception.launch.py', {
            'enable_perception': LaunchConfiguration('enable_perception'),
            'perception_failure_is_fatal': LaunchConfiguration(
                'perception_failure_is_fatal'),
            'camera_framerate': LaunchConfiguration('camera_framerate'),
        }),
        _include('manipulation.launch.py', {
            **common,
            'enable_moveit': LaunchConfiguration('enable_moveit'),
            'moveit_failure_is_fatal': LaunchConfiguration('moveit_failure_is_fatal'),
            'controller_readiness_timeout': LaunchConfiguration(
                'controller_readiness_timeout'),
        }),
    ])
