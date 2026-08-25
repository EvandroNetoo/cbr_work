"""Launch the autonomous physical robot on the Banana Pi."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _validate_profile(context):
    enabled = {name: LaunchConfiguration(name).perform(context) == 'true'
               for name in ('enable_base', 'enable_arm',
                            'enable_perception', 'enable_moveit')}
    if enabled['enable_moveit'] and not enabled['enable_arm']:
        raise RuntimeError('enable_moveit:=true requires enable_arm:=true')
    if enabled['enable_perception'] and not enabled['enable_arm']:
        raise RuntimeError(
            'enable_perception:=true requires enable_arm:=true because the camera TF is on the arm')
    if not enabled['enable_base'] and not enabled['enable_arm']:
        raise RuntimeError('At least one of enable_base or enable_arm must be true')
    return []


def _include(filename, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('bringup'), 'launch', filename])),
        launch_arguments=arguments.items())


def generate_launch_description():
    common = {'enable_base': LaunchConfiguration('enable_base'),
              'enable_arm': LaunchConfiguration('enable_arm')}
    return LaunchDescription([
        DeclareLaunchArgument('enable_base', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_arm', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_perception', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_moveit', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('port', default_value='__from_config__'),
        DeclareLaunchArgument('robot_id', default_value='__from_config__'),
        DeclareLaunchArgument('calibration_file', default_value='__from_config__'),
        DeclareLaunchArgument('camera_framerate', default_value='15.0'),
        DeclareLaunchArgument('controller_update_rate', default_value='30'),
        DeclareLaunchArgument('controller_manager_timeout', default_value='120.0'),
        DeclareLaunchArgument('hardware_state_timeout', default_value='45.0'),
        OpaqueFunction(function=_validate_profile),
        _include('hardware.launch.py', {
            **common, 'port': LaunchConfiguration('port'),
            'robot_id': LaunchConfiguration('robot_id'),
            'calibration_file': LaunchConfiguration('calibration_file'),
            'controller_update_rate': LaunchConfiguration('controller_update_rate'),
            'controller_manager_timeout': LaunchConfiguration('controller_manager_timeout'),
            'hardware_state_timeout': LaunchConfiguration('hardware_state_timeout')}),
        _include('sensors.launch.py', {'enable_base': LaunchConfiguration('enable_base')}),
        _include('localization.launch.py', {'enable_base': LaunchConfiguration('enable_base')}),
        _include('perception.launch.py', {
            'enable_perception': LaunchConfiguration('enable_perception'),
            'camera_framerate': LaunchConfiguration('camera_framerate')}),
        _include('manipulation.launch.py', {
            **common, 'enable_moveit': LaunchConfiguration('enable_moveit')}),
    ])
