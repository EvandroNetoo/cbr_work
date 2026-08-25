"""Static-map Nav2 and the idle-on-start mission manager."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def _nav2_launch(filename, arguments, condition):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nav2_bringup'), 'launch', filename])),
        launch_arguments=arguments.items(), condition=condition)


def generate_launch_description():
    navigation_enabled = IfCondition(LaunchConfiguration('enable_navigation'))
    mission_enabled = IfCondition(LaunchConfiguration('enable_mission'))
    nav_params = RewrittenYaml(
        source_file=PathJoinSubstitution([
            FindPackageShare('bringup'), 'config', 'nav2_navigation.yaml']),
        param_rewrites={'default_nav_to_pose_bt_xml': PathJoinSubstitution([
            FindPackageShare('bringup'), 'behavior_trees',
            'navigate_to_pose_safe.xml'])},
        convert_types=True)
    amcl_params = PathJoinSubstitution([
        FindPackageShare('bringup'), 'config', 'amcl_localization.yaml'])
    map_file = PathJoinSubstitution([
        FindPackageShare('bringup'), 'maps', 'arena.yaml'])
    mission_file = PathJoinSubstitution([
        FindPackageShare('mission_manager'), 'config', 'missions.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('enable_navigation', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('enable_mission', default_value='false',
                              choices=['true', 'false']),
        _nav2_launch('localization_launch.py', {
            'map': map_file, 'params_file': amcl_params,
            'use_composition': 'false', 'autostart': 'true'}, navigation_enabled),
        _nav2_launch('navigation_launch.py', {
            'params_file': nav_params, 'use_composition': 'false',
            'autostart': 'true'}, navigation_enabled),
        Node(package='mission_manager', executable='mission_manager',
             name='mission_manager', output='screen', condition=mission_enabled,
             parameters=[{'mission_file': mission_file}]),
    ])
