"""Start only the camera acquisition layer, with optional rectification."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution([
        FindPackageShare('camera'), 'config', 'camera.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='Single source of acquisition parameters.'),
        DeclareLaunchArgument(
            'rectify', default_value='false', choices=['true', 'false'],
            description='Publish image_rect; requires a valid calibration.'),
        DeclareLaunchArgument(
            'framerate', default_value='15.0',
            description='Camera capture rate; use 30.0 for performance rollback.'),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            namespace='camera',
            name='driver',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {'framerate': ParameterValue(
                    LaunchConfiguration('framerate'), value_type=float)},
            ],
        ),
        Node(
            package='image_proc',
            executable='rectify_node',
            namespace='camera',
            name='rectify',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rectify')),
            remappings=[('image', 'image_raw')],
        ),
    ])
