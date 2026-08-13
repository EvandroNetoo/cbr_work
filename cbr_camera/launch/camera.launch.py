"""Start only the camera acquisition layer, with optional rectification."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution([
        FindPackageShare('cbr_camera'), 'config', 'camera.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='Single source of acquisition parameters.'),
        DeclareLaunchArgument(
            'rectify', default_value='false', choices=['true', 'false'],
            description='Publish image_rect; requires a valid calibration.'),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            namespace='camera',
            name='driver',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
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
