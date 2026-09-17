"""Start the on-demand Bin 3 container detector."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config = PathJoinSubstitution([
        FindPackageShare('container_detection'), 'config',
        'container_detection.yaml',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/image_rect'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        Node(
            package='container_detection',
            executable='container_detector',
            name='container_detector',
            output='screen',
            parameters=[config, {
                'image_topic': LaunchConfiguration('image_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'base_frame': LaunchConfiguration('base_frame'),
            }],
        ),
    ])
