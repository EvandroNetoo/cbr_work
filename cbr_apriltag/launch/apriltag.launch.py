"""Launch the AprilTag detector with ROS-native calibrated camera topics."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    config = PathJoinSubstitution([FindPackageShare('cbr_apriltag'), 'config', 'apriltag.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/image_rect'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        Node(
            package='cbr_apriltag',
            executable='apriltag_detector',
            name='apriltag_detector',
            output='screen',
            parameters=[config, {
                'image_topic': LaunchConfiguration('image_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'base_frame': LaunchConfiguration('base_frame'),
            }],
        ),
    ])
