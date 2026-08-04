"""Start the physical SO-ARM-101 LeRobot bridge."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('robot_id', default_value='so101_follower'),
        DeclareLaunchArgument(
            'calibration_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('so_arm_101_hardware'),
                'config', 'so101_follower.json']),
        ),
        Node(
            package='so_arm_101_hardware',
            executable='so101_hardware_node',
            name='so101_hardware_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('so_arm_101_hardware'), 'config', 'real.yaml']),
                {'port': LaunchConfiguration('port'),
                 'robot_id': LaunchConfiguration('robot_id'),
                 'calibration_file': LaunchConfiguration('calibration_file')},
            ],
        ),
    ])
