"""Open the main RViz telemetry dashboard for an already running CBR robot."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution([
        FindPackageShare('bringup'), 'config', 'telemetry.rviz'])

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='telemetry',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
