"""Start the sequential mission manager with package-installed YAML files."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare('mission_manager')
    config = PathJoinSubstitution([share, 'config', 'mission_manager.yaml'])
    arena = PathJoinSubstitution([share, 'config', 'arena.yaml'])
    plans = PathJoinSubstitution([share, 'config', 'plans'])
    return LaunchDescription([
        Node(
            package='mission_manager',
            executable='mission_manager_node',
            name='mission_manager',
            output='screen',
            parameters=[config, {
                'arena_file': arena,
                'plans_directory': plans,
            }],
        ),
    ])
