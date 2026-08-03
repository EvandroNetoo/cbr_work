"""Launch Gazebo, RViz and keyboard control for the SO-ARM-101."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Build the complete simulation and keyboard launch description."""
    package_share = FindPackageShare('so_arm_101_description')
    headless = LaunchConfiguration('headless')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, 'launch', 'sim.launch.py'])
        ),
        launch_arguments={'headless': headless}.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', PathJoinSubstitution([
                package_share, 'config', 'display.rviz',
            ]),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, 'launch', 'teleop.launch.py'])
        ),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            choices=['true', 'false'],
            description='Run Gazebo without its graphical client',
        ),
        simulation,
        rviz,
        teleop,
    ])
