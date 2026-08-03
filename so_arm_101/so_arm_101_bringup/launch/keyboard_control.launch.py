"""Launch simulation, RViz and keyboard teleoperation."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare('so_arm_101_bringup')
    headless = LaunchConfiguration('headless')
    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='false', choices=['true', 'false'],
            description='Run Gazebo without its graphical client'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bringup_share, 'launch', 'sim.launch.py'])),
            launch_arguments={'headless': headless}.items()),
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', PathJoinSubstitution([
                bringup_share, 'config', 'display.rviz'])],
            parameters=[{'use_sim_time': True}], output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bringup_share, 'launch', 'teleop.launch.py']))),
    ])
