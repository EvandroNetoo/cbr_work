"""Optional workstation tools; autonomous processes stay on the robot."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from so_arm_101_moveit_config.configuration import get_combined_moveit_config


def generate_launch_description():
    moveit_config = get_combined_moveit_config()
    xbox_config = PathJoinSubstitution([
        FindPackageShare('bringup'), 'config', 'controllers.yaml'])
    rviz = Node(
        package='rviz2', executable='rviz2', name='workstation_rviz', output='screen',
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('bringup'), 'config', 'telemetry.rviz'])],
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    moveit_config.planning_pipelines])
    keyboard = Node(
        package='so_arm_101_teleop', executable='keyboard_teleop',
        name='keyboard_teleop', output='screen', emulate_tty=True,
        prefix='gnome-terminal --wait --',
        condition=IfCondition(LaunchConfiguration('enable_keyboard_teleop')))
    joy = Node(
        package='joy', executable='joy_node', name='joy_node', output='screen',
        parameters=[xbox_config],
        condition=IfCondition(LaunchConfiguration('enable_xbox_teleop')))
    xbox = Node(
        package='bringup', executable='xbox_base_teleop',
        name='xbox_base_teleop', output='screen', parameters=[xbox_config],
        condition=IfCondition(LaunchConfiguration('enable_xbox_teleop')))
    return LaunchDescription([
        DeclareLaunchArgument('enable_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_keyboard_teleop', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('enable_xbox_teleop', default_value='false',
                              choices=['true', 'false']),
        rviz, keyboard, joy, xbox])
