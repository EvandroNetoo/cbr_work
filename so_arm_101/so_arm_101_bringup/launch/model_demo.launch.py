"""Launch an offline URDF model with slider-controlled joint states."""

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = FindPackageShare('so_arm_101_description')
    bringup_share = FindPackageShare('so_arm_101_bringup')
    robot_description = ParameterValue(Command([
        'xacro ', PathJoinSubstitution([
            description_share, 'urdf', 'so_101.urdf.xacro'])]), value_type=str)
    return LaunchDescription([
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', PathJoinSubstitution([
                bringup_share, 'config', 'display.rviz'])],
        ),
    ])
