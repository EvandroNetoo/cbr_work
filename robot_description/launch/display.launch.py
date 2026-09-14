"""Display the complete robot model offline in RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    xacro_file = PathJoinSubstitution([
        FindPackageShare('robot_description'), 'urdf', 'robot.urdf.xacro'])
    robot_description = ParameterValue(Command([
        'xacro ', xacro_file,
        ' use_real_ros2_control:=false',
    ]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gui', default_value='true', choices=['true', 'false'],
            description='Use the joint_state_publisher graphical sliders.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true', choices=['true', 'false'],
            description='Open RViz with the complete robot model.'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('base_description'), 'rviz', 'base.rviz']),
            description='RViz configuration file.'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
            output='screen'),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(use_gui),
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
            output='screen'),
    ])
