from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = PathJoinSubstitution([
        FindPackageShare('cbr_base_description'), 'urdf', 'cbr_base.urdf.xacro'])
    rviz_config = LaunchConfiguration('rviz_config')
    use_gui = LaunchConfiguration('use_gui')
    description = ParameterValue(Command(['xacro ', model]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('cbr_base_description'),
                'rviz', 'cbr_base.rviz'])),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(use_gui)),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             condition=UnlessCondition(use_gui)),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', rviz_config], output='screen',
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
