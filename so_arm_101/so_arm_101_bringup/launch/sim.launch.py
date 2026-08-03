"""Launch the SO-ARM-101 robot in Gazebo Sim."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    description_package = 'so_arm_101_description'
    bringup_package = 'so_arm_101_bringup'
    description_share_path = get_package_share_directory(description_package)
    description_share = FindPackageShare(description_package)
    bringup_share = FindPackageShare(bringup_package)

    resource_path = os.pathsep.join(filter(None, [
        os.path.dirname(description_share_path),
        os.environ.get('GZ_SIM_RESOURCE_PATH'),
    ]))

    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false', choices=['true', 'false'],
        description='Run Gazebo without its graphical client')
    headless = LaunchConfiguration('headless')
    xacro_file = PathJoinSubstitution([
        description_share, 'urdf', 'so_101.urdf.xacro'])
    controllers_file = PathJoinSubstitution([
        bringup_share, 'config', 'controllers.yaml'])
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_gz_ros2_control:=true controllers_file:=',
            controllers_file,
        ]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }])

    spawners = [
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster']),
        Node(package='controller_manager', executable='spawner',
             arguments=['arm_controller']),
        Node(package='controller_manager', executable='spawner',
             arguments=['gripper_controller']),
    ]

    world = PathJoinSubstitution([bringup_share, 'worlds', 'empty.sdf'])
    gz_flags = PythonExpression([
        "('-r -s ' if '", headless, "' == 'true' else '-r ')",
        " + '--physics-engine gz-physics-bullet-featherstone-plugin '",
    ])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': [gz_flags, world]}.items())
    spawn_robot = Node(
        package='ros_gz_sim', executable='create', arguments=[
            '-topic', '/robot_description', '-name', 'so_101', '-z', '0.001'])
    spawn_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=spawners,
        ))

    return LaunchDescription([
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH',
                               value=resource_path),
        headless_arg,
        robot_state_publisher,
        gazebo,
        spawn_controllers,
        spawn_robot,
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
             output='screen'),
    ])
