"""Start the physical SO-ARM-101 ros2_control stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    hardware_share = FindPackageShare('so_arm_101_hardware')
    description_share = FindPackageShare('so_arm_101_description')
    bringup_share = FindPackageShare('so_arm_101_bringup')

    xacro_file = PathJoinSubstitution([
        description_share, 'urdf', 'so_101.urdf.xacro'])
    controllers_file = PathJoinSubstitution([
        bringup_share, 'config', 'controllers.yaml'])
    real_overrides_file = PathJoinSubstitution([
        bringup_share, 'config', 'controllers_real_overrides.yaml'])
    robot_description = ParameterValue(Command([
        'xacro ', xacro_file,
        ' use_real_ros2_control:=true',
        ' hardware_plugin:=so_arm_101_hardware_interface/SO101System',
    ]), value_type=str)

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            hardware_share, 'launch', 'driver.launch.py'])),
        launch_arguments={
            'port': LaunchConfiguration('port'),
            'robot_id': LaunchConfiguration('robot_id'),
            'calibration_file': LaunchConfiguration('calibration_file'),
        }.items())

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
        output='screen')
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_file,
            real_overrides_file,
        ],
        output='screen')
    spawners = TimerAction(period=2.0, actions=[
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster',
                        '--controller-manager', '/controller_manager'],
             output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['arm_controller',
                        '--controller-manager', '/controller_manager'],
             output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['gripper_controller',
                        '--controller-manager', '/controller_manager'],
             output='screen'),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value=''),
        DeclareLaunchArgument('robot_id', default_value='so101_follower'),
        DeclareLaunchArgument(
            'calibration_file',
            default_value=PathJoinSubstitution([
                hardware_share, 'config', 'so101_follower.json'])),
        driver,
        robot_state_publisher,
        control_node,
        spawners,
    ])
