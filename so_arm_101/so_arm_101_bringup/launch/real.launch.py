"""Start the physical SO-ARM-101 ros2_control stack."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


CONFIG_DEFAULT = '__from_config__'


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
            'disable_torque': LaunchConfiguration('disable_torque'),
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
    readiness = Node(
        package='so_arm_101_bringup', executable='wait_for_joint_states',
        output='screen',
        parameters=[{
            'timeout_sec': LaunchConfiguration('hardware_state_timeout'),
        }],
    )
    spawn_joint = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager',
                   '/controller_manager', '--controller-manager-timeout', '10'],
        output='screen')
    spawn_arm = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller', '--controller-manager',
                   '/controller_manager', '--controller-manager-timeout', '10'],
        output='screen')
    spawn_gripper = Node(
        package='controller_manager', executable='spawner',
        arguments=['gripper_controller', '--controller-manager',
                   '/controller_manager', '--controller-manager-timeout', '10'],
        output='screen')

    def shutdown(reason):
        return [EmitEvent(event=Shutdown(reason=reason))]

    def next_after(action, next_action, label):
        def callback(event, context):
            del context
            if event.returncode != 0:
                return shutdown(f'{label} encerrou com código {event.returncode}.')
            return [next_action]
        return RegisterEventHandler(OnProcessExit(
            target_action=action, on_exit=callback))

    def start_control_after_readiness(event, context):
        del context
        if event.returncode != 0:
            return shutdown('O hardware não forneceu um estado válido.')
        return [control_node, spawn_joint]

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value=CONFIG_DEFAULT),
        DeclareLaunchArgument('robot_id', default_value=CONFIG_DEFAULT),
        DeclareLaunchArgument('disable_torque', default_value=CONFIG_DEFAULT),
        DeclareLaunchArgument(
            'hardware_state_timeout',
            default_value='45.0',
            description=(
                'Maximum seconds to wait for the physical driver to connect '
                'and publish one complete joint state.'),
        ),
        DeclareLaunchArgument(
            'calibration_file',
            default_value=CONFIG_DEFAULT),
        driver,
        robot_state_publisher,
        readiness,
        RegisterEventHandler(OnProcessExit(
            target_action=readiness, on_exit=start_control_after_readiness)),
        RegisterEventHandler(OnProcessExit(
            target_action=control_node,
            on_exit=lambda event, context: shutdown(
                f'ros2_control_node encerrou com código {event.returncode}.'),
        )),
        next_after(spawn_joint, spawn_arm, 'joint_state_broadcaster'),
        next_after(spawn_arm, spawn_gripper, 'arm_controller'),
    ])
