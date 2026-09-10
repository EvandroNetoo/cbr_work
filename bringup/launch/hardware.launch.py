"""Start the hardware-facing half of the physical CBR robot."""

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


def generate_launch_description():
    hardware_timeout = LaunchConfiguration('hardware_state_timeout')
    robot_description = ParameterValue(Command([
        'xacro ', PathJoinSubstitution([
            FindPackageShare('robot_description'), 'urdf', 'robot.urdf.xacro']),
    ]), value_type=str)
    controllers = PathJoinSubstitution([
        FindPackageShare('bringup'), 'config', 'controllers.yaml'])

    arm_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('so_arm_101_hardware'), 'launch', 'driver.launch.py'])),
        launch_arguments={
            'port': LaunchConfiguration('port'),
            'robot_id': LaunchConfiguration('robot_id'),
            'calibration_file': LaunchConfiguration('calibration_file'),
        }.items())
    base_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('base_hardware'), 'launch', 'driver.launch.py'])),
        launch_arguments={
            'deduplicate_commands': LaunchConfiguration('base_deduplicate_commands'),
            'command_heartbeat_hz': LaunchConfiguration('base_command_heartbeat_hz'),
        }.items())
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('lidar'), 'launch', 'lidar.launch.py'])))
    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('imu'), 'launch', 'imu.launch.py'])))
    vl53_distance = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('vl53_distance'), 'launch',
            'vl53_distance.launch.py'])))

    readiness = Node(
        package='bringup', executable='wait_for_hardware_states',
        parameters=[{'timeout_sec': hardware_timeout}], output='screen')
    description_publisher = Node(
        package='bringup', executable='publish_robot_description',
        parameters=[{'robot_description': robot_description}], output='screen')
    control = Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[
            controllers,
            {'update_rate': ParameterValue(
                LaunchConfiguration('controller_update_rate'), value_type=int)},
        ], output='screen')
    joint = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen')
    arm = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller', '-c', '/controller_manager'], output='screen')
    gripper = Node(
        package='controller_manager', executable='spawner',
        arguments=['gripper_controller', '-c', '/controller_manager'], output='screen')
    base = Node(
        package='controller_manager', executable='spawner',
        arguments=[
            'base_controller', '-c', '/controller_manager',
            '--controller-ros-args', '--remap ~/reference:=/cmd_vel',
            '--controller-ros-args', '--remap ~/odometry:=/wheel/odom',
        ], output='screen')

    def shutdown(reason):
        return [EmitEvent(event=Shutdown(reason=reason))]

    def start_control(event, context):
        del context
        return [control, joint] if event.returncode == 0 else shutdown(
            'Braço, base ou IMU não forneceram estado válido.')

    def chain(current, following, label):
        def callback(event, context):
            del context
            return [following] if event.returncode == 0 else shutdown(
                f'Falha ao ativar {label}.')
        return RegisterEventHandler(
            OnProcessExit(target_action=current, on_exit=callback))

    def start_vl53(event, context):
        del context
        return [vl53_distance] if event.returncode == 0 else shutdown(
            'Falha ao ativar base_controller.')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value=CONFIG_DEFAULT),
        DeclareLaunchArgument('robot_id', default_value=CONFIG_DEFAULT),
        DeclareLaunchArgument('hardware_state_timeout', default_value='45.0'),
        DeclareLaunchArgument('controller_update_rate', default_value='30'),
        DeclareLaunchArgument(
            'base_deduplicate_commands', default_value='true',
            choices=['true', 'false']),
        DeclareLaunchArgument('base_command_heartbeat_hz', default_value='5.0'),
        DeclareLaunchArgument('calibration_file', default_value=CONFIG_DEFAULT),
        arm_driver,
        base_driver,
        lidar,
        imu,
        description_publisher,
        readiness,
        RegisterEventHandler(
            OnProcessExit(target_action=readiness, on_exit=start_control)),
        chain(joint, arm, 'joint_state_broadcaster'),
        chain(arm, gripper, 'arm_controller'),
        chain(gripper, base, 'gripper_controller'),
        RegisterEventHandler(
            OnProcessExit(target_action=base, on_exit=start_vl53)),
        RegisterEventHandler(OnProcessExit(
            target_action=description_publisher,
            on_exit=lambda event, context: shutdown(
                'Publicador de robot_description encerrou.'))),
        RegisterEventHandler(OnProcessExit(
            target_action=control,
            on_exit=lambda event, context: shutdown(
                f'controller_manager encerrou com código {event.returncode}.'))),
    ])
