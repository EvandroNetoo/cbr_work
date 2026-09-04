"""Run the complete physical CBR robot with one controller manager."""

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
from moveit_configs_utils.launches import generate_move_group_launch

from so_arm_101_moveit_config.configuration import get_combined_moveit_config


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
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('imu'), 'launch', 'imu_localization.launch.py'])))
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('camera'), 'launch', 'camera.launch.py'])),
        launch_arguments={
            'rectify': 'true',
            'framerate': LaunchConfiguration('camera_framerate'),
        }.items())
    apriltag = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('apriltag'), 'launch', 'apriltag.launch.py'])),
        launch_arguments={
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'base_frame': LaunchConfiguration('base_frame'),
        }.items())
    vl53_distance = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('vl53_distance'), 'launch',
            'vl53_distance.launch.py'])))
    manipulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('manipulation'), 'launch',
            'manipulation.launch.py'])))
    readiness = Node(
        package='bringup', executable='wait_for_hardware_states',
        parameters=[{'timeout_sec': hardware_timeout}], output='screen')
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
        output='screen')
    control = Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers,
            {'update_rate': ParameterValue(
                LaunchConfiguration('controller_update_rate'), value_type=int)},
        ], output='screen')
    joint = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'], output='screen')
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
    move_group_entities = generate_move_group_launch(get_combined_moveit_config()).entities

    def shutdown(reason):
        return [EmitEvent(event=Shutdown(reason=reason))]

    def start_control(event, context):
        del context
        return [control, joint] if event.returncode == 0 else shutdown(
            'Braço ou base não forneceram estado válido.')

    def chain(current, following, label):
        def callback(event, context):
            del context
            return [following] if event.returncode == 0 else shutdown(
                f'Falha ao ativar {label}.')
        return RegisterEventHandler(OnProcessExit(target_action=current, on_exit=callback))

    def start_move_group(event, context):
        del context
        return (
            [vl53_distance, manipulation] + move_group_entities
            if event.returncode == 0
            else shutdown('Falha ao ativar base_controller.')
        )

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value=CONFIG_DEFAULT),
        DeclareLaunchArgument('robot_id', default_value=CONFIG_DEFAULT),
        DeclareLaunchArgument('hardware_state_timeout', default_value='45.0'),
        DeclareLaunchArgument('camera_framerate', default_value='15.0'),
        DeclareLaunchArgument('controller_update_rate', default_value='30'),
        DeclareLaunchArgument(
            'base_deduplicate_commands', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('base_command_heartbeat_hz', default_value='5.0'),
        DeclareLaunchArgument(
            'calibration_file', default_value=CONFIG_DEFAULT),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_rect'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument(
            'base_frame', default_value='arm_base_link',
            description=(
                'Referencial cartesiano da manipulação. Deve permanecer na '
                'base física do braço para ter a mesma semântica do launch '
                'standalone.')),
        arm_driver, base_driver, lidar, localization, camera, apriltag, rsp, readiness,
        RegisterEventHandler(OnProcessExit(target_action=readiness, on_exit=start_control)),
        chain(joint, arm, 'joint_state_broadcaster'),
        chain(arm, gripper, 'arm_controller'),
        chain(gripper, base, 'gripper_controller'),
        RegisterEventHandler(OnProcessExit(target_action=base, on_exit=start_move_group)),
        RegisterEventHandler(OnProcessExit(
            target_action=control,
            on_exit=lambda event, context: shutdown(
                f'controller_manager encerrou com código {event.returncode}.'))),
    ])
