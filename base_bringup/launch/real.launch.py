"""Start the physical base stack; all base configuration comes from YAML."""

from launch import LaunchDescription
from launch.actions import EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description = ParameterValue(Command([
        'xacro ', PathJoinSubstitution([
            FindPackageShare('base_description'), 'urdf', 'base.urdf.xacro']),
    ]), value_type=str)
    controllers = PathJoinSubstitution([
        FindPackageShare('base_bringup'), 'config', 'controllers.yaml'])
    driver = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare('base_hardware'), 'launch', 'driver.launch.py'])))
    lidar = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare('lidar'), 'launch', 'lidar.launch.py'])))
    localization = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare('imu'), 'launch', 'imu_localization.launch.py'])))
    readiness = Node(
        package='base_bringup', executable='wait_for_wheel_states', output='screen')
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': description, 'use_sim_time': False}], output='screen')
    control = Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[{'robot_description': description}, controllers], output='screen')
    joint = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'], output='screen')
    base = Node(
        package='controller_manager', executable='spawner',
        arguments=[
            'base_controller', '-c', '/controller_manager',
            '--controller-ros-args', '--remap ~/reference:=/cmd_vel',
            '--controller-ros-args', '--remap ~/odometry:=/wheel/odom',
        ], output='screen')

    def after_readiness(event, context):
        del context
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='A base não forneceu estado válido.'))]
        return [control, joint]

    def after_joint(event, context):
        del context
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Falha no joint_state_broadcaster.'))]
        return [base]

    return LaunchDescription([
        driver, lidar, localization, readiness, rsp,
        RegisterEventHandler(OnProcessExit(target_action=readiness, on_exit=after_readiness)),
        RegisterEventHandler(OnProcessExit(target_action=joint, on_exit=after_joint)),
        RegisterEventHandler(OnProcessExit(
            target_action=control,
            on_exit=[EmitEvent(event=Shutdown(reason='controller_manager da base encerrou.'))])),
    ])
