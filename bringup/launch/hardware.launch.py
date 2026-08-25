"""Physical drivers, the sole robot_state_publisher and controller manager."""

import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


CONFIG_DEFAULT = '__from_config__'


def _project_python(env_name, package_name):
    configured = os.environ.get(env_name)
    if configured and Path(configured).is_file():
        return configured
    if os.environ.get('VIRTUAL_ENV'):
        candidate = Path(os.environ['VIRTUAL_ENV']) / 'bin' / 'python'
        if candidate.is_file():
            return str(candidate)
    share = Path(get_package_share_directory(package_name)).resolve()
    for parent in (share, *share.parents):
        candidate = parent / '.venv' / 'bin' / 'python'
        if candidate.is_file():
            return str(candidate)
    return sys.executable


def _shutdown_if_failed(action, label):
    def callback(event, context):
        del context
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(
                reason=f'{label} encerrou com código {event.returncode}.'))]
        return []
    return RegisterEventHandler(OnProcessExit(target_action=action, on_exit=callback))


def _driver_shutdown(action, label):
    return RegisterEventHandler(OnProcessExit(
        target_action=action,
        on_exit=[EmitEvent(event=Shutdown(reason=f'{label} encerrou.'))]))


def _launch_hardware(context):
    base_enabled = LaunchConfiguration('enable_base').perform(context) == 'true'
    arm_enabled = LaunchConfiguration('enable_arm').perform(context) == 'true'
    rate = int(LaunchConfiguration('controller_update_rate').perform(context))
    timeout = LaunchConfiguration('controller_manager_timeout').perform(context)
    state_timeout = float(LaunchConfiguration('hardware_state_timeout').perform(context))

    if base_enabled and arm_enabled:
        description_file = Path(get_package_share_directory('robot_description')) / 'urdf/robot.urdf.xacro'
        controller_files = [Path(get_package_share_directory('bringup')) / 'config/controllers.yaml']
    elif arm_enabled:
        description_file = Path(get_package_share_directory('so_arm_101_description')) / 'urdf/so_101.urdf.xacro'
        controller_files = [
            Path(get_package_share_directory('so_arm_101_bringup')) / 'config/controllers.yaml',
            Path(get_package_share_directory('so_arm_101_bringup')) / 'config/controllers_real_overrides.yaml']
    else:
        description_file = Path(get_package_share_directory('base_description')) / 'urdf/base.urdf.xacro'
        controller_files = [Path(get_package_share_directory('base_bringup')) / 'config/controllers.yaml']

    xacro = ['xacro ', str(description_file)]
    if arm_enabled and not base_enabled:
        xacro += [' use_real_ros2_control:=true',
                  ' hardware_plugin:=so_arm_101_hardware_interface/SO101System']
    description = ParameterValue(Command(xacro), value_type=str)
    actions = []

    if arm_enabled:
        arm_parameters = [str(Path(get_package_share_directory(
            'so_arm_101_hardware')) / 'config/real.yaml')]
        overrides = {}
        for argument in ('port', 'robot_id', 'calibration_file'):
            value = LaunchConfiguration(argument).perform(context)
            if value != CONFIG_DEFAULT:
                overrides[argument] = value
        if overrides:
            arm_parameters.append(overrides)
        arm_driver = Node(
            package='so_arm_101_hardware', executable='so101_hardware_node',
            name='so101_hardware_node', output='screen',
            prefix=_project_python('SO_ARM_101_PYTHON', 'so_arm_101_hardware'),
            parameters=arm_parameters)
        actions += [_driver_shutdown(arm_driver, 'Driver físico do SO-101'), arm_driver]

    if base_enabled:
        base_driver = Node(
            package='base_hardware', executable='base_hardware_node', output='screen',
            prefix=_project_python('CBR_BASE_PYTHON', 'base_hardware'),
            parameters=[str(Path(get_package_share_directory(
                'base_hardware')) / 'config/hardware.yaml')])
        actions += [_driver_shutdown(base_driver, 'Driver físico da base'), base_driver]

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': description, 'use_sim_time': False}], output='screen')
    control = Node(
        package='controller_manager', executable='ros2_control_node',
        parameters=[{'robot_description': description}, *map(str, controller_files),
                    {'update_rate': rate}], output='screen')
    actions += [rsp]

    controllers = ['joint_state_broadcaster']
    if arm_enabled:
        controllers += ['arm_controller', 'gripper_controller']
    if base_enabled:
        controllers += ['base_controller']
    controller_actions = [_driver_shutdown(control, 'controller_manager'), control]
    for controller in controllers:
        arguments = [controller, '--controller-manager', '/controller_manager',
                     '--controller-manager-timeout', timeout]
        if controller == 'base_controller':
            arguments += [
                '--controller-ros-args', '--remap ~/reference:=/cmd_vel',
                '--controller-ros-args', '--remap ~/odometry:=/wheel/odom']
        spawner = Node(
            package='controller_manager', executable='spawner',
            name=f'spawn_{controller}', arguments=arguments, output='screen')
        controller_actions += [
            _shutdown_if_failed(spawner, f'Spawner de {controller}'), spawner]

    readiness = Node(
        package='bringup', executable='hardware_readiness', output='screen',
        parameters=[{'enable_base': base_enabled, 'enable_arm': arm_enabled,
                     'timeout_sec': state_timeout}])

    def after_readiness(event, launch_context):
        del launch_context
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(
                reason='Hardware não forneceu estado físico válido.'))]
        return controller_actions

    actions += [
        RegisterEventHandler(OnProcessExit(
            target_action=readiness, on_exit=after_readiness)),
        readiness]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_base', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_arm', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('port', default_value=CONFIG_DEFAULT),
        DeclareLaunchArgument('robot_id', default_value=CONFIG_DEFAULT),
        DeclareLaunchArgument('calibration_file', default_value=CONFIG_DEFAULT),
        DeclareLaunchArgument('controller_update_rate', default_value='30'),
        DeclareLaunchArgument('controller_manager_timeout', default_value='120.0'),
        DeclareLaunchArgument('hardware_state_timeout', default_value='45.0'),
        OpaqueFunction(function=_launch_hardware)])
