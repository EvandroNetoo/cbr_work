"""MoveIt planning on the embedded computer; no RViz is started here."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils.launches import generate_move_group_launch

from so_arm_101_moveit_config.configuration import get_combined_moveit_config, get_moveit_config


def _move_group_entities(context):
    if LaunchConfiguration('enable_moveit').perform(context) != 'true':
        return []
    if LaunchConfiguration('enable_arm').perform(context) != 'true':
        raise RuntimeError('MoveIt requires enable_arm:=true')
    combined = LaunchConfiguration('enable_base').perform(context) == 'true'
    entities = list(generate_move_group_launch(
        get_combined_moveit_config() if combined else get_moveit_config()).entities)

    def is_move_group(action):
        return getattr(action, '_Node__node_executable', None) == 'move_group'

    def shutdown_if_failed(event, launch_context):
        fatal = LaunchConfiguration('moveit_failure_is_fatal').perform(launch_context) == 'true'
        if fatal and event.returncode != 0:
            return [EmitEvent(event=Shutdown(
                reason=f'move_group encerrou com código {event.returncode}.'))]
        return []

    entities.append(RegisterEventHandler(OnProcessExit(
        target_action=is_move_group, on_exit=shutdown_if_failed)))
    return entities


def _launch_when_ready(context):
    if LaunchConfiguration('enable_moveit').perform(context) != 'true':
        return []
    if LaunchConfiguration('enable_arm').perform(context) != 'true':
        raise RuntimeError('MoveIt requires enable_arm:=true')

    readiness = Node(
        package='bringup', executable='controller_readiness', output='screen',
        parameters=[{
            'controllers': [
                'joint_state_broadcaster', 'arm_controller', 'gripper_controller'],
            'timeout_sec': float(
                LaunchConfiguration('controller_readiness_timeout').perform(context)),
        }])

    def after_readiness(event, launch_context):
        if event.returncode == 0:
            return _move_group_entities(launch_context)
        if LaunchConfiguration('moveit_failure_is_fatal').perform(launch_context) == 'true':
            return [EmitEvent(event=Shutdown(
                reason='Controllers do braço não ficaram ativos para o MoveIt.'))]
        return []

    return [
        RegisterEventHandler(OnProcessExit(
            target_action=readiness, on_exit=after_readiness)),
        readiness,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_base', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_arm', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_moveit', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('moveit_failure_is_fatal', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('controller_readiness_timeout', default_value='120.0'),
        OpaqueFunction(function=_launch_when_ready)])
