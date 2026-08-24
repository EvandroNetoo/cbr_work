"""MoveIt planning on the embedded computer; no RViz is started here."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils.launches import generate_move_group_launch

from so_arm_101_moveit_config.configuration import get_combined_moveit_config, get_moveit_config


def _launch_move_group(context):
    if LaunchConfiguration('enable_moveit').perform(context) != 'true':
        return []
    if LaunchConfiguration('enable_arm').perform(context) != 'true':
        raise RuntimeError('MoveIt requires enable_arm:=true')
    combined = LaunchConfiguration('enable_base').perform(context) == 'true'
    entities = list(generate_move_group_launch(
        get_combined_moveit_config() if combined else get_moveit_config()).entities)

    def is_move_group(action):
        return getattr(action, '_Node__node_executable', None) == 'move_group'

    entities.append(RegisterEventHandler(OnProcessExit(
        target_action=is_move_group,
        on_exit=[EmitEvent(event=Shutdown(reason='move_group encerrou.'))])))
    return entities


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_base', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_arm', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_moveit', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=_launch_move_group)])
