"""Start the physical stack and MoveIt planning on the robot computer."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils.launches import generate_move_group_launch

from so_arm_101_moveit_config.configuration import get_moveit_config


def generate_launch_description():
    bringup_share = FindPackageShare('so_arm_101_bringup')
    moveit_config = get_moveit_config()
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            bringup_share, 'launch', 'real.launch.py'])),
        launch_arguments={
            'port': LaunchConfiguration('port'),
            'robot_id': LaunchConfiguration('robot_id'),
            'calibration_file': LaunchConfiguration('calibration_file'),
        }.items())
    controller_ready = Node(
        package='so_arm_101_bringup', executable='wait_for_controllers',
        output='screen',
        # On Banana Pi, controller plugins can take several seconds to load
        # sequentially after the manager service becomes available.
        parameters=[{'timeout_sec': 60.0}],
    )
    move_group_entities = generate_move_group_launch(moveit_config).entities

    def start_move_group(event, context):
        del context
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(
                reason='Controllers não ficaram ativos para o MoveIt.'))]
        return move_group_entities

    def is_move_group(action):
        return getattr(action, '_Node__node_executable', None) == 'move_group'

    def move_group_exit(event, context):
        del context
        return [EmitEvent(event=Shutdown(reason=(
            f'move_group encerrou com código {event.returncode}.')))]

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value=''),
        DeclareLaunchArgument('robot_id', default_value='so101_follower'),
        DeclareLaunchArgument(
            'calibration_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('so_arm_101_hardware'),
                'config', 'so101_follower.json'])),
        hardware,
        controller_ready,
        RegisterEventHandler(OnProcessExit(
            target_action=controller_ready, on_exit=start_move_group)),
        RegisterEventHandler(OnProcessExit(
            target_action=is_move_group, on_exit=move_group_exit)),
    ])
