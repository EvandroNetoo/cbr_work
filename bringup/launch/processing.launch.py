"""Start the processing half of the physical CBR robot on Raspberry Pi."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _enabled(context, name):
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def _resolve_map_file(map_name, maps_directory=None):
    """Resolve one installed map by stem and reject paths or extensions."""
    if not map_name or Path(map_name).name != map_name or Path(map_name).suffix:
        raise RuntimeError(
            "O argumento 'map' deve ser apenas o nome do mapa, sem caminho "
            "ou extensão (exemplo: map:=arena).")

    maps_directory = Path(maps_directory or (
        Path(get_package_share_directory('bringup')) / 'maps'))
    candidate = maps_directory / f'{map_name}.yaml'
    if candidate.is_file():
        return str(candidate)

    available = ', '.join(sorted(path.stem for path in maps_directory.glob('*.yaml')))
    raise RuntimeError(
        f"Mapa '{map_name}' não encontrado em {maps_directory}. "
        f'Mapas disponíveis: {available or "nenhum"}.')


def _shutdown(reason):
    return [EmitEvent(event=Shutdown(reason=reason))]


def _launch_setup(context):
    manipulation_enabled = _enabled(context, 'enable_manipulation')
    moveit_config = None
    move_group_entities = []
    if manipulation_enabled:
        # Keep MoveIt imports and configuration out of disabled profiles.
        from moveit_configs_utils.launches import generate_move_group_launch
        from so_arm_101_moveit_config.configuration import (
            get_combined_moveit_config,
        )

        moveit_config = get_combined_moveit_config()
        move_group_entities = generate_move_group_launch(moveit_config).entities

    if moveit_config is None:
        robot_description = ParameterValue(Command([
            'xacro ', PathJoinSubstitution([
                FindPackageShare('robot_description'), 'urdf',
                'robot.urdf.xacro']),
        ]), value_type=str)
        rsp_parameters = [
            {'robot_description': robot_description, 'use_sim_time': False},
        ]
    else:
        rsp_parameters = [
            moveit_config.robot_description,
            {'use_sim_time': False},
        ]

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=rsp_parameters, output='screen')
    ekf = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node', output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('imu'), 'config', 'ekf.yaml'])],
        remappings=[('odometry/filtered', '/odom')])
    actions = [
        rsp,
        ekf,
        RegisterEventHandler(OnProcessExit(
            target_action=ekf,
            on_exit=[EmitEvent(event=Shutdown(
                reason='O filtro de odometria encerrou.'))])),
    ]

    if _enabled(context, 'enable_vision'):
        actions.extend([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('camera'), 'launch', 'camera.launch.py'])),
                launch_arguments={
                    'rectify': 'true',
                    'framerate': LaunchConfiguration('camera_framerate'),
                }.items()),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('apriltag'), 'launch',
                    'apriltag.launch.py'])),
                launch_arguments={
                    'image_topic': LaunchConfiguration('image_topic'),
                    'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                    'base_frame': LaunchConfiguration('base_frame'),
                }.items()),
        ])

    if _enabled(context, 'enable_navigation'):
        map_file = _resolve_map_file(LaunchConfiguration('map').perform(context))
        localization_params = PathJoinSubstitution([
            FindPackageShare('bringup'), 'config', 'amcl_localization.yaml'])
        navigation_params = PathJoinSubstitution([
            FindPackageShare('bringup'), 'config',
            'nav2_navigation_light.yaml'])
        actions.extend([
            # Scope both includes because they declare generic names such as
            # params_file. Without this, AMCL's YAML leaks into Nav2.
            GroupAction(scoped=True, actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(PathJoinSubstitution([
                        FindPackageShare('nav2_bringup'), 'launch',
                        'localization_launch.py'])),
                    launch_arguments={
                        'map': map_file,
                        'params_file': localization_params,
                        'use_sim_time': 'false',
                        'autostart': 'true',
                        # nav2_bringup evaluates this in a PythonExpression.
                        'use_composition': 'False',
                        'use_respawn': 'False',
                        'log_level': 'info',
                    }.items()),
            ]),
            GroupAction(scoped=True, actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(PathJoinSubstitution([
                        FindPackageShare('bringup'), 'launch',
                        'navigation.launch.py'])),
                    launch_arguments={
                        'params_file': navigation_params,
                        'use_sim_time': 'false',
                        'autostart': 'true',
                        'log_level': 'info',
                    }.items()),
            ]),
        ])

    if manipulation_enabled:
        controller_ready = Node(
            package='so_arm_101_bringup', executable='wait_for_controllers',
            output='screen', parameters=[{'timeout_sec': 60.0}])
        manipulation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('manipulation'), 'launch',
                'manipulation.launch.py'])))

        def start_manipulation(event, launch_context):
            del launch_context
            if event.returncode != 0:
                return _shutdown(
                    'Controllers do braço não ficaram ativos para o MoveIt.')
            return move_group_entities + [manipulation]

        actions.extend([
            controller_ready,
            RegisterEventHandler(OnProcessExit(
                target_action=controller_ready,
                on_exit=start_manipulation)),
        ])

    if _enabled(context, 'enable_mission'):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('mission_manager'), 'launch',
                'mission_manager.launch.py']))))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_vision', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'enable_navigation', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'enable_manipulation', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'enable_mission', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'map', default_value='arena',
            description='Nome de um mapa instalado, sem caminho ou extensão.'),
        DeclareLaunchArgument('camera_framerate', default_value='15.0'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_rect'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('base_frame', default_value='arm_base_link'),
        OpaqueFunction(function=_launch_setup),
    ])
