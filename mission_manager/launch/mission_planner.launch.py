"""Separate static Nav2 planner for mission generation, also usable on the PC."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

from mission_manager.loaders import load_arena


def _launch(context):
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    if not namespace:
        raise ValueError('O planner de missão exige um namespace separado da navegação.')
    map_path = Path(LaunchConfiguration('map').perform(context)).expanduser().resolve()
    mask_argument = LaunchConfiguration('keepout_mask').perform(context)
    mask = Path(mask_argument).expanduser().resolve() if mask_argument else map_path.with_name(
        map_path.stem + '_keepout.yaml')
    for path in (map_path, mask):
        if not path.is_file():
            raise ValueError(f'Arquivo não encontrado: {path}. Informe map:= e keepout_mask:= locais.')
    arena = load_arena(LaunchConfiguration('arena').perform(context))
    params = ParameterFile(RewrittenYaml(
        source_file=LaunchConfiguration('params_file'), root_key=namespace,
        param_rewrites={'global_frame': arena.frame_id, 'map_topic': '/' + namespace + '/map',
                        'filter_info_topic': '/' + namespace + '/keepout_costmap_filter_info'},
        convert_types=True), allow_substs=True)
    prefix = '/' + namespace
    tf = [('/tf', prefix + '/tf'), ('/tf_static', prefix + '/tf_static')]
    nodes = []
    for name, filename, topic in (
            ('map_server', map_path, 'map'),
            ('keepout_filter_mask_server', mask, 'keepout_filter_mask')):
        nodes.append(Node(
            package='nav2_map_server', executable='map_server', name=name,
            namespace=namespace, output='screen', parameters=[{
                'yaml_filename': str(filename), 'topic_name': topic,
                'frame_id': arena.frame_id, 'use_sim_time': False}], remappings=tf))
    nodes.extend([
        Node(package='nav2_map_server', executable='costmap_filter_info_server',
             name='keepout_costmap_filter_info_server', namespace=namespace, output='screen',
             parameters=[{'type': 0, 'filter_info_topic': 'keepout_costmap_filter_info',
                          'mask_topic': prefix + '/keepout_filter_mask', 'base': 0.0,
                          'multiplier': 1.0, 'use_sim_time': False}], remappings=tf),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='mission_planning_pose', namespace=namespace, output='screen', remappings=tf,
             arguments=['--x', str(arena.start.x_m), '--y', str(arena.start.y_m),
                        '--yaw', str(arena.start.yaw_rad), '--frame-id', arena.frame_id,
                        '--child-frame-id', 'mission_planning_base']),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             namespace=namespace, output='screen', parameters=[params, {'use_sim_time': False}],
             remappings=tf + [('map', prefix + '/map'),
                              ('keepout_costmap_filter_info', prefix + '/keepout_costmap_filter_info')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager', namespace=namespace, output='screen', parameters=[{
                 'autostart': True, 'use_sim_time': False,
                 'node_names': ['map_server', 'keepout_filter_mask_server',
                                'keepout_costmap_filter_info_server', 'planner_server']}]),
    ])
    return nodes


def generate_launch_description():
    share = FindPackageShare('mission_manager')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='mission_planning'),
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution([
            FindPackageShare('bringup'), 'maps', 'arena.yaml'])),
        DeclareLaunchArgument('keepout_mask', default_value='',
                              description='Padrão: <nome_do_mapa>_keepout.yaml na mesma pasta'),
        DeclareLaunchArgument('arena', default_value=PathJoinSubstitution([share, 'config', 'arena.yaml'])),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            share, 'config', 'mission_planner.yaml'])),
        OpaqueFunction(function=_launch),
    ])
