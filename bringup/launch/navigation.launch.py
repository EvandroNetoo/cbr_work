"""Start the CBR Nav2 graph with a small, deterministic DDS footprint.

The generic Jazzy navigation launch starts ten independent processes. On the
physical CBR graph that fragmented Fast DDS discovery and starved TF. Its
composed variant also loads docking_server, whose unstamped ``cmd_vel`` type is
incompatible with the stamped command chain used by this robot.

This launch deliberately contains only the servers used by
``navigate_to_pose_safe.xml`` plus the output safety chain. The four core
servers share one container; the lifecycle and collision-monitor processes are
isolated to avoid lifecycle/destruction races, totaling three DDS participants.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    fastdds_profile = PathJoinSubstitution([
        FindPackageShare('bringup'), 'config', 'fastdds_nav2.xml'])
    default_params_file = PathJoinSubstitution([
        FindPackageShare('bringup'), 'config', 'nav2_navigation_light.yaml'])
    navigate_to_pose_bt = PathJoinSubstitution([
        FindPackageShare('bringup'), 'config', 'navigate_to_pose_safe.xml'])

    configured_params = ParameterFile(params_file, allow_substs=True)
    node_parameters = [configured_params, {'use_sim_time': use_sim_time}]
    tf_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    cmd_vel_remapping = [('cmd_vel', 'cmd_vel_nav')]
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
    ]

    container = ComposableNodeContainer(
        name='cbr_nav2_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        # controller_server e planner_server criam os costmaps como nós filhos.
        # O arquivo no processo garante que essas seções aninhadas também sejam
        # resolvidas, além dos parâmetros passados a cada componente abaixo.
        parameters=node_parameters,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=node_parameters,
                remappings=tf_remappings + cmd_vel_remapping,
            ),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=node_parameters,
                remappings=tf_remappings,
            ),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=node_parameters,
                remappings=tf_remappings + cmd_vel_remapping,
            ),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                # O caminho e resolvido no share instalado, mantendo o YAML
                # portavel entre workspace, notebook e Banana Pi.
                parameters=node_parameters + [{
                    'default_nav_to_pose_bt_xml': navigate_to_pose_bt,
                }],
                remappings=tf_remappings,
            ),
        ],
    )
    # Jazzy collision_monitor encerra com segfault quando destruído dentro de
    # um container junto aos costmaps. Isolá-lo mantém o caminho de segurança
    # e ainda deixa o grafo inteiro em apenas três participantes DDS.
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=node_parameters,
        remappings=tf_remappings,
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'autostart': autostart,
            'use_sim_time': use_sim_time,
            'node_names': lifecycle_nodes,
        }],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        # Fast DDS 2.x usa FASTRTPS_*; versões novas usam FASTDDS_*. Definir
        # ambas mantém o launch compatível sem afetar outros processos ROS.
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE', fastdds_profile),
        SetEnvironmentVariable(
            'FASTDDS_DEFAULT_PROFILES_FILE', fastdds_profile),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Arquivo YAML de parâmetros Nav2 da CBR.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        container,
        collision_monitor,
        lifecycle_manager,
    ])
