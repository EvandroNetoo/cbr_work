"""Run the complete CBR robot against a physical Gazebo Sim arena."""

from pathlib import Path
import os

from ament_index_python.packages import (
    PackageNotFoundError, get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
    RegisterEventHandler, EmitEvent, ExecuteProcess,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _check_runtime_packages(components):
    requested = set(components.split(','))
    if 'all' in requested:
        requested = {'ekf', 'vision', 'localization', 'navigation',
                     'moveit', 'manipulation', 'mission'}
    required = {'robot_localization'} if 'ekf' in requested else set()
    if {'localization', 'navigation'} & requested:
        required.update({'nav2_bringup', 'nav2_amcl', 'nav2_controller',
                         'nav2_bt_navigator', 'nav2_planner'})
    if 'mission' in requested:
        required.add('nav2_msgs')
    missing = []
    for package in sorted(required):
        try:
            get_package_share_directory(package)
        except PackageNotFoundError:
            missing.append(package)
    if missing:
        raise RuntimeError(
            'Dependências ROS ausentes para os componentes selecionados: '
            + ', '.join(missing) + '. Instale navigation2, nav2-bringup e '
            'robot-localization do ROS Jazzy para a missão completa.')


def _setup(context):
    _check_runtime_packages(
        LaunchConfiguration('processing_components').perform(context))
    share = Path(get_package_share_directory('cbr_simulation'))
    robot_share = Path(get_package_share_directory('robot_description'))
    bringup_share = Path(get_package_share_directory('bringup'))
    vl53_share = Path(get_package_share_directory('vl53_distance'))
    controllers = str(share / 'config/controllers.yaml')
    description = xacro.process_file(
        str(robot_share / 'urdf/robot.urdf.xacro'), mappings={
            'use_gz_ros2_control': 'true',
            'use_real_ros2_control': 'false',
            'hardware_plugin': 'gz_ros2_control/GazeboSimSystem',
            'controllers_file': controllers,
        }).toxml()
    world = str(share / 'worlds/arena.sdf')
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    gz_args = f'-r {world}' if gui else f'-r -s {world}'
    world_name = 'arena'
    robot_x = LaunchConfiguration('x').perform(context)
    robot_y = LaunchConfiguration('y').perform(context)
    robot_yaw = LaunchConfiguration('yaw').perform(context)

    resource_paths = [
        str(share.parent),
        str(Path(get_package_share_directory('so_arm_101_description')).parent),
    ]
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', *gz_args.split()], output='screen',
        additional_env={
            'GZ_SIM_RESOURCE_PATH': os.pathsep.join(resource_paths + [
                os.environ.get('GZ_SIM_RESOURCE_PATH', '')]),
            'GZ_SIM_SYSTEM_PLUGIN_PATH': os.pathsep.join([
                '/opt/ros/jazzy/lib',
                os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
            ]),
        })
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='gazebo_bridge', output='screen',
        parameters=[{'config_file': str(share / 'config/bridge.yaml'),
                     'use_sim_time': True}])
    scan_filter = Node(
        package='cbr_simulation', executable='simulated_xv11',
        output='screen', parameters=[{'use_sim_time': True}])
    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': description, 'use_sim_time': True}])
    spawn = Node(
        package='ros_gz_sim', executable='create', name='spawn_cbr',
        arguments=['-world', world_name, '-string', description,
                   '-name', 'cbr', '-x', robot_x, '-y', robot_y,
                   '-z', '0.03', '-Y', robot_yaw],
        output='screen')
    joint_broadcaster = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen')
    arm = Node(package='controller_manager', executable='spawner',
               arguments=['arm_controller', '-c', '/controller_manager'],
               output='screen')
    gripper = Node(package='controller_manager', executable='spawner',
                   arguments=['gripper_controller', '-c', '/controller_manager'],
                   output='screen')
    base = Node(
        package='controller_manager', executable='spawner',
        arguments=['base_controller', '-c', '/controller_manager',
                   '--controller-ros-args', '--remap ~/reference:=/cmd_vel',
                   '--controller-ros-args', '--remap ~/odometry:=/wheel/odom'],
        output='screen')
    vl53 = Node(
        package='vl53_distance', executable='vl53_distance_action',
        name='vl53_distance_action', output='screen',
        parameters=[str(vl53_share / 'config/vl53_distance.yaml'), {
            'sensor.source': 'gazebo',
            'sensor.left.offset_mm': 0,
            'sensor.right.offset_mm': 0,
            'use_sim_time': True,
        }])
    processing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            bringup_share / 'launch/processing.launch.py')),
        launch_arguments={
            'components': LaunchConfiguration('processing_components'),
            'disable_components': 'rsp,camera',
            'use_sim_time': 'true',
            'map': LaunchConfiguration('map'),
        }.items())

    def next_if_success(following, reason):
        def callback(event, _context):
            if event.returncode == 0:
                return [following]
            return [EmitEvent(event=Shutdown(reason=reason))]
        return callback

    return [
        gazebo, bridge, scan_filter, state_publisher, spawn,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn,
            on_exit=next_if_success(joint_broadcaster, 'Spawn do robô falhou.'))),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_broadcaster,
            on_exit=next_if_success(arm, 'Joint state broadcaster falhou.'))),
        RegisterEventHandler(OnProcessExit(
            target_action=arm,
            on_exit=next_if_success(gripper, 'Arm controller falhou.'))),
        RegisterEventHandler(OnProcessExit(
            target_action=gripper,
            on_exit=next_if_success(base, 'Gripper controller falhou.'))),
        RegisterEventHandler(OnProcessExit(
            target_action=base,
            on_exit=next_if_success(vl53, 'Base controller falhou.'))),
        processing,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('map', default_value='arena'),
        DeclareLaunchArgument('processing_components', default_value='all'),
        DeclareLaunchArgument('x', default_value='2.5'),
        DeclareLaunchArgument('y', default_value='2.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=_setup),
    ])
