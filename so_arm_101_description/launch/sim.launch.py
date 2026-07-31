"""Launch the SO-ARM-101 robot in Gazebo."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_name = 'so_arm_101_description'
    pkg_share_path = get_package_share_directory(package_name)
    pkg_share = FindPackageShare(package_name)

    # Gazebo resolves package:// mesh URIs as model://<package>/...
    # Its resource path must therefore contain the parent of the package share.
    gazebo_resource_path = os.pathsep.join(filter(None, [
        os.path.dirname(pkg_share_path),
        os.environ.get('GZ_SIM_RESOURCE_PATH'),
    ]))
    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gazebo_resource_path,
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        choices=['true', 'false'],
        description='Run only the Gazebo server, without its graphical client',
    )
    headless = LaunchConfiguration('headless')

    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'so_101.urdf.xacro'])
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '180',
        ],
    )
    spawner_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '180',
        ],
    )
    spawner_gripper = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '180',
        ],
    )

    spawn_controllers = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[spawner_jsb, spawner_arm, spawner_gripper],
        ),
    )

    world = PathJoinSubstitution([pkg_share, 'worlds', 'empty.sdf'])
    gz_flags = PythonExpression([
        "('-r -s ' if '", headless, "' == 'true' else '-r ')",
        " + '--physics-engine gz-physics-bullet-featherstone-plugin '",
    ])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py',
            ])
        ),
        launch_arguments={'gz_args': [gz_flags, world]}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'so_101',
            '-z', '0.001',
        ],
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    return LaunchDescription([
        set_gazebo_resource_path,
        headless_arg,
        robot_state_publisher,
        spawn_controllers,
        gazebo,
        spawn_robot,
        clock_bridge,
    ])
