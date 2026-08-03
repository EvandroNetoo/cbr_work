"""Start Gazebo, MoveIt move_group and MoveIt RViz."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare('so_arm_101_bringup')
    moveit_share = FindPackageShare('so_arm_101_moveit_config')
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            bringup_share, 'launch', 'sim.launch.py'])))
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            moveit_share, 'launch', 'move_group.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items())
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            moveit_share, 'launch', 'moveit_rviz.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items())

    # MoveIt must receive Gazebo's current joint state before it builds its
    # planning scene. The action check also prevents Execute from racing the
    # ros2_control controller startup.
    wait_for_robot = Node(
        package='so_arm_101_moveit_config',
        executable='wait_for_robot',
        output='screen',
    )
    start_moveit_when_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_robot,
            on_exit=[move_group, rviz],
        )
    )

    # Register the exit handler before starting the short-lived waiter.
    return LaunchDescription([
        simulation,
        start_moveit_when_ready,
        wait_for_robot,
    ])
