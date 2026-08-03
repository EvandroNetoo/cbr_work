"""Start Gazebo, MoveIt move_group and MoveIt RViz."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
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

    return LaunchDescription([
        simulation,
        move_group,
        rviz,
    ])
