"""Inicia a IMU física e a fusão planar com a odometria das rodas."""

from launch import LaunchDescription
from launch.actions import EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('cbr_imu'), 'launch', 'imu.launch.py'])))
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('cbr_imu'), 'config', 'ekf.yaml'])],
        remappings=[('odometry/filtered', '/odom')],
    )
    return LaunchDescription([
        imu,
        ekf,
        RegisterEventHandler(OnProcessExit(
            target_action=ekf,
            on_exit=[EmitEvent(event=Shutdown(
                reason='O filtro de odometria encerrou.'))],
        )),
    ])
