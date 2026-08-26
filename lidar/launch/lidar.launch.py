"""Inicia o LiDAR físico usando exclusivamente config/lidar.yaml."""

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    driver = Node(
        package='lidar',
        executable='lidar_node',
        output='screen',
        # pyserial e libgpiod são dependências apt/rosdep deste pacote.
        prefix='/usr/bin/python3',
        parameters=[PathJoinSubstitution([
            FindPackageShare('lidar'), 'config', 'lidar.yaml'])],
    )
    return LaunchDescription([
        driver,
        RegisterEventHandler(OnProcessExit(
            target_action=driver,
            on_exit=[EmitEvent(event=Shutdown(
                reason='O driver físico do LiDAR encerrou.'))],
        )),
    ])
