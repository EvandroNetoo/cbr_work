"""Launch only RViz for a running robot or simulation."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare('so_arm_101_bringup')
    return LaunchDescription([
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', PathJoinSubstitution([
                bringup_share, 'config', 'display.rviz'])],
        ),
    ])
