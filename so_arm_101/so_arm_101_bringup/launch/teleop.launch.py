"""Launch keyboard teleoperation after an external driver is running."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='so_arm_101_teleop', executable='keyboard_teleop',
            output='screen', emulate_tty=True,
            prefix='gnome-terminal --wait --'),
    ])
