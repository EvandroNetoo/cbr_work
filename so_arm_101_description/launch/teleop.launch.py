"""Launch the keyboard interface used by simulation and real hardware."""

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create the keyboard node that talks directly to ros2_control."""
    keyboard = Node(
        package='so_arm_101_description',
        executable='keyboard_teleop',
        output='screen',
        emulate_tty=True,
        prefix='gnome-terminal --wait --',
    )

    return LaunchDescription([keyboard])
