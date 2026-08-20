"""Start MoveIt RViz for the standalone arm simulation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from moveit_configs_utils.launches import generate_moveit_rviz_launch

from so_arm_101_moveit_config.configuration import get_moveit_config


def generate_launch_description():
    generated = generate_moveit_rviz_launch(get_moveit_config())
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        GroupAction(actions=[
            SetParameter(
                name='use_sim_time',
                value=LaunchConfiguration('use_sim_time')),
            *generated.entities,
        ]),
    ])
