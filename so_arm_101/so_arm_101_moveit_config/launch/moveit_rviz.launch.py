"""Start the MoveIt RViz panel for the complete CBR robot."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

from so_arm_101_moveit_config.configuration import get_combined_moveit_config


def generate_launch_description():
    moveit_config = get_combined_moveit_config()
    rviz_config = PathJoinSubstitution([
        FindPackageShare('so_arm_101_moveit_config'),
        'config', 'moveit_robot.rviz',
    ])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='moveit_rviz',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        GroupAction(actions=[
            SetParameter(
                name='use_sim_time',
                value=LaunchConfiguration('use_sim_time')),
            rviz,
        ]),
    ])
