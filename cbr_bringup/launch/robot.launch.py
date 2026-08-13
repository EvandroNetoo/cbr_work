"""Run all currently available embedded robot services on the Banana Pi."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    moveit_share = FindPackageShare('so_arm_101_moveit_config')
    camera_share = FindPackageShare('cbr_camera')
    apriltag_share = FindPackageShare('cbr_apriltag')
    port = LaunchConfiguration('port')
    robot_id = LaunchConfiguration('robot_id')
    calibration_file = LaunchConfiguration('calibration_file')
    hardware_state_timeout = LaunchConfiguration('hardware_state_timeout')
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value=''),
        DeclareLaunchArgument('robot_id', default_value='so101_follower'),
        DeclareLaunchArgument(
            'hardware_state_timeout', default_value='45.0'),
        DeclareLaunchArgument('calibration_file', default_value=PathJoinSubstitution([
            FindPackageShare('so_arm_101_hardware'), 'config',
            'so101_follower.json'])),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_rect'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                camera_share, 'launch', 'camera.launch.py'])),
            launch_arguments={'rectify': 'true'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                moveit_share, 'launch', 'real_planning.launch.py'])),
            launch_arguments={
                'port': port,
                'robot_id': robot_id,
                'calibration_file': calibration_file,
                'hardware_state_timeout': hardware_state_timeout,
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                apriltag_share, 'launch', 'apriltag.launch.py'])),
            launch_arguments={
                'image_topic': LaunchConfiguration('image_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'base_frame': LaunchConfiguration('base_frame'),
            }.items()),
    ])
