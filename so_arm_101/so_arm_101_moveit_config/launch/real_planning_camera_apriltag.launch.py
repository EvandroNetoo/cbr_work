"""Start the physical arm, MoveIt, wrist camera, and AprilTag detector."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


CONFIG_DEFAULT = '__from_config__'


def generate_launch_description() -> LaunchDescription:
    arm_planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('so_arm_101_moveit_config'), 'launch',
            'real_planning.launch.py',
        ])),
        launch_arguments={
            'port': LaunchConfiguration('port'),
            'robot_id': LaunchConfiguration('robot_id'),
            'calibration_file': LaunchConfiguration('calibration_file'),
            'hardware_state_timeout': LaunchConfiguration(
                'hardware_state_timeout'),
        }.items())
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('camera'), 'launch', 'camera.launch.py',
        ])),
        launch_arguments={
            'rectify': 'true',
            'framerate': LaunchConfiguration('camera_framerate'),
        }.items())
    apriltag = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('apriltag'), 'launch', 'apriltag.launch.py',
        ])),
        launch_arguments={
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'base_frame': LaunchConfiguration('base_frame'),
        }.items())

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value=CONFIG_DEFAULT),
        DeclareLaunchArgument('robot_id', default_value=CONFIG_DEFAULT),
        DeclareLaunchArgument('calibration_file', default_value=CONFIG_DEFAULT),
        DeclareLaunchArgument('hardware_state_timeout', default_value='45.0'),
        DeclareLaunchArgument('camera_framerate', default_value='15.0'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_rect'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('base_frame', default_value='arm_base_link'),
        arm_planning,
        camera,
        apriltag,
    ])
