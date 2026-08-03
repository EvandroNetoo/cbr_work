from pathlib import Path

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
from moveit_configs_utils import MoveItConfigsBuilder


def require_execution_plugin():
    """Fail early when MoveIt cannot forward plans to ros2_control."""
    try:
        get_package_prefix('moveit_simple_controller_manager')
    except PackageNotFoundError as error:
        raise RuntimeError(
            'MoveIt execution plugin is missing. Install it with: '
            'sudo apt install ros-jazzy-moveit-simple-controller-manager'
        ) from error


def get_moveit_config():
    require_execution_plugin()
    description_share = Path(
        get_package_share_directory('so_arm_101_description'))
    urdf = description_share / 'urdf' / 'so_101.urdf.xacro'

    return (
        MoveItConfigsBuilder(
            'so_arm_101', package_name='so_arm_101_moveit_config')
        .robot_description(file_path=urdf)
        .robot_description_semantic('config/so_arm_101.srdf')
        .robot_description_kinematics('config/kinematics.yaml')
        .joint_limits('config/joint_limits.yaml')
        .planning_pipelines(
            default_planning_pipeline='ompl',
            pipelines=['ompl'],
            load_all=False)
        .trajectory_execution(
            'config/moveit_controllers.yaml',
            moveit_manage_controllers=False)
        .to_moveit_configs()
    )
