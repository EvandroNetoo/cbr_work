from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def get_moveit_config():
    return (
        MoveItConfigsBuilder(
            'so_101', package_name='so_arm_101_moveit_config')
        .robot_description(mappings={
            'arm_base_link_name': 'arm_base_link',
        })
        .robot_description_semantic(
            file_path='config/so_arm_101.srdf',
            mappings={
                'robot_name': 'so_101',
                'arm_base_link_name': 'arm_base_link',
            },
        )
        .planning_pipelines(
            default_planning_pipeline='ompl',
            pipelines=['ompl'],
            load_all=False)
        .trajectory_execution(
            moveit_manage_controllers=False)
        .to_moveit_configs()
    )


def get_combined_moveit_config():
    """Use the same arm planning setup with the composed mobile robot URDF."""
    description = (
        get_package_share_directory('robot_description')
        + '/urdf/robot.urdf.xacro'
    )
    return (
        MoveItConfigsBuilder(
            'robot', package_name='so_arm_101_moveit_config')
        .robot_description(file_path=description)
        .robot_description_semantic(
            file_path='config/so_arm_101.srdf',
            mappings={
                'robot_name': 'robot',
                'arm_base_link_name': 'arm_base_link',
            },
        )
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .planning_pipelines(
            default_planning_pipeline='ompl', pipelines=['ompl'], load_all=False)
        .trajectory_execution(moveit_manage_controllers=False)
        .to_moveit_configs()
    )
