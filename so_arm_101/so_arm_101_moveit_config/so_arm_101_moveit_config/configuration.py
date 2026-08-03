from moveit_configs_utils import MoveItConfigsBuilder


def get_moveit_config():
    return (
        MoveItConfigsBuilder(
            'so_101', package_name='so_arm_101_moveit_config')
        .planning_pipelines(
            default_planning_pipeline='ompl',
            pipelines=['ompl'],
            load_all=False)
        .trajectory_execution(
            moveit_manage_controllers=False)
        .to_moveit_configs()
    )
