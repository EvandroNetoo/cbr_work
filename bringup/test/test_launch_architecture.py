from pathlib import Path

import yaml


PACKAGE = Path(__file__).parents[1]
LAUNCH = PACKAGE / 'launch'


def source(name):
    return (LAUNCH / name).read_text()


def test_launch_set_is_small_and_explicit():
    assert {path.name for path in LAUNCH.glob('*.launch.py')} == {
        'robot.launch.py', 'hardware.launch.py', 'sensors.launch.py',
        'localization.launch.py', 'perception.launch.py',
        'manipulation.launch.py', 'workstation.launch.py'}


def test_only_central_bringup_installs_launch_files():
    repository = PACKAGE.parent
    setup_files = list(repository.glob('*/setup.py'))
    setup_files += list((repository / 'so_arm_101').glob('*/setup.py'))
    for setup_file in setup_files:
        if setup_file == PACKAGE / 'setup.py':
            continue
        assert "glob('launch/*.launch.py')" not in setup_file.read_text()


def test_robot_defaults_are_complete_and_subsystems_are_coarse():
    robot = source('robot.launch.py')
    for argument in ('enable_base', 'enable_arm', 'enable_perception', 'enable_moveit'):
        assert f"'{argument}', default_value='true'" in robot
    for child in ('hardware', 'sensors', 'localization', 'perception', 'manipulation'):
        assert f"_include('{child}.launch.py'" in robot


def test_one_control_and_state_publisher_path():
    all_embedded = '\n'.join(source(path.name) for path in LAUNCH.glob('*.launch.py')
                             if path.name != 'workstation.launch.py')
    assert all_embedded.count("executable='ros2_control_node'") == 1
    assert all_embedded.count("executable='robot_state_publisher'") == 1
    assert 'wait_for_' not in all_embedded
    assert 'TimerAction' not in all_embedded


def test_spawners_are_native_and_parallel_not_chained():
    hardware = source('hardware.launch.py')
    assert "executable='spawner'" in hardware
    assert "'--controller-manager-timeout', timeout" in hardware
    assert 'OnProcessExit' in hardware
    assert 'next_after' not in hardware


def test_readiness_gates_only_actuator_state_not_imu():
    readiness = (PACKAGE / 'bringup' / 'hardware_readiness.py').read_text()
    assert "'/so101_hardware/raw_joint_states'" in readiness
    assert "'/base_hardware/raw_joint_states'" in readiness
    assert 'Imu' not in readiness
    assert 'sleep' not in readiness


def test_workstation_has_no_autonomous_nodes():
    workstation = source('workstation.launch.py')
    assert "package='rviz2'" in workstation
    assert "executable='keyboard_teleop'" in workstation
    for forbidden in ('ros2_control_node', 'robot_state_publisher', 'move_group',
                      'apriltag_detector', 'ekf_node', 'lidar_node', 'imu_node'):
        assert forbidden not in workstation


def test_main_rviz_observes_robot_and_moveit():
    config = yaml.safe_load((PACKAGE / 'config' / 'telemetry.rviz').read_text())
    displays = config['Visualization Manager']['Displays']
    classes = {display['Class'] for display in displays}
    assert 'rviz_default_plugins/RobotModel' in classes
    assert 'rviz_default_plugins/LaserScan' in classes
    assert 'rviz_default_plugins/Odometry' in classes
    assert 'moveit_rviz_plugin/MotionPlanning' in classes
