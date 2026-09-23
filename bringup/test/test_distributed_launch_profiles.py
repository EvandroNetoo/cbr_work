"""Contracts for the Banana Pi and Raspberry Pi launch profiles."""

import importlib.util
from pathlib import Path

from launch import LaunchContext


PACKAGE_ROOT = Path(__file__).parents[1]
LAUNCH_DIR = PACKAGE_ROOT / 'launch'


def _source(name):
    return (LAUNCH_DIR / name).read_text()


def _load_processing_module():
    path = LAUNCH_DIR / 'processing.launch.py'
    spec = importlib.util.spec_from_file_location('processing_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_hardware_profile_contains_only_hardware_and_local_control():
    source = _source('hardware.launch.py')

    for package in ('so_arm_101_hardware', 'base_hardware', 'lidar', 'imu',
                    'vl53_distance', 'controller_manager'):
        assert f"FindPackageShare('{package}')" in source or (
            f"package='{package}'" in source)
    for forbidden in ('camera', 'apriltag', 'robot_localization',
                      'robot_state_publisher', 'move_group', 'manipulation',
                      'nav2_', 'mission_manager'):
        assert forbidden not in source

    assert source.count("executable='ros2_control_node'") == 1
    assert "executable='publish_robot_description'" in source
    assert 'target_action=description_publisher' in source
    assert "arguments=['joint_state_broadcaster'" in source
    assert "arguments=['arm_controller'" in source
    assert "arguments=['gripper_controller'" in source
    assert "'base_controller', '-c', '/controller_manager'" in source
    assert 'OnProcessExit(target_action=base, on_exit=start_vl53)' in source


def test_processing_profile_exposes_component_selection():
    source = _source('processing.launch.py')

    assert "'components', default_value='all'" in source
    assert "'disable_components', default_value=''" in source
    for legacy_argument in (
            'enable_vision', 'enable_navigation', 'enable_manipulation',
            'enable_mission'):
        assert legacy_argument not in source

    assert "'map', default_value='arena'" in source
    assert "'camera_framerate', default_value='15.0'" in source
    assert "'image_topic', default_value='/camera/image_rect'" in source
    assert "'camera_info_topic', default_value='/camera/camera_info'" in source
    assert "'base_frame', default_value='arm_base_link'" in source
    assert "'use_composition': 'False'" in source
    assert "'config', 'amcl_localization.yaml'" in source
    assert "'nav2_navigation_light.yaml'" in source
    assert source.count('GroupAction(scoped=True') == 2


def test_processing_component_lists_default_to_everything_and_can_be_filtered(
        monkeypatch, tmp_path):
    module = _load_processing_module()
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    context = LaunchContext()
    context.launch_configurations.update({
        'components': 'all',
        'disable_components': 'mission',
    })

    selected = module._selected_components(context)

    assert selected == set(module.COMPONENTS) - {'mission'}


def test_processing_component_allow_list_starts_only_requested_items(
        monkeypatch, tmp_path):
    module = _load_processing_module()
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    context = LaunchContext()
    context.launch_configurations.update({
        'components': 'moveit,manipulation',
        'disable_components': '',
    })

    assert module._selected_components(context) == {'moveit', 'manipulation'}


def test_processing_component_lists_reject_unknown_names():
    module = _load_processing_module()

    try:
        module._parse_components('mission,typo', 'components')
    except RuntimeError as error:
        assert 'typo' in str(error)
        assert 'Disponíveis' in str(error)
    else:
        raise AssertionError('Componente desconhecido foi aceito.')


def test_hardware_description_publisher_is_transient_local_and_has_no_tf():
    source = (
        PACKAGE_ROOT / 'bringup' / 'robot_description_publisher.py'
    ).read_text()

    assert "String, '/robot_description', qos" in source
    assert 'DurabilityPolicy.TRANSIENT_LOCAL' in source
    assert 'ReliabilityPolicy.RELIABLE' in source
    assert 'robot_state_publisher' not in source
    assert "'/tf'" not in source


def test_processing_profile_has_no_physical_driver_or_control_manager():
    source = _source('processing.launch.py')

    for forbidden in ("FindPackageShare('so_arm_101_hardware')",
                      "FindPackageShare('base_hardware')",
                      "FindPackageShare('lidar')",
                      "FindPackageShare('vl53_distance')",
                      "package='controller_manager'",
                      "executable='ros2_control_node'"):
        assert forbidden not in source

    assert "package='robot_state_publisher'" in source
    assert "package='robot_localization'" in source
    assert "FindPackageShare('camera')" in source
    assert "FindPackageShare('vision')" in source
    assert "FindPackageShare('nav2_bringup')" in source
    assert "'navigation.launch.py'" in source
    assert 'get_combined_moveit_config' in source
    assert "FindPackageShare('manipulation')" in source
    assert "FindPackageShare('mission_manager')" in source


def test_map_names_resolve_without_paths_or_extensions():
    module = _load_processing_module()
    maps_directory = PACKAGE_ROOT / 'maps'

    for yaml_file in maps_directory.glob('*.yaml'):
        assert module._resolve_map_file(
            yaml_file.stem, maps_directory) == str(yaml_file)

    for invalid in ('', 'arena.yaml', '../arena', '/tmp/arena'):
        try:
            module._resolve_map_file(invalid, maps_directory)
        except RuntimeError as error:
            assert "argumento 'map'" in str(error)
        else:
            raise AssertionError(f'Mapa inválido aceito: {invalid}')

    try:
        module._resolve_map_file('mapa_inexistente', maps_directory)
    except RuntimeError as error:
        assert "Mapa 'mapa_inexistente' não encontrado" in str(error)
        assert 'arena' in str(error)
    else:
        raise AssertionError('Nome de mapa inexistente foi aceito.')


def test_maps_are_installed_with_the_bringup_package():
    setup_source = (PACKAGE_ROOT / 'setup.py').read_text()
    assert "os.path.join('share', package_name, 'maps')" in setup_source
    assert "glob('maps/*.yaml')" in setup_source
    assert "glob('maps/*.pgm')" in setup_source


def test_legacy_robot_launch_remains_the_monolithic_profile():
    source = _source('robot.launch.py')
    assert "FindPackageShare('camera')" in source
    assert "FindPackageShare('vision')" in source
    assert "FindPackageShare('manipulation')" in source
    assert 'generate_move_group_launch' in source
