import importlib.util
import os
from pathlib import Path

from launch import LaunchContext


PACKAGE_ROOT = Path(__file__).parents[1]


def _driver_launch_module():
    path = PACKAGE_ROOT / 'launch' / 'driver.launch.py'
    spec = importlib.util.spec_from_file_location('so101_driver_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context(module, **values):
    os.environ['ROS_LOG_DIR'] = '/tmp/launch_parameter_tests'
    context = LaunchContext()
    defaults = {
        'port': module.CONFIG_DEFAULT,
        'robot_id': module.CONFIG_DEFAULT,
        'buffer_commands': module.CONFIG_DEFAULT,
        'deduplicate_commands': module.CONFIG_DEFAULT,
    }
    defaults.update(values)
    context.launch_configurations.update(defaults)
    return context


def test_launch_defaults_do_not_override_real_yaml():
    module = _driver_launch_module()
    assert module._optional_parameter_overrides(_context(module)) == {}


def test_explicit_cli_values_override_real_yaml_with_native_types():
    module = _driver_launch_module()
    overrides = module._optional_parameter_overrides(_context(
        module,
        port='/dev/ttyUSB4',
        robot_id='outro_robo',
        buffer_commands='false',
        deduplicate_commands='true',
    ))
    assert overrides == {
        'port': '/dev/ttyUSB4',
        'robot_id': 'outro_robo',
        'buffer_commands': False,
        'deduplicate_commands': True,
    }


def test_all_arm_wrappers_propagate_config_default_sentinel():
    paths = [
        PACKAGE_ROOT / 'launch' / 'real.launch.py',
        PACKAGE_ROOT.parent / 'so_arm_101_bringup' / 'launch' / 'real.launch.py',
        PACKAGE_ROOT.parent / 'so_arm_101_moveit_config' / 'launch'
        / 'real_planning.launch.py',
        PACKAGE_ROOT.parents[1] / 'bringup' / 'launch' / 'robot.launch.py',
    ]
    for path in paths:
        source = path.read_text()
        assert "CONFIG_DEFAULT = '__from_config__'" in source
        assert "DeclareLaunchArgument('port', default_value=CONFIG_DEFAULT)" in source
