"""Checks for clear diagnostics when system MoveIt plugins are absent."""
from pathlib import Path
import sys
from unittest.mock import patch

import pytest
from ament_index_python.packages import PackageNotFoundError

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from so_arm_101_moveit_config.configuration import require_execution_plugin


def test_missing_controller_plugin_reports_install_command():
    with patch(
        'so_arm_101_moveit_config.configuration.get_package_prefix',
        side_effect=PackageNotFoundError('moveit_simple_controller_manager'),
    ):
        with pytest.raises(RuntimeError) as error:
            require_execution_plugin()

    assert 'ros-jazzy-moveit-simple-controller-manager' in str(error.value)
