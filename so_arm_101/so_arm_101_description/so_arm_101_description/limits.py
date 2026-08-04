"""Load the model joint limits used by the SO-ARM-101 packages."""

from pathlib import Path

import yaml


def _limits_file() -> Path:
    """Resolve the installed share file, with a source-tree fallback."""
    try:
        from ament_index_python.packages import PackageNotFoundError
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return Path(__file__).resolve().parents[1] / "config" / "joint_limits.yaml"
    try:
        return (
            Path(get_package_share_directory("so_arm_101_description"))
            / "config"
            / "joint_limits.yaml"
        )
    except PackageNotFoundError:
        return Path(__file__).resolve().parents[1] / "config" / "joint_limits.yaml"


def load_joint_limits() -> dict[str, dict[str, float]]:
    """Return position/velocity/effort limits keyed by ROS joint name."""
    with _limits_file().open(encoding="utf-8") as limits_file:
        return yaml.safe_load(limits_file)["joint_limits"]


def position_limits() -> dict[str, tuple[float, float]]:
    """Return inclusive position bounds for command limiting."""
    return {
        name: (float(values["min_position"]), float(values["max_position"]))
        for name, values in load_joint_limits().items()
    }
