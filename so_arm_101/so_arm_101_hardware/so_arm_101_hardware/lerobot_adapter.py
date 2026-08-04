"""Small, testable adapter around LeRobot's SO-101 follower API."""

from __future__ import annotations

from typing import Mapping


LEROBOT_JOINTS = (
    'shoulder_pan', 'shoulder_lift', 'elbow_flex',
    'wrist_flex', 'wrist_roll', 'gripper',
)

# ROS uses the names from the URDF; LeRobot uses motor feature names.
ROS_TO_LEROBOT = {
    'base_link_to_link1': 'shoulder_pan',
    'link1_to_link2': 'shoulder_lift',
    'link2_to_link3': 'elbow_flex',
    'link3_to_link4': 'wrist_flex',
    'link4_to_link5': 'wrist_roll',
    'right_clamp': 'gripper',
}
LEROBOT_TO_ROS = {value: key for key, value in ROS_TO_LEROBOT.items()}


def make_follower(port: str, robot_id: str, *, use_degrees: bool = False):
    """Build the official LeRobot follower, keeping imports out of module load."""
    try:
        from lerobot.robots.so_follower import (  # type: ignore
            SO101Follower,
            SO101FollowerConfig,
        )
    except ImportError as error:
        raise RuntimeError(
            "LeRobot não está instalado. Instale com: "
            "pip install 'lerobot[feetech]'"
        ) from error

    config = SO101FollowerConfig(
        port='/dev/ttyUSB1',
        id=robot_id,
        use_degrees=use_degrees,
    )
    return SO101Follower(config)


def observation_to_ros(observation: Mapping[str, object], *, use_degrees: bool):
    """Convert a LeRobot observation into ROS joint-name/value pairs."""
    scale = 3.141592653589793 / 180.0 if use_degrees else 1.0
    result = {}
    for motor_name in LEROBOT_JOINTS:
        key = f'{motor_name}.pos'
        if key in observation:
            result[LEROBOT_TO_ROS[motor_name]] = float(observation[key]) * scale
    return result


def ros_to_action(values: Mapping[str, float], *, use_degrees: bool):
    """Convert ROS joint positions (radians) to a LeRobot action mapping."""
    scale = 180.0 / 3.141592653589793 if use_degrees else 1.0
    return {
        f"{ROS_TO_LEROBOT[name]}.pos": float(value) * scale
        for name, value in values.items()
        if name in ROS_TO_LEROBOT
    }
