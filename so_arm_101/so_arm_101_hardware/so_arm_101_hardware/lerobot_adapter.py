"""Small, testable adapter around LeRobot's SO-101 follower API."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

import yaml

from so_arm_101_description.limits import position_limits


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

# The physical gripper motor is angular, while the ROS model represents the
# jaw motion as a prismatic joint.  These are the calibrated endpoints of the
# physical motor and the corresponding URDF displacement.
def _load_gripper_calibration() -> dict[str, float]:
    """Load the physical servo endpoint configuration."""
    try:
        from ament_index_python.packages import PackageNotFoundError
        from ament_index_python.packages import get_package_share_directory

        try:
            calibration_file = (
                Path(get_package_share_directory('so_arm_101_hardware'))
                / 'config' / 'gripper_calibration.yaml')
        except PackageNotFoundError:
            calibration_file = (
                Path(__file__).resolve().parents[1]
                / 'config' / 'gripper_calibration.yaml')
    except ImportError:
        calibration_file = (
            Path(__file__).resolve().parents[1]
            / 'config' / 'gripper_calibration.yaml')
    if not calibration_file.is_file():
        calibration_file = (
            Path(__file__).resolve().parents[1]
            / 'config' / 'gripper_calibration.yaml')
    with calibration_file.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)['gripper']


GRIPPER_CALIBRATION = _load_gripper_calibration()
GRIPPER_CLOSED_ANGLE_RAD = float(
    GRIPPER_CALIBRATION['motor_closed_position_rad'])
GRIPPER_OPEN_ANGLE_RAD = float(
    GRIPPER_CALIBRATION['motor_open_position_rad'])
GRIPPER_CLOSED_POSITION_M, GRIPPER_OPEN_POSITION_M = position_limits()['right_clamp']


@dataclass(frozen=True)
class JointCalibration:
    """Relationship between a LeRobot motor angle and a URDF joint angle.

    The URDF is the ROS source of truth.  Motor orientation and the mechanical
    zero are handled only at this hardware boundary, in both directions.
    """

    sign: int = 1
    offset: float = 0.0

    def __post_init__(self):
        if self.sign not in (-1, 1):
            raise ValueError('Joint calibration sign must be -1 or 1.')


# q_ros = sign * q_lerobot + offset
#
# These values describe the motor directions relative to the ROS/URDF joints.
# The ROS zero is intentionally the LeRobot calibration zero for every joint;
# geometric zero corrections belong in the URDF joint origins, not here.
JOINT_CALIBRATION = {
    'base_link_to_link1': JointCalibration(sign=-1),
    'link1_to_link2': JointCalibration(sign=-1),
    'link2_to_link3': JointCalibration(sign=-1),
    'link3_to_link4': JointCalibration(sign=-1),
    'link4_to_link5': JointCalibration(sign=1),
    'right_clamp': JointCalibration(sign=1),
}


def _motor_to_ros(joint_name: str, value: float) -> float:
    calibration = JOINT_CALIBRATION[joint_name]
    return calibration.sign * value + calibration.offset


def _ros_to_motor(joint_name: str, value: float) -> float:
    calibration = JOINT_CALIBRATION[joint_name]
    return calibration.sign * (value - calibration.offset)


def _gripper_angle_to_position(angle_rad: float) -> float:
    """Convert the physical gripper angle to the URDF linear position."""
    return (
        GRIPPER_CLOSED_POSITION_M
        + (angle_rad - GRIPPER_CLOSED_ANGLE_RAD)
        / (GRIPPER_OPEN_ANGLE_RAD - GRIPPER_CLOSED_ANGLE_RAD)
        * (GRIPPER_OPEN_POSITION_M - GRIPPER_CLOSED_POSITION_M)
    )


def _gripper_position_to_angle(position_m: float) -> float:
    """Convert the URDF linear position to the physical gripper angle."""
    return (
        GRIPPER_CLOSED_ANGLE_RAD
        + (position_m - GRIPPER_CLOSED_POSITION_M)
        / (GRIPPER_OPEN_POSITION_M - GRIPPER_CLOSED_POSITION_M)
        * (GRIPPER_OPEN_ANGLE_RAD - GRIPPER_CLOSED_ANGLE_RAD)
    )


def make_follower(
    port: str,
    robot_id: str,
    *,
    use_degrees: bool = False,
    calibration_file: str = '',
):
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

    calibration_path = Path(calibration_file) if calibration_file else None
    if calibration_path is not None:
        if not calibration_path.is_file():
            raise FileNotFoundError(
                f'Arquivo de calibração não encontrado: {calibration_path}')
        # LeRobot resolves calibration as calibration_dir / id.json.
        calibration_dir = calibration_path.parent
        calibration_id = calibration_path.stem
    else:
        calibration_dir = None
        calibration_id = robot_id

    config = SO101FollowerConfig(
        port=port,
        id=calibration_id,
        calibration_dir=calibration_dir,
        use_degrees=use_degrees,
    )
    return SO101Follower(config)


def connect_follower(follower, *, disable_torque: bool) -> None:
    """Connect the follower in normal or manual-positioning mode.

    ``SOFollower.connect()`` calls ``configure()``, whose torque-disabled
    context manager always enables torque when configuration finishes.  That
    is correct for normal control, but unsafe for a launch explicitly asking
    for a freely movable arm.  In that mode, connect the public bus API,
    disable torque immediately, and keep the connection open only for state
    reads.  Motor configuration is intentionally skipped because it is not
    needed for observation and would re-enable torque.
    """
    if not disable_torque:
        follower.connect(calibrate=False)
        return

    follower.bus.connect()
    follower.bus.disable_torque(num_retry=5)
    for camera in follower.cameras.values():
        camera.connect()


def observation_to_ros(observation: Mapping[str, object], *, use_degrees: bool):
    """Convert a LeRobot observation into ROS joint-name/value pairs."""
    scale = 3.141592653589793 / 180.0 if use_degrees else 1.0
    result = {}
    for motor_name in LEROBOT_JOINTS:
        key = f'{motor_name}.pos'
        if key in observation:
            joint_name = LEROBOT_TO_ROS[motor_name]
            motor_position = float(observation[key]) * scale
            ros_position = _motor_to_ros(joint_name, motor_position)
            if joint_name == 'right_clamp':
                ros_position = _gripper_angle_to_position(ros_position)
            result[joint_name] = ros_position
    return result


def ros_to_action(values: Mapping[str, float], *, use_degrees: bool):
    """Convert ROS joint positions to a LeRobot action mapping.

    Revolute ROS joints use radians; ``right_clamp`` is the prismatic URDF
    joint and therefore uses metres before being converted to the motor angle.
    """
    scale = 180.0 / 3.141592653589793 if use_degrees else 1.0
    result = {}
    for name, value in values.items():
        if name not in ROS_TO_LEROBOT:
            continue
        ros_position = float(value)
        if name == 'right_clamp':
            ros_position = _gripper_position_to_angle(ros_position)
        result[f'{ROS_TO_LEROBOT[name]}.pos'] = (
            _ros_to_motor(name, ros_position) * scale)
    return result
