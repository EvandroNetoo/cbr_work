import pytest

from manipulation.errors import ConfigurationError
from manipulation.node import ManipulationServer
from interfaces.action import PlaceAtPose, PlaceInContainer, PlaceOnTable


def _valid_pose():
    goal = PlaceAtPose.Goal()
    goal.release_pose.header.frame_id = 'arm_base_link'
    goal.release_pose.pose.position.x = 0.1
    goal.release_pose.pose.position.y = -0.2
    goal.release_pose.pose.position.z = 0.05
    goal.release_pose.pose.orientation.w = 1.0
    return goal.release_pose


def test_explicit_release_pose_must_be_in_arm_base_link():
    pose = _valid_pose()
    pose.header.frame_id = 'map'

    with pytest.raises(ConfigurationError, match='arm_base_link'):
        ManipulationServer._validate_target_pose(pose)


def test_explicit_release_pose_rejects_null_quaternion():
    pose = _valid_pose()
    pose.pose.orientation.w = 0.0

    with pytest.raises(ConfigurationError, match='quaternion nulo'):
        ManipulationServer._validate_target_pose(pose)


def test_table_and_container_accept_arbitrary_ws_height_in_centimeters():
    table = PlaceOnTable.Goal()
    table.ws_height_cm = 12.75
    container = PlaceInContainer.Goal()
    container.ws_height_cm = -3.5

    assert table.ws_height_cm == pytest.approx(12.75)
    assert container.ws_height_cm == pytest.approx(-3.5)


def test_container_colors_are_an_enum():
    assert PlaceInContainer.Goal.RED == 1
    assert PlaceInContainer.Goal.BLUE == 2
