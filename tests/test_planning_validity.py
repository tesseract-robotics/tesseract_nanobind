"""The shared validity check works on joint-only groups, which have no kinematic solver."""

import numpy as np
import pytest

from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.common import GeneralResourceLocator
from tesseract_robotics.planning import Robot
from tesseract_robotics.planning.composer import PlanningResult, TrajectoryPoint

from .planning_validity import JointGoal, TrajectoryInvalidError, assert_trajectory_valid

# `manipulator_aux` of the puzzle-piece workcell: KUKA IIWA (7) + positioner (2), a joint group
# without an inverse-kinematics solver.
GROUP = "manipulator_aux"
# The example's initial configuration, inside every limit.
HOME_RAD = np.array([-0.785398, 0.4, 0.0, -1.9, 0.0, 1.0, 0.0, 0.0, 0.0])
# Far outside any joint's limit.
OUT_OF_LIMITS_RAD = 100.0


@pytest.fixture(scope="module")
def robot() -> Robot:
    return Robot.from_urdf(
        "package://tesseract/support/urdf/puzzle_piece_workcell.urdf",
        "package://tesseract/support/urdf/puzzle_piece_workcell.srdf",
        GeneralResourceLocator(paths=[get_tesseract_resource_path()]),
    )


def _result(robot: Robot, positions: np.ndarray) -> PlanningResult:
    names = robot.get_joint_names(GROUP)
    return PlanningResult(successful=True, trajectory=[TrajectoryPoint(names, positions)])


def test_joint_only_group_within_limits_is_valid(robot):
    assert_trajectory_valid(
        robot,
        _result(robot, HOME_RAD),
        JointGoal.build(HOME_RAD),
        group=GROUP,
        check_collision=False,
    )


def test_joint_only_group_outside_limits_is_rejected(robot):
    positions = HOME_RAD.copy()
    positions[0] = OUT_OF_LIMITS_RAD
    with pytest.raises(TrajectoryInvalidError, match="outside"):
        assert_trajectory_valid(
            robot, _result(robot, positions), group=GROUP, check_collision=False
        )
