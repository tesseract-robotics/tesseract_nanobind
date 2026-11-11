"""The freespace problems the scaling benchmarks time are solvable: both states are collision-free (#341).

The benchmarks themselves run under `pixi run test-benchmark`; these checks run in the normal
suite so a bad obstacle layout fails there, not as a planner timeout inside a benchmark.
"""

import numpy as np
import pytest

from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.common import GeneralResourceLocator
from tesseract_robotics.planning import Pose, Robot, create_obstacle, sphere

from .benchmarks.freespace_problems import (
    JOINT_END,
    JOINT_START,
    OBSTACLE_RADIUS_M,
    FixtureStateInCollisionError,
    assert_state_collision_free,
    freespace_problem,
)

# An obstacle position that overlaps link_5 at JOINT_END: the layout #341 removed.
OVERLAPPING_OBSTACLE_XYZ = (0.6, 0.1, 0.65)


def test_collision_check_raises_for_a_colliding_state():
    """The invariant fires, naming the links, on the state the old `sphere_3` overlapped."""
    robot = Robot.from_tesseract_support(
        "lbr_iiwa_14_r820", GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    )
    create_obstacle(
        robot,
        name="probe",
        geometry=sphere(OBSTACLE_RADIUS_M),
        transform=Pose.from_xyz(*OVERLAPPING_OBSTACLE_XYZ),
    )
    joint_names = robot.get_joint_names("manipulator")
    with pytest.raises(FixtureStateInCollisionError, match="probe"):
        assert_state_collision_free(robot, joint_names, JOINT_END)


@pytest.mark.parametrize("num_obstacles", [1, 3, 5])
def test_problem_states_are_collision_free(num_obstacles):
    """`freespace_problem` builds only when both states clear every obstacle."""
    robot, program = freespace_problem(num_obstacles)
    joint_names = robot.get_joint_names("manipulator")
    for state in (JOINT_START, JOINT_END):
        assert_state_collision_free(robot, joint_names, np.asarray(state))
    assert len(program) == 2
