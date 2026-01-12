"""Tests for IterativeSplineParameterization time parameterization.

Tests the ISP time parameterization using the 0.33 profile-based API.
"""

import numpy as np

from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    MoveInstruction,
    MoveInstructionPoly_wrap_MoveInstruction,
    MoveInstructionType_FREESPACE,
    StateWaypoint,
    StateWaypointPoly_wrap_StateWaypoint,
)
from tesseract_robotics.tesseract_common import ManipulatorInfo
from tesseract_robotics.tesseract_time_parameterization import (
    InstructionsTrajectory,
    ISPCompositeProfile,
    IterativeSplineParameterization,
)


def create_straight_trajectory():
    """Create a simple straight-line trajectory for testing."""
    num = 10
    max_ = 2.0
    joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

    program = CompositeInstruction("DEFAULT")
    manip = ManipulatorInfo()
    manip.manipulator = "manipulator"
    program.setManipulatorInfo(manip)

    for i in range(num):
        p = np.zeros((6,), dtype=np.float64)
        p[0] = i * (max_ / num)
        swp = StateWaypoint(joint_names, p)
        program.appendMoveInstruction(
            MoveInstructionPoly_wrap_MoveInstruction(
                MoveInstruction(
                    StateWaypointPoly_wrap_StateWaypoint(swp),
                    MoveInstructionType_FREESPACE,
                )
            )
        )

    p = np.zeros((6,), dtype=np.float64)
    p[0] = max_
    swp = StateWaypoint(joint_names, p)
    program.appendMoveInstruction(
        MoveInstructionPoly_wrap_MoveInstruction(
            MoveInstruction(
                StateWaypointPoly_wrap_StateWaypoint(swp),
                MoveInstructionType_FREESPACE,
            )
        )
    )

    return program


def test_isp_constructor():
    """Test IterativeSplineParameterization constructor."""
    isp = IterativeSplineParameterization()
    assert isp is not None
    assert isp.getName() == "ISP"


def test_isp_profile():
    """Test ISPCompositeProfile settings."""
    profile = ISPCompositeProfile()
    assert profile.add_points is True

    profile.add_points = False
    assert profile.add_points is False

    # Test velocity/acceleration limits
    profile.override_limits = True
    profile.velocity_limits = np.column_stack((-np.ones(6) * 2, np.ones(6) * 2))
    profile.acceleration_limits = np.column_stack((-np.ones(6), np.ones(6)))
    assert profile.velocity_limits.shape == (6, 2)
    assert profile.acceleration_limits.shape == (6, 2)


def test_instructions_trajectory():
    """Test InstructionsTrajectory from CompositeInstruction."""
    program = create_straight_trajectory()
    traj = InstructionsTrajectory(program)

    assert traj.size() == 11  # 10 + 1 final point
    assert traj.dof() == 6
    assert not traj.empty()

    # Check positions are correct
    pos0 = traj.getPosition(0)
    assert np.allclose(pos0, np.zeros(6))

    pos_last = traj.getPosition(traj.size() - 1)
    expected = np.array([2.0, 0, 0, 0, 0, 0])
    assert np.allclose(pos_last, expected)
