"""Tests for TimeOptimalTrajectoryGeneration time parameterization.

Tests the TOTG time parameterization using the 0.33 profile-based API.
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
    TimeOptimalTrajectoryGeneration,
    TOTGCompositeProfile,
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


def test_totg_constructor():
    """Test TimeOptimalTrajectoryGeneration constructor."""
    totg = TimeOptimalTrajectoryGeneration()
    assert totg is not None
    assert totg.getName() == "TOTG"

    totg_custom = TimeOptimalTrajectoryGeneration("CustomTOTG")
    assert totg_custom.getName() == "CustomTOTG"


def test_totg_profile_default():
    """Test TOTGCompositeProfile default values."""
    profile = TOTGCompositeProfile()
    assert profile.max_velocity_scaling_factor == 1.0
    assert profile.max_acceleration_scaling_factor == 1.0
    assert profile.path_tolerance == 0.1
    assert profile.min_angle_change == 0.001
    assert profile.override_limits is False


def test_totg_profile_custom():
    """Test TOTGCompositeProfile with custom values."""
    profile = TOTGCompositeProfile(0.5, 0.5, 0.05, 0.002)
    assert profile.max_velocity_scaling_factor == 0.5
    assert profile.max_acceleration_scaling_factor == 0.5
    assert profile.path_tolerance == 0.05
    assert profile.min_angle_change == 0.002


def test_totg_profile_limits():
    """Test TOTGCompositeProfile with override limits."""
    profile = TOTGCompositeProfile()
    profile.override_limits = True

    max_vel = 2.088
    max_acc = 1.0
    profile.velocity_limits = np.column_stack((-np.ones(6) * max_vel, np.ones(6) * max_vel))
    profile.acceleration_limits = np.column_stack((-np.ones(6) * max_acc, np.ones(6) * max_acc))

    assert profile.override_limits is True
    assert profile.velocity_limits.shape == (6, 2)
    assert profile.acceleration_limits.shape == (6, 2)
    assert np.allclose(profile.velocity_limits[:, 1], max_vel)
    assert np.allclose(profile.acceleration_limits[:, 1], max_acc)


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
