"""Tests for tesseract_time_parameterization bindings.

Tests cover:
- TimeOptimalTrajectoryGeneration (TOTG)
- IterativeSplineParameterization (ISP)
- InstructionsTrajectory container
- Profile-based time parameterization (0.33 API)
"""

import gc
import sys

import numpy as np
import pytest

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
    ConstantTCPSpeedCompositeProfile,
    ConstantTCPSpeedParameterization,
    InstructionsTrajectory,
    ISPCompositeProfile,
    ISPMoveProfile,
    IterativeSplineParameterization,
    TimeOptimalTrajectoryGeneration,
    TimeParameterization,
    TOTGCompositeProfile,
)


class TestTimeOptimalTrajectoryGeneration:
    """Test TOTG time parameterization."""

    def test_default_constructor(self):
        # 0.33 API: constructor takes name
        totg = TimeOptimalTrajectoryGeneration()
        assert totg is not None
        assert totg.getName() == "TOTG"

    def test_custom_name(self):
        # 0.33 API: constructor takes name parameter
        totg = TimeOptimalTrajectoryGeneration("MyTOTG")
        assert totg is not None
        assert totg.getName() == "MyTOTG"


class TestIterativeSplineParameterization:
    """Test ISP time parameterization."""

    def test_default_constructor(self):
        # 0.33 API: constructor takes name
        isp = IterativeSplineParameterization()
        assert isp is not None
        assert isp.getName() == "ISP"

    def test_custom_name(self):
        # 0.33 API: constructor takes name parameter
        isp = IterativeSplineParameterization("MyISP")
        assert isp is not None
        assert isp.getName() == "MyISP"


class TestTOTGCompositeProfile:
    """Test TOTGCompositeProfile."""

    def test_default_constructor(self):
        profile = TOTGCompositeProfile()
        assert profile is not None
        assert profile.max_velocity_scaling_factor == 1.0
        assert profile.max_acceleration_scaling_factor == 1.0
        assert profile.path_tolerance == 0.1
        assert profile.min_angle_change == 0.001

    def test_custom_constructor(self):
        profile = TOTGCompositeProfile(0.5, 0.5, 0.05, 0.002)
        assert profile.max_velocity_scaling_factor == 0.5
        assert profile.max_acceleration_scaling_factor == 0.5
        assert profile.path_tolerance == 0.05
        assert profile.min_angle_change == 0.002

    def test_override_limits(self):
        profile = TOTGCompositeProfile()
        profile.override_limits = True
        profile.velocity_limits = np.column_stack((-np.ones(6), np.ones(6)))
        profile.acceleration_limits = np.column_stack((-np.ones(6) * 2, np.ones(6) * 2))
        assert profile.override_limits is True
        assert profile.velocity_limits.shape == (6, 2)
        assert profile.acceleration_limits.shape == (6, 2)


class TestISPCompositeProfile:
    """Test ISPCompositeProfile."""

    def test_default_constructor(self):
        profile = ISPCompositeProfile()
        assert profile is not None
        assert profile.add_points is True
        assert profile.max_velocity_scaling_factor == 1.0
        assert profile.max_acceleration_scaling_factor == 1.0

    def test_custom_constructor(self):
        profile = ISPCompositeProfile(0.5, 0.5)
        assert profile.max_velocity_scaling_factor == 0.5
        assert profile.max_acceleration_scaling_factor == 0.5

    def test_add_points_setting(self):
        profile = ISPCompositeProfile()
        profile.add_points = False
        assert profile.add_points is False


class TestISPMoveProfile:
    """Test ISPMoveProfile."""

    def test_default_constructor(self):
        profile = ISPMoveProfile()
        assert profile is not None
        assert profile.max_velocity_scaling_factor == 1.0
        assert profile.max_acceleration_scaling_factor == 1.0


class TestInstructionsTrajectory:
    """Test InstructionsTrajectory container."""

    def test_create_from_composite(self):
        """Test creating InstructionsTrajectory from CompositeInstruction."""
        program = CompositeInstruction("DEFAULT")
        manip = ManipulatorInfo()
        manip.manipulator = "manipulator"
        program.setManipulatorInfo(manip)

        # Add joint waypoints
        joint_names = ["j1", "j2", "j3", "j4", "j5", "j6"]
        positions = [
            np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]),
            np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2]),
        ]

        for pos in positions:
            wp = StateWaypoint(joint_names, pos)
            mi = MoveInstruction(
                StateWaypointPoly_wrap_StateWaypoint(wp),
                MoveInstructionType_FREESPACE,
                "DEFAULT",
            )
            program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(mi))

        assert len(program) == 3

        # Create trajectory container
        trajectory = InstructionsTrajectory(program)
        assert trajectory.size() == 3
        assert trajectory.dof() == 6
        assert not trajectory.empty()


class TestConstantTCPSpeedParameterization:
    def test_default_constructor(self):
        tp = ConstantTCPSpeedParameterization()
        assert tp is not None
        assert tp.getName() == "ConstantTCPSpeed"

    def test_custom_name(self):
        tp = ConstantTCPSpeedParameterization("MyTCP")
        assert tp.getName() == "MyTCP"

    def test_is_time_parameterization(self):
        tp = ConstantTCPSpeedParameterization()
        assert isinstance(tp, TimeParameterization)


class TestConstantTCPSpeedCompositeProfile:
    def test_default_construction(self):
        p = ConstantTCPSpeedCompositeProfile()
        assert p is not None
        # Defaults (from C++ header)
        assert p.max_translational_velocity == pytest.approx(1.0)
        assert p.max_rotational_velocity == pytest.approx(1.0)
        assert p.max_translational_acceleration == pytest.approx(1.0)
        assert p.max_rotational_acceleration == pytest.approx(1.0)
        assert p.max_velocity_scaling_factor == pytest.approx(1.0)
        assert p.max_acceleration_scaling_factor == pytest.approx(1.0)

    def test_full_constructor(self):
        p = ConstantTCPSpeedCompositeProfile(
            max_translational_velocity=0.5,
            max_rotational_velocity=0.8,
            max_translational_acceleration=1.5,
            max_rotational_acceleration=2.0,
            max_velocity_scaling_factor=0.75,
            max_acceleration_scaling_factor=0.6,
        )
        assert p.max_translational_velocity == pytest.approx(0.5)
        assert p.max_rotational_velocity == pytest.approx(0.8)
        assert p.max_translational_acceleration == pytest.approx(1.5)
        assert p.max_rotational_acceleration == pytest.approx(2.0)
        assert p.max_velocity_scaling_factor == pytest.approx(0.75)
        assert p.max_acceleration_scaling_factor == pytest.approx(0.6)

    def test_constructor_scaling_defaults(self):
        # scaling factors default to 1.0 when omitted
        p = ConstantTCPSpeedCompositeProfile(0.2, 0.3, 0.4, 0.5)
        assert p.max_translational_velocity == pytest.approx(0.2)
        assert p.max_velocity_scaling_factor == pytest.approx(1.0)
        assert p.max_acceleration_scaling_factor == pytest.approx(1.0)

    def test_member_assignment(self):
        p = ConstantTCPSpeedCompositeProfile()
        p.max_translational_velocity = 0.123
        p.max_velocity_scaling_factor = 0.5
        assert p.max_translational_velocity == pytest.approx(0.123)
        assert p.max_velocity_scaling_factor == pytest.approx(0.5)


# ---------------------------------------------------------------------------
# Helpers: programs on the ABB IRB2400 joints
# ---------------------------------------------------------------------------

ABB_JOINTS = [f"joint_{i}" for i in range(1, 7)]


def _move(position):
    swp = StateWaypoint(ABB_JOINTS, np.asarray(position, dtype=np.float64))
    return MoveInstructionPoly_wrap_MoveInstruction(
        MoveInstruction(
            StateWaypointPoly_wrap_StateWaypoint(swp), MoveInstructionType_FREESPACE, "DEFAULT"
        )
    )


def _manipulator_info():
    manip = ManipulatorInfo()
    manip.manipulator = "manipulator"
    manip.working_frame = "base_link"
    manip.tcp_frame = "tool0"
    return manip


def _abb_program(num=10, span=1.0):
    """A flat FREESPACE program on the IRB2400 joints, joint_1 swept over `span` rad."""
    program = CompositeInstruction("DEFAULT")
    program.setManipulatorInfo(_manipulator_info())
    for i in range(num + 1):
        p = np.zeros(6)
        p[0] = i * span / num
        p[1] = 0.5 * i * span / num
        program.appendMoveInstruction(_move(p))
    return program


# ---------------------------------------------------------------------------
# #277: the stub names Environment, not the C++ type
# ---------------------------------------------------------------------------


def test_compute_signature_names_python_environment():
    assert "tesseract::" not in TimeParameterization.compute.__doc__


# ---------------------------------------------------------------------------
# #281: InstructionsTrajectory keeps its program alive
# ---------------------------------------------------------------------------


def test_instructions_trajectory_keeps_program_alive():
    first = np.array([0.25, -0.5, 0.75, -1.0, 1.25, -1.5])

    def make_program():
        program = CompositeInstruction("DEFAULT")
        program.setManipulatorInfo(_manipulator_info())
        program.appendMoveInstruction(_move(first))
        program.appendMoveInstruction(_move(np.zeros(6)))
        return program

    traj = InstructionsTrajectory(make_program())
    gc.collect()
    # Reuse the freed memory, so a dangling reference reads someone else's data.
    junk = [_abb_program(num=20, span=9.0) for _ in range(20)]
    del junk
    gc.collect()
    np.testing.assert_array_equal(traj.getPosition(0), first)


# ---------------------------------------------------------------------------
# #280: ISPCompositeProfile.minimum_time_delta
# ---------------------------------------------------------------------------


def test_isp_composite_profile_minimum_time_delta():
    profile = ISPCompositeProfile()
    assert profile.minimum_time_delta == sys.float_info.epsilon
    profile.minimum_time_delta = 1e-3
    assert profile.minimum_time_delta == 1e-3
