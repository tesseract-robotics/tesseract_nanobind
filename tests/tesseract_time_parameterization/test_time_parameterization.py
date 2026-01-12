"""Tests for tesseract_time_parameterization bindings.

Tests cover:
- TimeOptimalTrajectoryGeneration (TOTG)
- IterativeSplineParameterization (ISP)
- InstructionsTrajectory container
- Profile-based time parameterization (0.33 API)
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
    ISPMoveProfile,
    IterativeSplineParameterization,
    TimeOptimalTrajectoryGeneration,
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
