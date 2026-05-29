"""Tests for tesseract_motion_planners_descartes bindings."""

import numpy as np
import pytest

from tesseract_robotics.tesseract_command_language import (
    CartesianWaypoint,
    CartesianWaypointPoly_wrap_CartesianWaypoint,
    CompositeInstruction,
    InstructionPoly_as_MoveInstructionPoly,
    MoveInstruction,
    MoveInstructionPoly_wrap_MoveInstruction,
    MoveInstructionType_LINEAR,
    ProfileDictionary,
    WaypointPoly_as_StateWaypointPoly,
)
from tesseract_robotics.tesseract_common import (
    FilesystemPath,
    GeneralResourceLocator,
    Isometry3d,
    ManipulatorInfo,
    Quaterniond,
    Translation3d,
)
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_motion_planners import PlannerRequest
from tesseract_robotics.tesseract_motion_planners_simple import generateInterpolatedProgram

# Descartes imports - skip tests if not available
try:
    from tesseract_robotics.tesseract_motion_planners_descartes import (
        DescartesDefaultMoveProfileD,
        DescartesDefaultPlanProfileD,
        DescartesEdgeEvaluatorD,
        DescartesMotionPlannerD,
        DescartesMoveProfileD,
        cast_DescartesMoveProfileD,
        cast_DescartesPlanProfileD,
    )

    DESCARTES_AVAILABLE = True
except ImportError:
    DESCARTES_AVAILABLE = False


DESCARTES_DEFAULT_NAMESPACE = "DescartesMotionPlannerTask"


@pytest.fixture
def abb_irb2400_environment():
    """Load ABB IRB2400 robot environment for testing (has OPW kinematics)."""
    locator = GeneralResourceLocator()
    urdf_path = FilesystemPath(
        locator.locateResource("package://tesseract/support/urdf/abb_irb2400.urdf").getFilePath()
    )
    srdf_path = FilesystemPath(
        locator.locateResource("package://tesseract/support/urdf/abb_irb2400.srdf").getFilePath()
    )

    t_env = Environment()
    assert t_env.init(urdf_path, srdf_path, locator), "Failed to initialize ABB IRB2400"

    manip_info = ManipulatorInfo()
    manip_info.tcp_frame = "tool0"
    manip_info.manipulator = "manipulator"
    manip_info.manipulator_ik_solver = "OPWInvKin"
    manip_info.working_frame = "base_link"

    joint_names = list(t_env.getJointGroup("manipulator").getJointNames())
    return t_env, manip_info, joint_names


class TestDescartesProfiles:
    """Test Descartes profile types."""

    def test_default_plan_profile(self):
        profile = DescartesDefaultPlanProfileD()
        assert profile is not None
        # Test base class methods
        assert profile.getKey() is not None

    def test_default_plan_profile_attributes(self):
        """Test DescartesDefaultPlanProfileD attributes."""
        profile = DescartesDefaultPlanProfileD()

        # Test pose sampling attributes
        profile.target_pose_fixed = False
        assert profile.target_pose_fixed is False

        profile.target_pose_sample_resolution = 0.5
        assert profile.target_pose_sample_resolution == 0.5

        profile.target_pose_sample_min = -np.pi
        assert profile.target_pose_sample_min == pytest.approx(-np.pi)

        profile.target_pose_sample_max = np.pi
        assert profile.target_pose_sample_max == pytest.approx(np.pi)

        # Test collision attributes
        profile.allow_collision = True
        assert profile.allow_collision is True

        profile.enable_collision = False
        assert profile.enable_collision is False

        profile.enable_edge_collision = True
        assert profile.enable_edge_collision is True

        profile.use_redundant_joint_solutions = True
        assert profile.use_redundant_joint_solutions is True

        profile.debug = True
        assert profile.debug is True


class TestDescartesMotionPlanner:
    """Test Descartes motion planner."""

    def test_constructor(self):
        planner = DescartesMotionPlannerD(DESCARTES_DEFAULT_NAMESPACE)
        assert planner is not None
        assert planner.getName() == DESCARTES_DEFAULT_NAMESPACE

    def test_terminate_and_clear(self):
        """Test terminate() and clear() methods."""
        planner = DescartesMotionPlannerD(DESCARTES_DEFAULT_NAMESPACE)
        # These should not raise exceptions
        planner.terminate()
        planner.clear()


class TestDescartesPlanning:
    """Integration test for Descartes motion planning."""

    def test_descartes_freespace_fixed_poses(self, abb_irb2400_environment):
        """Test Descartes planning between two fixed Cartesian poses."""
        env, manip_info, joint_names = abb_irb2400_environment

        # Two cartesian waypoints
        wp1 = CartesianWaypoint(
            Isometry3d.Identity()
            * Translation3d(0.8, -0.2, 0.8)
            * Quaterniond.from_xyzw(0, -1.0, 0, 0)
        )
        wp2 = CartesianWaypoint(
            Isometry3d.Identity()
            * Translation3d(0.8, 0.2, 0.8)
            * Quaterniond.from_xyzw(0, -1.0, 0, 0)
        )

        start_instruction = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp1),
            MoveInstructionType_LINEAR,
            "TEST_PROFILE",
        )
        start_instruction.setManipulatorInfo(manip_info)

        plan_f1 = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp2),
            MoveInstructionType_LINEAR,
            "TEST_PROFILE",
        )
        plan_f1.setManipulatorInfo(manip_info)

        program = CompositeInstruction()
        program.setManipulatorInfo(manip_info)
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_f1))

        # Interpolate the program (Descartes requires seed trajectory)
        interpolated_program = generateInterpolatedProgram(program, env, 3.14, 1.0, 3.14, 10)

        # Setup Descartes profiles
        plan_profile = DescartesDefaultPlanProfileD()
        # Use cast helper to convert to Profile for ProfileDictionary
        plan_profile_base = cast_DescartesPlanProfileD(plan_profile)

        profiles = ProfileDictionary()
        profiles.addProfile(DESCARTES_DEFAULT_NAMESPACE, "TEST_PROFILE", plan_profile_base)

        # Create planner and request
        single_descartes_planner = DescartesMotionPlannerD(DESCARTES_DEFAULT_NAMESPACE)

        request = PlannerRequest()
        request.instructions = interpolated_program
        request.env = env
        request.profiles = profiles

        # Solve
        response = single_descartes_planner.solve(request)
        assert response.successful, f"Descartes planning failed: {response.message}"

        # Verify results - iterate through instructions
        results = response.results
        count = 0
        for instr in results:
            if instr.isMoveInstruction():
                move_instr = InstructionPoly_as_MoveInstructionPoly(instr)
                wp = move_instr.getWaypoint()
                if wp.isStateWaypoint():
                    state_wp = WaypointPoly_as_StateWaypointPoly(wp)
                    assert len(state_wp.getNames()) == 6  # ABB IRB2400 has 6 joints
                    assert isinstance(state_wp.getPosition(), np.ndarray)
                    assert len(state_wp.getPosition()) == 6
                count += 1
        assert count > 0, "Expected at least one move instruction in results"


class TestPythonEdgeEvaluator:
    """End-to-end test of a Python-side custom EdgeEvaluator plugged into a
    Python-side DescartesMoveProfileD subclass."""

    def test_custom_python_edge_evaluator_is_called(self, abb_irb2400_environment):
        env, manip_info, _ = abb_irb2400_environment

        # Counter visible across all instances; cheap way to prove the C++
        # planner called back into our Python evaluate() during the solve.
        call_counter = {"n": 0}

        class JointDistanceEdgeEvaluator(DescartesEdgeEvaluatorD):
            """Cost = L2 norm of joint-space delta; always valid."""

            def evaluate(self, start, end):
                call_counter["n"] += 1
                delta = end.values - start.values
                cost = float(np.linalg.norm(delta))
                return (True, cost)

        class PythonMoveProfile(DescartesMoveProfileD):
            """Delegates sampling/state-evaluation to the default profile,
            but supplies a Python EdgeEvaluator."""

            def __init__(self):
                super().__init__()
                self._inner = DescartesDefaultMoveProfileD()
                # Keep evaluators alive on the profile so the planner sees
                # the same Python instance for every edge it evaluates.
                self._edge_evaluator = JointDistanceEdgeEvaluator()

            def createWaypointSampler(self, move_instruction, composite_manip_info, env):
                return self._inner.createWaypointSampler(move_instruction, composite_manip_info, env)

            def createEdgeEvaluator(self, move_instruction, composite_manip_info, env):
                return self._edge_evaluator

            def createStateEvaluator(self, move_instruction, composite_manip_info, env):
                return self._inner.createStateEvaluator(move_instruction, composite_manip_info, env)

        # Build the same fixed-pose program as the default-profile test.
        wp1 = CartesianWaypoint(
            Isometry3d.Identity()
            * Translation3d(0.8, -0.2, 0.8)
            * Quaterniond.from_xyzw(0, -1.0, 0, 0)
        )
        wp2 = CartesianWaypoint(
            Isometry3d.Identity()
            * Translation3d(0.8, 0.2, 0.8)
            * Quaterniond.from_xyzw(0, -1.0, 0, 0)
        )

        start_instruction = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp1),
            MoveInstructionType_LINEAR,
            "PY_EDGE_PROFILE",
        )
        start_instruction.setManipulatorInfo(manip_info)

        plan_f1 = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp2),
            MoveInstructionType_LINEAR,
            "PY_EDGE_PROFILE",
        )
        plan_f1.setManipulatorInfo(manip_info)

        program = CompositeInstruction()
        program.setManipulatorInfo(manip_info)
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_f1))

        interpolated_program = generateInterpolatedProgram(program, env, 3.14, 1.0, 3.14, 10)

        py_profile = PythonMoveProfile()
        profiles = ProfileDictionary()
        profiles.addProfile(
            DESCARTES_DEFAULT_NAMESPACE,
            "PY_EDGE_PROFILE",
            cast_DescartesMoveProfileD(py_profile),
        )

        planner = DescartesMotionPlannerD(DESCARTES_DEFAULT_NAMESPACE)
        request = PlannerRequest()
        request.instructions = interpolated_program
        request.env = env
        request.profiles = profiles

        response = planner.solve(request)
        assert response.successful, f"Descartes planning failed: {response.message}"
        assert call_counter["n"] > 0, "Python EdgeEvaluator.evaluate() was never called by the planner"
