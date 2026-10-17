"""Tests for tesseract_motion_planners bindings."""

from pathlib import Path

import numpy as np
import pytest

from tesseract_robotics.tesseract_collision import (
    CollisionCheckConfig,
    CollisionEvaluatorType,
    ContactResultMap,
    ContactTrajectoryResults,
)
from tesseract_robotics.tesseract_command_language import (
    CartesianWaypoint,
    CartesianWaypointPoly_wrap_CartesianWaypoint,
    CompositeInstruction,
    InstructionPoly,
    JointWaypoint,
    JointWaypointPoly_wrap_JointWaypoint,
    MoveInstruction,
    MoveInstructionPoly_wrap_MoveInstruction,
    MoveInstructionType_FREESPACE,
    ProfileDictionary,
    StateWaypoint,
    StateWaypointPoly_wrap_StateWaypoint,
    WaypointPoly_as_CartesianWaypointPoly,
    WaypointPoly_as_JointWaypointPoly,
    WaypointPoly_as_StateWaypointPoly,
)
from tesseract_robotics.tesseract_common import (
    GeneralResourceLocator,
    Isometry3d,
    ManipulatorInfo,
    Quaterniond,
    Translation3d,
)
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_kinematics import KinGroupIKInput
from tesseract_robotics.tesseract_motion_planners import (
    MotionPlanner,
    PlannerRequest,
    RobotConfig,
    contactCheckProgram,
    formatProgram,
    getJointTurns,
    getRobotConfig,
    isWithinJointLimits,
    toToolpath,
)
from tesseract_robotics.tesseract_motion_planners_ompl import (
    OMPLMotionPlanner,
    OMPLRealVectorPlanProfile,
    ProfileDictionary_addOMPLProfile,
)
from tesseract_robotics.tesseract_motion_planners_simple import (
    SimpleMotionPlanner,
    generateInterpolatedProgram,
)

OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask"


@pytest.fixture
def abb_irb2400_environment():
    """Load ABB IRB2400 robot environment for testing."""
    locator = GeneralResourceLocator()
    urdf_path = Path(
        locator.locateResource("package://tesseract/support/urdf/abb_irb2400.urdf").getFilePath()
    )
    srdf_path = Path(
        locator.locateResource("package://tesseract/support/urdf/abb_irb2400.srdf").getFilePath()
    )
    t_env = Environment()
    assert t_env.init(urdf_path, srdf_path, locator), "Failed to initialize ABB IRB2400"
    return t_env


class TestPlannerRequest:
    """Test PlannerRequest creation."""

    def test_default_constructor(self):
        request = PlannerRequest()
        assert request is not None


class TestOMPLMotionPlanner:
    """Test OMPL motion planner."""

    def test_constructor(self):
        planner = OMPLMotionPlanner(OMPL_DEFAULT_NAMESPACE)
        assert planner is not None

    def test_plan_profile(self):
        profile = OMPLRealVectorPlanProfile()
        assert profile is not None


class TestOMPLPlanning:
    """Integration test for OMPL motion planning with ABB robot."""

    def test_ompl_planning_workflow(self, abb_irb2400_environment):
        """Test complete OMPL planning workflow as in abb_irb2400_viewer.py."""
        t_env = abb_irb2400_environment

        # Setup manipulator info
        manip_info = ManipulatorInfo()
        manip_info.tcp_frame = "tool0"
        manip_info.manipulator = "manipulator"
        manip_info.working_frame = "base_link"

        # Set initial state
        joint_names = [f"joint_{i + 1}" for i in range(6)]
        t_env.setState(joint_names, np.ones(6) * 0.1)

        # Define waypoints
        wp1 = CartesianWaypoint(
            Isometry3d.Identity()
            * Translation3d(0.8, -0.3, 1.455)
            * Quaterniond.from_xyzw(0, 0.70710678, 0, 0.70710678)
        )
        wp2 = CartesianWaypoint(
            Isometry3d.Identity()
            * Translation3d(0.8, 0.3, 1.455)
            * Quaterniond.from_xyzw(0, 0.70710678, 0, 0.70710678)
        )

        # Create instructions
        start_instruction = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp1),
            MoveInstructionType_FREESPACE,
            "DEFAULT",
        )
        plan_f1 = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp2),
            MoveInstructionType_FREESPACE,
            "DEFAULT",
        )

        # Create program
        program = CompositeInstruction("DEFAULT")
        program.setManipulatorInfo(manip_info)
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_f1))

        # Setup OMPL planner - use helper function for cross-module profile registration
        plan_profile = OMPLRealVectorPlanProfile()
        profiles = ProfileDictionary()
        ProfileDictionary_addOMPLProfile(profiles, OMPL_DEFAULT_NAMESPACE, "DEFAULT", plan_profile)

        # Create request
        request = PlannerRequest()
        request.instructions = program
        request.env = t_env
        request.profiles = profiles

        # Solve
        ompl_planner = OMPLMotionPlanner(OMPL_DEFAULT_NAMESPACE)
        response = ompl_planner.solve(request)

        assert response.successful, f"OMPL planning failed: {response.message}"
        assert response.results is not None

        # Interpolate results
        interpolated_results = generateInterpolatedProgram(
            response.results, t_env, 3.14, 1.0, 3.14, 10
        )
        assert interpolated_results is not None


# NOTE(#103): a TestOMPLSeedDeterminism contract test (same seed -> bit-identical
# path via single-planner + optimize=False) lived here briefly. It cannot pass on
# Windows: conda-forge win-64 ompl ships only a static Library/lib/ompl.lib, so
# the planner DLL and the Python extension each embed their own copy of OMPL's
# RNGSeedGenerator - RNG_setSeed seeds the extension's copy, RRTConnect draws
# from the DLL's. Restore the test once OMPL is shared on Windows or upstream
# OMPLSolverConfig grows a seed field applied inside parallelPlan.


class TestSimplePlanner:
    """Test simple motion planner utilities."""

    def test_generate_interpolated_program(self, abb_irb2400_environment):
        """Test generateInterpolatedProgram function."""
        t_env = abb_irb2400_environment

        # Setup manipulator info
        manip_info = ManipulatorInfo()
        manip_info.tcp_frame = "tool0"
        manip_info.manipulator = "manipulator"
        manip_info.working_frame = "base_link"

        # Set initial state
        joint_names = [f"joint_{i + 1}" for i in range(6)]
        t_env.setState(joint_names, np.ones(6) * 0.1)

        # Create simple program with one move
        wp1 = CartesianWaypoint(
            Isometry3d.Identity()
            * Translation3d(0.8, 0.0, 1.455)
            * Quaterniond.from_xyzw(0, 0.70710678, 0, 0.70710678)
        )

        mi = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp1),
            MoveInstructionType_FREESPACE,
            "DEFAULT",
        )

        program = CompositeInstruction("DEFAULT")
        program.setManipulatorInfo(manip_info)
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(mi))

        # First solve with OMPL to get a valid result - use helper function
        plan_profile = OMPLRealVectorPlanProfile()
        profiles = ProfileDictionary()
        ProfileDictionary_addOMPLProfile(profiles, OMPL_DEFAULT_NAMESPACE, "DEFAULT", plan_profile)

        request = PlannerRequest()
        request.instructions = program
        request.env = t_env
        request.profiles = profiles

        ompl_planner = OMPLMotionPlanner(OMPL_DEFAULT_NAMESPACE)
        response = ompl_planner.solve(request)

        if response.successful:
            # Now test interpolation
            interpolated = generateInterpolatedProgram(response.results, t_env, 3.14, 1.0, 3.14, 10)
            assert interpolated is not None


# ---- shared helpers for the binding-gap tests below (#303, #332, #333) ----

IRB2400_JOINTS = [f"joint_{i + 1}" for i in range(6)]
IRB2400_START = np.zeros(6)
IRB2400_END = np.array([0.5, 0.2, -0.3, 0.4, 0.6, -0.2])
# Shoulder and elbow folded forward and down (both within limits): the arm hits the base.
IRB2400_COLLIDING = np.array([0.0, 1.309, 1.134, 0.0, 0.0, 0.0])


def _irb2400_manip_info():
    return ManipulatorInfo("manipulator", "base_link", "tool0")


def _joint_move(names, values):
    wp = JointWaypointPoly_wrap_JointWaypoint(JointWaypoint(names, values))
    return MoveInstructionPoly_wrap_MoveInstruction(
        MoveInstruction(wp, MoveInstructionType_FREESPACE, "DEFAULT")
    )


def _joint_program(*states, names=IRB2400_JOINTS):
    program = CompositeInstruction("DEFAULT")
    program.setManipulatorInfo(_irb2400_manip_info())
    for q in states:
        program.appendMoveInstruction(_joint_move(names, q))
    return program


# A non-root working frame: toToolpath must ignore it and return world-frame poses.
TOOLPATH_WORKING_FRAME = "link_2"


def _link2_manip_info():
    return ManipulatorInfo("manipulator", TOOLPATH_WORKING_FRAME, "tool0")


def _assert_world_frame(env, q, pose):
    transforms = env.getState(IRB2400_JOINTS, q).link_transforms
    np.testing.assert_allclose(pose.matrix, transforms["tool0"].matrix, atol=1e-9)
    in_working = transforms[TOOLPATH_WORKING_FRAME].inverse() * transforms["tool0"]
    assert not np.allclose(pose.matrix, in_working.matrix, atol=1e-6)


class TestToToolpath:
    """#303: toToolpath returns world-frame TCP poses per move, whatever the working frame."""

    def test_composite_overload(self, abb_irb2400_environment):
        env = abb_irb2400_environment
        program = _joint_program(IRB2400_START, IRB2400_END)
        program.setManipulatorInfo(_link2_manip_info())
        toolpath = toToolpath(program, env)
        assert len(toolpath) == 1
        poses = toolpath[0]
        assert len(poses) == 2
        _assert_world_frame(env, IRB2400_START, poses[0])
        _assert_world_frame(env, IRB2400_END, poses[1])

    def test_move_and_instruction_overloads(self, abb_irb2400_environment):
        env = abb_irb2400_environment
        mi = _joint_move(IRB2400_JOINTS, IRB2400_END)
        mi.setManipulatorInfo(_link2_manip_info())
        for arg in (mi, InstructionPoly(mi)):
            toolpath = toToolpath(arg, env)
            assert len(toolpath) == 1
            _assert_world_frame(env, IRB2400_END, toolpath[0][0])


class TestFormatProgram:
    """#303: formatProgram reorders joint waypoints to the group's joint order."""

    def test_reorders_out_of_order_waypoints(self, abb_irb2400_environment):
        reversed_names = list(reversed(IRB2400_JOINTS))
        program = _joint_program(IRB2400_END[::-1].copy(), names=reversed_names)
        assert formatProgram(program, abb_irb2400_environment) is True
        wp = WaypointPoly_as_JointWaypointPoly(program[0].asMoveInstruction().getWaypoint())
        assert list(wp.getNames()) == IRB2400_JOINTS
        np.testing.assert_allclose(wp.getPosition(), IRB2400_END)

    def test_returns_false_when_already_formatted(self, abb_irb2400_environment):
        program = _joint_program(IRB2400_START, IRB2400_END)
        assert formatProgram(program, abb_irb2400_environment) is False


def _state_program(*states):
    program = CompositeInstruction("DEFAULT")
    program.setManipulatorInfo(_irb2400_manip_info())
    for q in states:
        wp = StateWaypointPoly_wrap_StateWaypoint(StateWaypoint(IRB2400_JOINTS, q))
        program.appendMoveInstruction(
            MoveInstructionPoly_wrap_MoveInstruction(
                MoveInstruction(wp, MoveInstructionType_FREESPACE, "DEFAULT")
            )
        )
    return program


CONTACT_MANAGERS = pytest.mark.parametrize(
    ("evaluator", "manager"),
    [
        (CollisionEvaluatorType.DISCRETE, "getDiscreteContactManager"),
        (CollisionEvaluatorType.CONTINUOUS, "getContinuousContactManager"),
    ],
    ids=["discrete", "continuous"],
)


class TestContactCheckProgram:
    """#303: contactCheckProgram -> (ContactTrajectoryResults, list[ContactResultMap])."""

    @staticmethod
    def _check(env, evaluator, manager, program):
        config = CollisionCheckConfig()
        config.type = evaluator
        return contactCheckProgram(getattr(env, manager)(), env.getStateSolver(), program, config)

    @CONTACT_MANAGERS
    def test_free_program_has_no_contacts(self, abb_irb2400_environment, evaluator, manager):
        program = _state_program(IRB2400_START, IRB2400_END)
        results, contacts = self._check(abb_irb2400_environment, evaluator, manager, program)
        assert isinstance(results, ContactTrajectoryResults)
        assert not results
        assert all(c.count() == 0 for c in contacts)

    @CONTACT_MANAGERS
    def test_colliding_program_reports_the_step(self, abb_irb2400_environment, evaluator, manager):
        program = _state_program(IRB2400_START, IRB2400_COLLIDING)
        results, contacts = self._check(abb_irb2400_environment, evaluator, manager, program)
        assert results
        assert results.numContacts() > 0
        assert all(isinstance(c, ContactResultMap) for c in contacts)
        # The contact is at the end of the program. Discrete checks states, so the free
        # start state reports nothing; continuous checks the segment into the contact.
        assert contacts[-1].count() > 0
        if evaluator == CollisionEvaluatorType.DISCRETE:
            assert contacts[0].count() == 0


class TestCheckRequestReason:
    """#332: checkRequestReason gives the C++ failure reason."""

    def test_invalid_request(self):
        request = PlannerRequest()
        assert MotionPlanner.checkRequest(request) is False
        assert MotionPlanner.checkRequestReason(request) == "PlannerRequest environment is nullptr"

    def test_valid_request(self, abb_irb2400_environment):
        request = PlannerRequest()
        request.env = abb_irb2400_environment
        request.instructions = _joint_program(IRB2400_START, IRB2400_END)
        assert MotionPlanner.checkRequest(request) is True
        assert MotionPlanner.checkRequestReason(request) == ""


class TestAssignSolution:
    """#332: assignSolution writes a joint solution into a move instruction."""

    @staticmethod
    def _cartesian_move():
        wp = CartesianWaypointPoly_wrap_CartesianWaypoint(CartesianWaypoint(Isometry3d.Identity()))
        return MoveInstructionPoly_wrap_MoveInstruction(
            MoveInstruction(wp, MoveInstructionType_FREESPACE, "DEFAULT")
        )

    def test_replaces_waypoint_with_state(self):
        mi = self._cartesian_move()
        MotionPlanner.assignSolution(mi, IRB2400_JOINTS, IRB2400_END, False)
        wp = mi.getWaypoint()
        assert wp.isStateWaypoint()
        swp = WaypointPoly_as_StateWaypointPoly(wp)
        assert list(swp.getNames()) == IRB2400_JOINTS
        np.testing.assert_allclose(swp.getPosition(), IRB2400_END)

    def test_format_as_input_seeds_cartesian(self):
        mi = self._cartesian_move()
        MotionPlanner.assignSolution(mi, IRB2400_JOINTS, IRB2400_END, True)
        wp = mi.getWaypoint()
        assert wp.isCartesianWaypoint()
        seed = WaypointPoly_as_CartesianWaypointPoly(wp).getSeed()
        np.testing.assert_allclose(seed.position, IRB2400_END)


def _waypoint_positions(composite):
    """Joint positions of every move in a (possibly nested) composite, in order."""
    for instruction in composite:
        if instruction.isCompositeInstruction():
            yield from _waypoint_positions(instruction.asCompositeInstruction())
            continue
        wp = instruction.asMoveInstruction().getWaypoint()
        if wp.isStateWaypoint():
            yield WaypointPoly_as_StateWaypointPoly(wp).getPosition()
        else:
            yield WaypointPoly_as_JointWaypointPoly(wp).getPosition()


class TestMotionPlannerBase:
    """#299: SimpleMotionPlanner is a MotionPlanner."""

    def test_simple_is_motion_planner(self):
        assert isinstance(SimpleMotionPlanner("x"), MotionPlanner)

    def test_inherited_methods(self):
        planner = SimpleMotionPlanner("x")
        assert planner.getName() == "x"
        assert planner.terminate() is False  # upstream: not implemented
        planner.clear()
        clone = planner.clone()
        assert type(clone) is SimpleMotionPlanner
        assert clone.getName() == "x"

    def test_base_solve_dispatches(self, abb_irb2400_environment):
        env = abb_irb2400_environment
        env.setState(IRB2400_JOINTS, IRB2400_START)
        request = PlannerRequest()
        request.env = env
        request.instructions = _joint_program(IRB2400_START, IRB2400_END)
        request.profiles = ProfileDictionary()
        planner = SimpleMotionPlanner("x")
        via_base = MotionPlanner.solve(planner, request)
        direct = planner.solve(request)
        assert via_base.successful and direct.successful
        via_base_positions = list(_waypoint_positions(via_base.results))
        direct_positions = list(_waypoint_positions(direct.results))
        assert len(via_base_positions) == len(direct_positions) > 0  # zip(strict=True) is 3.10+
        for a, b in zip(via_base_positions, direct_positions):
            np.testing.assert_allclose(a, b)


# ---- robot configuration (#333) ----------------------------------------------

IRB2400_IK_TARGET = np.array([0.8, 0.0, 1.0])


def _irb2400_ik_solutions(env):
    manip = env.getKinematicGroup("manipulator")
    target = (
        Isometry3d.Identity()
        * Translation3d(*IRB2400_IK_TARGET)
        * Quaterniond.from_xyzw(0, 0.70710678, 0, 0.70710678)
    )
    return manip, manip.calcInvKin(KinGroupIKInput(target, "base_link", "tool0"), np.zeros(6))


class TestRobotConfig:
    def test_enum_members(self):
        assert [c.name for c in RobotConfig] == [
            "NUT",
            "FUT",
            "NDT",
            "FDT",
            "NDB",
            "FDB",
            "NUB",
            "FUB",
        ]

    def test_home_state_is_fut(self, abb_irb2400_environment):
        manip = abb_irb2400_environment.getKinematicGroup("manipulator")
        # j5 = 0 → flip (F); j3 = 0 < π/2 → up (U); tool0 in front of j1 (T).
        assert getRobotConfig(manip, "base_link", "tool0", np.zeros(6)) == RobotConfig.FUT

    def test_ik_solutions_have_distinct_configs(self, abb_irb2400_environment):
        # The IRB2400 joint-3 limits leave the two wrist configurations of the elbow-up,
        # front family; OPW's other six solutions are outside the limits.
        manip, solutions = _irb2400_ik_solutions(abb_irb2400_environment)
        configs = [getRobotConfig(manip, "base_link", "tool0", q) for q in solutions]
        assert sorted(c.name for c in configs) == ["FUT", "NUT"]

    def test_joint_turns(self):
        np.testing.assert_array_equal(getJointTurns(np.zeros(6)), np.zeros(6))
        np.testing.assert_array_equal(
            getJointTurns(np.array([0.0, 0.0, 0.0, 4.0, 0.0, -7.0])), [0, 0, 0, 1, 0, -2]
        )

    def test_is_within_joint_limits(self, abb_irb2400_environment):
        manip = abb_irb2400_environment.getKinematicGroup("manipulator")
        limits = manip.getLimits().joint_limits
        wide_coupling = np.array([-10.0, 10.0])
        assert isWithinJointLimits(np.zeros(6), limits, wide_coupling) is True
        # j4 + j6 = 0.4 outside a tight coupling window
        q = np.array([0.0, 0.0, 0.0, 0.2, 0.0, 0.2])
        assert isWithinJointLimits(q, limits, np.array([-0.1, 0.1])) is False
        assert isWithinJointLimits(limits[:, 1] + 0.1, limits, wide_coupling) is False
