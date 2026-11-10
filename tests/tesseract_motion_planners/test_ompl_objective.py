"""OMPL optimization objectives from Python (#362, the binding half).

A Python `OMPLRealVectorMoveProfile` subclass overrides `createOptimizationObjective`
and returns an `ompl::base` objective; the planner's `optimize` loop then ends when
that objective's cost threshold is met instead of when `planning_time` runs out.
Scene: the KUKA IIWA sweep around the sphere of `freespace_hybrid_example.py`.
"""

import math
import time

import numpy as np
import pytest

from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.planning import Pose, Robot, create_obstacle, sphere
from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    MoveInstruction,
    MoveInstructionPoly,
    MoveInstructionType,
    ProfileDictionary,
    StateWaypoint,
    StateWaypointPoly,
)
from tesseract_robotics.tesseract_common import GeneralResourceLocator, ManipulatorInfo
from tesseract_robotics.tesseract_motion_planners import PlannerRequest
from tesseract_robotics.tesseract_motion_planners_ompl import (
    Cost,
    MaximizeMinClearanceObjective,
    MechanicalWorkOptimizationObjective,
    MultiOptimizationObjective,
    OMPLMotionPlanner,
    OMPLObjectiveReturnTypeError,
    OMPLRealVectorMoveProfile,
    OMPLStateCostError,
    OptimizationObjective,
    PathLengthOptimizationObjective,
    RRTConnectConfigurator,
    SpaceInformation,
    StateCostIntegralObjective,
)

OMPL_NAMESPACE = "OMPLMotionPlannerTask"
PROFILE = "FREESPACE"

# freespace_hybrid_example.py (from the C++ example): joint_a1 -0.4 -> 0.4 rad around a
# 0.15 m sphere at (0.5, 0, 0.55).
JOINT_START = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
JOINT_END = np.array([0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
SPHERE_RADIUS_M = 0.15
SPHERE_CENTER = (0.5, 0.0, 0.55)
PLANNER_RANGE = 0.01  # the C++ example's RRTConnect range (rad)

# Long enough that a solve running to `planning_time` cannot pass the wall-time bound.
LONG_PLANNING_TIME_S = 10.0
# No solution-count exit: only the objective or the time cap ends the optimize loop.
UNREACHABLE_MAX_SOLUTIONS = 1_000_000
# A threshold-satisfied solve on this scene takes well under 1 s; 2 s is the bound.
THRESHOLD_SOLVE_BOUND_S = 2.0
CLEARANCE_WEIGHT = 0.5
# Joint count of the IIWA's manipulator group: the length of every state stateCost sees.
IIWA_DOF = 7


@pytest.fixture(scope="module")
def iiwa_sphere_env():
    robot = Robot.from_tesseract_support(
        "lbr_iiwa_14_r820", GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    )
    create_obstacle(
        robot, "sphere_attached", sphere(SPHERE_RADIUS_M), Pose.from_xyz(*SPHERE_CENTER)
    )
    joint_names = robot.get_joint_names("manipulator")
    robot.set_joints(JOINT_START, joint_names=joint_names)
    return robot.env, joint_names


def _request(env, joint_names, profile: OMPLRealVectorMoveProfile) -> PlannerRequest:
    manip_info = ManipulatorInfo()
    manip_info.manipulator = "manipulator"
    manip_info.tcp_frame = "tool0"
    manip_info.working_frame = "base_link"
    program = CompositeInstruction(PROFILE)
    program.setManipulatorInfo(manip_info)
    for joints in (JOINT_START, JOINT_END):
        wp = StateWaypointPoly(StateWaypoint(joint_names, joints))
        program.push_back(
            MoveInstructionPoly(MoveInstruction(wp, MoveInstructionType.FREESPACE, PROFILE))
        )
    profiles = ProfileDictionary()
    profiles.addProfile(OMPL_NAMESPACE, PROFILE, profile)
    request = PlannerRequest()
    request.instructions = program
    request.env = env
    request.profiles = profiles
    return request


def _configure(profile: OMPLRealVectorMoveProfile, planning_time_s: float) -> None:
    rrt = RRTConnectConfigurator()
    rrt.range = PLANNER_RANGE
    profile.solver_config.planners = [rrt, RRTConnectConfigurator()]
    profile.solver_config.planning_time = planning_time_s
    profile.solver_config.optimize = True
    profile.solver_config.max_solutions = UNREACHABLE_MAX_SOLUTIONS


class LengthAndClearance(OMPLRealVectorMoveProfile):
    """Path length x 1 + max-min clearance x w, with an optional cost threshold."""

    def __init__(self, threshold: float | None = None):
        super().__init__()
        self.threshold = threshold
        self.calls = 0
        self.seen: dict = {}

    def createOptimizationObjective(self, si, env, manip):
        self.calls += 1
        self.seen.update(si=si, env=env, manip=manip)
        multi = MultiOptimizationObjective(si)
        multi.addObjective(PathLengthOptimizationObjective(si), 1.0)
        multi.addObjective(MaximizeMinClearanceObjective(si), CLEARANCE_WEIGHT)
        if self.threshold is not None:
            multi.setCostThreshold(Cost(self.threshold))
        self.seen["objective_count"] = multi.getObjectiveCount()
        return multi


def test_python_objective_is_used(iiwa_sphere_env):
    env, joint_names = iiwa_sphere_env
    profile = LengthAndClearance(threshold=math.inf)
    _configure(profile, LONG_PLANNING_TIME_S)

    response = OMPLMotionPlanner(OMPL_NAMESPACE).solve(_request(env, joint_names, profile))

    assert response.successful, response.message
    assert profile.calls == 1
    assert isinstance(profile.seen["si"], SpaceInformation)
    assert profile.seen["manip"].getJointIds() == joint_names
    assert profile.seen["objective_count"] == 2


def test_threshold_ends_optimize_loop(iiwa_sphere_env):
    env, joint_names = iiwa_sphere_env
    profile = LengthAndClearance(threshold=math.inf)
    _configure(profile, LONG_PLANNING_TIME_S)
    request = _request(env, joint_names, profile)

    start = time.perf_counter()
    response = OMPLMotionPlanner(OMPL_NAMESPACE).solve(request)
    elapsed = time.perf_counter() - start

    assert response.successful, response.message
    assert elapsed < THRESHOLD_SOLVE_BOUND_S, (
        f"solve took {elapsed:.2f} s; the threshold should end the optimize loop at the "
        f"first solution, not at planning_time = {LONG_PLANNING_TIME_S} s"
    )


def test_objective_api(iiwa_sphere_env):
    """Cost, thresholds and the concrete objectives, built where an si exists."""
    env, joint_names = iiwa_sphere_env
    checks: dict = {}

    class Probe(OMPLRealVectorMoveProfile):
        def createOptimizationObjective(self, si, env, manip):
            length = PathLengthOptimizationObjective(si)
            length.setCostThreshold(Cost(2.0))
            checks["threshold"] = length.getCostThreshold().value()
            checks["below"] = length.isSatisfied(Cost(1.0))
            checks["above"] = length.isSatisfied(Cost(3.0))
            work = MechanicalWorkOptimizationObjective(si)
            checks["work_is_objective"] = isinstance(work, OptimizationObjective)
            return length

    profile = Probe()
    _configure(profile, LONG_PLANNING_TIME_S)
    OMPLMotionPlanner(OMPL_NAMESPACE).solve(_request(env, joint_names, profile))

    assert checks == {"threshold": 2.0, "below": True, "above": False, "work_is_objective": True}
    assert Cost(1.5).value() == 1.5


def test_override_must_return_an_objective(iiwa_sphere_env):
    env, joint_names = iiwa_sphere_env

    class Wrong(OMPLRealVectorMoveProfile):
        def createOptimizationObjective(self, si, env, manip):
            return "path length"

    profile = Wrong()
    _configure(profile, LONG_PLANNING_TIME_S)
    with pytest.raises(OMPLObjectiveReturnTypeError, match="createOptimizationObjective"):
        OMPLMotionPlanner(OMPL_NAMESPACE).solve(_request(env, joint_names, profile))


class StateCostProfile(OMPLRealVectorMoveProfile):
    """Returns `make_objective(si)` with a cost threshold every path meets.

    The infinite threshold ends the optimize loop at the first solution, after the
    planner has priced it with the objective (so `stateCost` runs).
    """

    def __init__(self, make_objective):
        super().__init__()
        self.make_objective = make_objective
        self.objective = None

    def createOptimizationObjective(self, si, env, manip):
        self.objective = self.make_objective(si)
        self.objective.setCostThreshold(Cost(math.inf))
        return self.objective


def _solve_state_cost(iiwa_sphere_env, make_objective):
    env, joint_names = iiwa_sphere_env
    profile = StateCostProfile(make_objective)
    _configure(profile, LONG_PLANNING_TIME_S)
    response = OMPLMotionPlanner(OMPL_NAMESPACE).solve(_request(env, joint_names, profile))
    return profile, response


def test_state_cost_integral_default_is_path_length(iiwa_sphere_env):
    """Without an override the C++ stateCost (Cost(1)) applies: the integral is path length."""
    profile, response = _solve_state_cost(iiwa_sphere_env, StateCostIntegralObjective)

    assert response.successful, response.message
    assert isinstance(profile.objective, OptimizationObjective)
    assert profile.objective.isMotionCostInterpolationEnabled() is False


class CountingStateCost(StateCostIntegralObjective):
    """stateCost = 1 + joint_a1^2, recording every state it is handed."""

    def __init__(self, si):
        super().__init__(si)
        self.calls = 0
        self.lengths: set[int] = set()

    def stateCost(self, values):
        self.calls += 1
        self.lengths.add(len(values))
        return 1.0 + values[0] ** 2


def test_python_state_cost_is_called(iiwa_sphere_env):
    profile, response = _solve_state_cost(iiwa_sphere_env, CountingStateCost)

    assert response.successful, response.message
    assert profile.objective.calls > 0
    assert profile.objective.lengths == {IIWA_DOF}


def test_state_cost_wrong_return_raises(iiwa_sphere_env):
    class WrongReturn(StateCostIntegralObjective):
        def stateCost(self, values):
            return "x"

    with pytest.raises(OMPLStateCostError, match="stateCost"):
        _solve_state_cost(iiwa_sphere_env, WrongReturn)


def test_state_cost_exception_propagates(iiwa_sphere_env):
    """A Python error in stateCost, raised on a planner thread, surfaces from solve (no crash)."""

    class Raises(StateCostIntegralObjective):
        def stateCost(self, values):
            raise ZeroDivisionError("state cost")

    with pytest.raises(ZeroDivisionError, match="state cost"):
        _solve_state_cost(iiwa_sphere_env, Raises)


def test_objective_override_exception_propagates(iiwa_sphere_env):
    """A Python error in createOptimizationObjective surfaces as itself, not as RuntimeError."""
    env, joint_names = iiwa_sphere_env

    class Raises(OMPLRealVectorMoveProfile):
        def createOptimizationObjective(self, si, env, manip):
            raise ZeroDivisionError("objective")

    profile = Raises()
    _configure(profile, LONG_PLANNING_TIME_S)
    with pytest.raises(ZeroDivisionError, match="objective"):
        OMPLMotionPlanner(OMPL_NAMESPACE).solve(_request(env, joint_names, profile))
