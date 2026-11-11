"""The `float` Descartes surface: the twin of every `D` type, read by the shipped pipelines (#350)."""

from pathlib import Path
from typing import override

import numpy as np
import pytest

import tesseract_robotics.motion_planners_descartes as descartes
from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.command_language import (
    CartesianWaypoint,
    CartesianWaypointPoly,
    CompositeInstruction,
    MoveInstruction,
    MoveInstructionPoly,
    MoveInstructionType,
    ProfileDictionary,
)
from tesseract_robotics.common import (
    GeneralResourceLocator,
    Isometry3d,
    ManipulatorInfo,
    Quaterniond,
    Translation3d,
)
from tesseract_robotics.environment import Environment
from tesseract_robotics.motion_planners import (
    MotionPlanner,
    PlannerRequest,
    ProfileOverrideNotDispatchedError,
)
from tesseract_robotics.motion_planners_simple import generateInterpolatedProgram

DESCARTES_DEFAULT_NAMESPACE = "DescartesMotionPlannerTask"


@pytest.fixture
def irb2400():
    """The ABB IRB2400 environment and its manipulator (analytic OPW kinematics)."""
    locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    support = "package://tesseract/support/urdf/abb_irb2400"
    urdf, srdf = (locator.locateResource(support + suffix) for suffix in (".urdf", ".srdf"))
    assert urdf is not None and srdf is not None
    env = Environment()
    assert env.init(Path(urdf.getFilePath()), Path(srdf.getFilePath()), locator)
    manip_info = ManipulatorInfo("manipulator", "base_link", "tool0")
    manip_info.manipulator_ik_solver = "OPWInvKin"
    return env, manip_info


def test_state_holds_float32_values():
    values = np.array([0.1, -0.2, 0.3], dtype=np.float32)

    state = descartes.DescartesStateF(values)

    assert state.values.dtype == np.float32
    np.testing.assert_array_equal(state.values, values)


def test_state_sample_keeps_state_and_cost():
    state = descartes.DescartesStateF(np.array([0.5, 0.25], dtype=np.float32))

    sample = descartes.DescartesStateSampleF(state, 1.5)

    assert sample.state is state
    assert sample.cost == 1.5


@pytest.mark.parametrize(
    "name",
    ["DescartesDefaultMoveProfile", "DescartesLadderGraphSolverProfile"],
)
def test_create_key_matches_instance_key(name):
    cls = getattr(descartes, f"{name}F")
    assert cls.createKey() == cls().getKey()


@pytest.mark.parametrize("base", ["DescartesMoveProfile", "DescartesSolverProfile"])
def test_float_and_double_keys_differ(base):
    assert getattr(descartes, f"{base}F").createKey() != getattr(descartes, f"{base}D").createKey()


def test_default_move_profile_equality():
    a, b = descartes.DescartesDefaultMoveProfileF(), descartes.DescartesDefaultMoveProfileF()
    assert a == b
    b.target_pose_sample_resolution = 0.5
    assert a != b
    with pytest.raises(TypeError):
        hash(a)


def test_ladder_graph_solver_profile_equality():
    a = descartes.DescartesLadderGraphSolverProfileF()
    b = descartes.DescartesLadderGraphSolverProfileF()
    assert a == b
    b.num_threads = 4
    assert a != b


@pytest.mark.parametrize(
    ("concrete", "hook", "base"),
    [
        ("DescartesDefaultMoveProfileF", "createWaypointSampler", "DescartesMoveProfileF"),
        ("DescartesDefaultMoveProfileF", "createEdgeEvaluator", "DescartesMoveProfileF"),
        ("DescartesDefaultMoveProfileF", "createStateEvaluator", "DescartesMoveProfileF"),
        ("DescartesLadderGraphSolverProfileF", "create", "fields"),
    ],
)
def test_concrete_hook_override_raises(concrete, hook, base):
    with pytest.raises(ProfileOverrideNotDispatchedError, match=base):
        type("Overrides", (getattr(descartes, concrete),), {hook: lambda self, *args: None})


def test_planner_is_a_motion_planner():
    planner = descartes.DescartesMotionPlannerF(DESCARTES_DEFAULT_NAMESPACE)
    assert isinstance(planner, MotionPlanner)
    assert planner.getName() == DESCARTES_DEFAULT_NAMESPACE


def _two_pose_program(manip_info, profile):
    """Two tool-down poses 0.4 m apart, interpolated by the caller."""
    program = CompositeInstruction()
    program.setManipulatorInfo(manip_info)
    for y in (-0.2, 0.2):
        pose = (
            Isometry3d.Identity() * Translation3d(0.8, y, 0.8) * Quaterniond.from_xyzw(0, -1, 0, 0)
        )
        move = MoveInstruction(
            CartesianWaypointPoly(CartesianWaypoint(pose)), MoveInstructionType.LINEAR, profile
        )
        move.setManipulatorInfo(manip_info)
        program.push_back(MoveInstructionPoly(move))
    return program


def test_python_profile_reaches_the_float_planner(irb2400):
    """A Python `DescartesMoveProfileF` drives `DescartesMotionPlannerF`, on float32 states."""
    env, manip_info = irb2400
    seen_dtypes = set()

    class JointDistance(descartes.DescartesEdgeEvaluatorF):
        @override
        def evaluate(self, start, end):
            seen_dtypes.update((start.values.dtype, end.values.dtype))
            return (True, float(np.linalg.norm(end.values - start.values)))

    class PythonMoveProfile(descartes.DescartesMoveProfileF):
        def __init__(self):
            super().__init__()
            self._inner = descartes.DescartesDefaultMoveProfileF()
            self._edge_evaluator = JointDistance()

        @override
        def createWaypointSampler(self, move_instruction, composite_manip_info, env):
            return self._inner.createWaypointSampler(move_instruction, composite_manip_info, env)

        @override
        def createEdgeEvaluator(self, move_instruction, composite_manip_info, env):
            return self._edge_evaluator

        @override
        def createStateEvaluator(self, move_instruction, composite_manip_info, env):
            return self._inner.createStateEvaluator(move_instruction, composite_manip_info, env)

    profiles = ProfileDictionary()
    profiles.addProfile(DESCARTES_DEFAULT_NAMESPACE, "PY_PROFILE", PythonMoveProfile())
    request = PlannerRequest()
    request.instructions = generateInterpolatedProgram(
        _two_pose_program(manip_info, "PY_PROFILE"), env, 3.14, 1.0, 3.14, 10
    )
    request.env = env
    request.profiles = profiles

    response = descartes.DescartesMotionPlannerF(DESCARTES_DEFAULT_NAMESPACE).solve(request)

    assert response.successful, response.message
    assert seen_dtypes == {np.dtype(np.float32)}
