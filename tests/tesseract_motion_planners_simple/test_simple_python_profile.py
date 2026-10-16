"""Python-authored simple-planner move profiles and the helpers a `generate` override calls (#340).

The crash-class tests (references that could dangle once Python drops its name) run in a
child interpreter, so a segfault fails the test instead of killing pytest.
"""

from __future__ import annotations

import subprocess
import sys
import textwrap
from pathlib import Path

import numpy as np
import pytest

from tesseract_robotics.tesseract_command_language import (
    CartesianWaypoint,
    CartesianWaypointPoly_wrap_CartesianWaypoint,
    CompositeInstruction,
    InstructionPoly,
    JointWaypoint,
    JointWaypointPoly_wrap_JointWaypoint,
    MoveInstruction,
    MoveInstructionPoly,
    MoveInstructionPoly_wrap_MoveInstruction,
    MoveInstructionType_FREESPACE,
    MoveInstructionType_LINEAR,
    ProfileDictionary,
    WaypointPoly_as_JointWaypointPoly,
    WaypointPoly_as_StateWaypointPoly,
)
from tesseract_robotics.tesseract_common import GeneralResourceLocator, JointState, ManipulatorInfo
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_kinematics import JointGroup, KinematicGroup
from tesseract_robotics.tesseract_motion_planners import PlannerRequest
from tesseract_robotics.tesseract_motion_planners_simple import (
    JointGroupInstructionInfo,
    KinematicGroupInstructionInfo,
    SimpleMotionPlanner,
    SimplePlannerFixedSizeMoveProfile,
    SimplePlannerMoveProfile,
    getClosestJointSolution,
    interpolateCartCartWaypoint,
    interpolateCartJointWaypoint,
    interpolateJointCartWaypoint,
    interpolateJointJointWaypoint,
)

# s: a child loads the IRB2400 environment and plans one segment; about 2 s on an M-series Mac
CHILD_TIMEOUT_S = 120.0
# rad: joint states the planner and IK round-trip agree to (IK is analytic OPW)
JOINT_ATOL = 1e-6

IRB2400_JOINTS = [f"joint_{i + 1}" for i in range(6)]
IRB2400_START = np.zeros(6)
IRB2400_END = np.array([0.5, 0.2, -0.3, 0.4, 0.6, -0.2])
PLANNER_NAME = "SIMPLE"
PROFILE = "PY_PROFILE"
# interpolation steps the fixed-size helpers take for a freespace / linear segment
FREESPACE_STEPS = 5
LINEAR_STEPS = 4


@pytest.fixture
def irb2400_env():
    locator = GeneralResourceLocator()
    urdf = Path(
        locator.locateResource("package://tesseract/support/urdf/abb_irb2400.urdf").getFilePath()
    )
    srdf = Path(
        locator.locateResource("package://tesseract/support/urdf/abb_irb2400.srdf").getFilePath()
    )
    env = Environment()
    assert env.init(urdf, srdf, locator)
    env.setState(IRB2400_JOINTS, IRB2400_START)
    return env


@pytest.fixture
def manip_info():
    return ManipulatorInfo("manipulator", "base_link", "tool0")


def _joint_move(position, profile=PROFILE):
    return MoveInstructionPoly_wrap_MoveInstruction(
        MoveInstruction(
            JointWaypointPoly_wrap_JointWaypoint(JointWaypoint(IRB2400_JOINTS, position)),
            MoveInstructionType_FREESPACE,
            profile,
        )
    )


def _cart_move(env, position, profile=PROFILE, seeded=False):
    """A LINEAR move to the tool0 pose of `position`, in the base_link frame.

    `seeded` sets `position` as the waypoint's seed, which the no-IK helpers read.
    """
    pose = env.getKinematicGroup("manipulator").calcFwdKin(position)["tool0"]
    waypoint = CartesianWaypoint(pose)
    if seeded:
        waypoint.setSeed(JointState(IRB2400_JOINTS, position))
    return MoveInstructionPoly_wrap_MoveInstruction(
        MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(waypoint),
            MoveInstructionType_LINEAR,
            profile,
        )
    )


def _position(instruction):
    waypoint = instruction.getWaypoint()
    if waypoint.isJointWaypoint():
        return WaypointPoly_as_JointWaypointPoly(waypoint).getPosition()
    return WaypointPoly_as_StateWaypointPoly(waypoint).getPosition()


def _request(env, manip_info, profile, end=IRB2400_END):
    program = CompositeInstruction(PROFILE)
    program.setManipulatorInfo(manip_info)
    program.appendMoveInstruction(_joint_move(IRB2400_START))
    program.appendMoveInstruction(_joint_move(end))
    profiles = ProfileDictionary()
    profiles.addProfile(PLANNER_NAME, PROFILE, profile)
    request = PlannerRequest()
    request.env = env
    request.instructions = program
    request.profiles = profiles
    return request


def _planned(response):
    return [i.asMoveInstruction() for i in response.results.getInstructions()][1:]


class EchoProfile(SimplePlannerMoveProfile):
    """Emits the base instruction alone: the smallest valid `generate`."""

    def __init__(self):
        super().__init__()
        self.calls = 0

    def generate(
        self,
        prev_instruction,
        prev_seed,
        base_instruction,
        next_instruction,
        env,
        global_manip_info,
    ):
        self.calls += 1
        return [base_instruction]


class ProfileError(Exception):
    """Raised by `RaisingProfile.generate`; a type no binding translates to."""


class RaisingProfile(SimplePlannerMoveProfile):
    def generate(
        self,
        prev_instruction,
        prev_seed,
        base_instruction,
        next_instruction,
        env,
        global_manip_info,
    ):
        raise ProfileError("from generate")


# ---- the trampoline --------------------------------------------------------


def test_python_profile_plans(irb2400_env, manip_info):
    profile = EchoProfile()
    response = SimpleMotionPlanner(PLANNER_NAME).solve(_request(irb2400_env, manip_info, profile))
    assert response.successful, response.message
    assert profile.calls == 1
    planned = _planned(response)
    assert len(planned) == 1
    np.testing.assert_array_equal(_position(planned[0]), IRB2400_END)
    assert planned[0].getProfile() == PROFILE


def test_generate_receives_the_planner_arguments(irb2400_env, manip_info):
    seen = {}

    class Recorder(SimplePlannerMoveProfile):
        def generate(
            self, prev_instruction, prev_seed, base_instruction, next_instruction, env, mi
        ):
            seen.update(
                prev=prev_instruction, base=base_instruction, next=next_instruction, env=env, mi=mi
            )
            return [base_instruction]

    response = SimpleMotionPlanner(PLANNER_NAME).solve(
        _request(irb2400_env, manip_info, Recorder())
    )
    assert response.successful, response.message
    assert isinstance(seen["base"], MoveInstructionPoly)
    np.testing.assert_array_equal(_position(seen["prev"]), IRB2400_START)
    np.testing.assert_array_equal(_position(seen["base"]), IRB2400_END)
    assert isinstance(seen["next"], InstructionPoly)
    assert seen["next"].isNull()  # the last move
    assert seen["env"] is irb2400_env
    assert seen["mi"].manipulator == "manipulator"


def test_generate_exception_propagates_from_solve(irb2400_env, manip_info):
    planner = SimpleMotionPlanner(PLANNER_NAME)
    with pytest.raises(ProfileError, match="from generate"):
        planner.solve(_request(irb2400_env, manip_info, RaisingProfile()))
    # the error is consumed: the next solve on the same planner and thread is unaffected
    response = planner.solve(_request(irb2400_env, manip_info, EchoProfile()))
    assert response.successful, response.message


def test_generate_without_override_fails_the_plan(irb2400_env, manip_info):
    class NoGenerate(SimplePlannerMoveProfile):
        pass

    response = SimpleMotionPlanner(PLANNER_NAME).solve(
        _request(irb2400_env, manip_info, NoGenerate())
    )
    assert not response.successful
    assert "generate" in response.message


def test_python_profile_matches_cpp_profile(irb2400_env, manip_info):
    """An override built from the bound helpers emits what the C++ fixed-size profile does."""

    class FixedSize(SimplePlannerMoveProfile):
        def generate(
            self, prev_instruction, prev_seed, base_instruction, next_instruction, env, mi
        ):
            prev = KinematicGroupInstructionInfo(prev_instruction, env, mi)
            base = KinematicGroupInstructionInfo(base_instruction, env, mi)
            return interpolateJointJointWaypoint(prev, base, LINEAR_STEPS, FREESPACE_STEPS)

    py = SimpleMotionPlanner(PLANNER_NAME).solve(_request(irb2400_env, manip_info, FixedSize()))
    cpp = SimpleMotionPlanner(PLANNER_NAME).solve(
        _request(
            irb2400_env,
            manip_info,
            SimplePlannerFixedSizeMoveProfile(FREESPACE_STEPS, LINEAR_STEPS),
        )
    )
    assert py.successful, py.message
    assert cpp.successful, cpp.message
    assert len(_planned(py)) == len(_planned(cpp)) == FREESPACE_STEPS
    for got, want in zip(_planned(py), _planned(cpp)):
        np.testing.assert_allclose(_position(got), _position(want), atol=1e-12)


# ---- the helpers -----------------------------------------------------------


def test_instruction_info_fields(irb2400_env, manip_info):
    instruction = _joint_move(IRB2400_END)
    kin = KinematicGroupInstructionInfo(instruction, irb2400_env, manip_info)
    joint = JointGroupInstructionInfo(instruction, irb2400_env, manip_info)
    for info in (kin, joint):
        assert info.working_frame == "base_link"
        assert info.tcp_frame == "tool0"
        assert info.has_cartesian_waypoint is False
        np.testing.assert_array_equal(info.extractJointPosition(), IRB2400_END)
        np.testing.assert_array_equal(_position(info.instruction), IRB2400_END)
    assert isinstance(kin.manip, KinematicGroup)
    assert isinstance(joint.manip, JointGroup)
    fk = irb2400_env.getKinematicGroup("manipulator").calcFwdKin(IRB2400_END)["tool0"]
    np.testing.assert_allclose(kin.calcCartesianPose(IRB2400_END).matrix, fk.matrix, atol=1e-12)
    np.testing.assert_allclose(joint.working_frame_transform.matrix, np.eye(4), atol=1e-12)


def test_instruction_info_extracts_cartesian_pose(irb2400_env, manip_info):
    instruction = _cart_move(irb2400_env, IRB2400_END)
    info = KinematicGroupInstructionInfo(instruction, irb2400_env, manip_info)
    assert info.has_cartesian_waypoint is True
    fk = irb2400_env.getKinematicGroup("manipulator").calcFwdKin(IRB2400_END)["tool0"]
    np.testing.assert_allclose(info.extractCartesianPose().matrix, fk.matrix, atol=1e-12)


def test_get_closest_joint_solution(irb2400_env, manip_info):
    info = KinematicGroupInstructionInfo(
        _cart_move(irb2400_env, IRB2400_END), irb2400_env, manip_info
    )
    np.testing.assert_allclose(
        getClosestJointSolution(info, IRB2400_END), IRB2400_END, atol=JOINT_ATOL
    )
    other = KinematicGroupInstructionInfo(
        _cart_move(irb2400_env, IRB2400_START), irb2400_env, manip_info
    )
    pair = getClosestJointSolution(info, other, IRB2400_END)
    assert isinstance(pair, tuple)
    assert len(pair) == 2
    for solution in pair:
        assert solution.shape == (6,)


# (helper, start is cartesian, end is cartesian, takes a scene state)
_INTERPOLATORS = [
    (interpolateJointJointWaypoint, False, False, False),
    (interpolateJointCartWaypoint, False, True, False),
    (interpolateCartJointWaypoint, True, False, False),
    (interpolateCartCartWaypoint, True, True, True),
]
# state, translation, rotation longest valid segment; min and max steps
_LVS_ARGS = (0.05, 0.05, 0.05, 2, 50)


@pytest.mark.parametrize("info_cls", [KinematicGroupInstructionInfo, JointGroupInstructionInfo])
@pytest.mark.parametrize(("fn", "start_cart", "end_cart", "scene"), _INTERPOLATORS)
def test_interpolate_lvs(irb2400_env, manip_info, info_cls, fn, start_cart, end_cart, scene):
    # the no-IK helpers read the joint state from each Cartesian waypoint's seed
    seeded = info_cls is JointGroupInstructionInfo
    start = (
        _cart_move(irb2400_env, IRB2400_START, seeded=seeded)
        if start_cart
        else _joint_move(IRB2400_START)
    )
    end = (
        _cart_move(irb2400_env, IRB2400_END, seeded=seeded)
        if end_cart
        else _joint_move(IRB2400_END)
    )
    prev = info_cls(start, irb2400_env, manip_info)
    base = info_cls(end, irb2400_env, manip_info)
    extra = (irb2400_env.getState(),) if scene else ()
    instructions = fn(prev, base, *_LVS_ARGS, *extra)
    assert len(instructions) >= _LVS_ARGS[3]
    assert all(isinstance(i, MoveInstructionPoly) for i in instructions)


@pytest.mark.parametrize(("fn", "start_cart", "end_cart", "scene"), _INTERPOLATORS)
def test_interpolate_fixed_size(irb2400_env, manip_info, fn, start_cart, end_cart, scene):
    start = _cart_move(irb2400_env, IRB2400_START) if start_cart else _joint_move(IRB2400_START)
    end = _cart_move(irb2400_env, IRB2400_END) if end_cart else _joint_move(IRB2400_END)
    prev = KinematicGroupInstructionInfo(start, irb2400_env, manip_info)
    base = KinematicGroupInstructionInfo(end, irb2400_env, manip_info)
    extra = (irb2400_env.getState(),) if scene else ()
    instructions = fn(prev, base, LINEAR_STEPS, FREESPACE_STEPS, *extra)
    # a freespace joint end uses the freespace count, a linear Cartesian end the linear count
    assert len(instructions) == (LINEAR_STEPS if end_cart else FREESPACE_STEPS)


# ---- lifetimes, in a child interpreter -------------------------------------

_CHILD_PREAMBLE = textwrap.dedent(
    """
    import gc
    from pathlib import Path

    import numpy as np

    from tesseract_robotics.tesseract_command_language import (
        CompositeInstruction,
        JointWaypoint,
        JointWaypointPoly_wrap_JointWaypoint,
        MoveInstruction,
        MoveInstructionPoly_wrap_MoveInstruction,
        MoveInstructionType_FREESPACE,
        ProfileDictionary,
        WaypointPoly_as_JointWaypointPoly,
    )
    from tesseract_robotics.tesseract_common import GeneralResourceLocator, ManipulatorInfo
    from tesseract_robotics.tesseract_environment import Environment
    from tesseract_robotics.tesseract_motion_planners import PlannerRequest
    from tesseract_robotics.tesseract_motion_planners_simple import (
        JointGroupInstructionInfo,
        KinematicGroupInstructionInfo,
        SimpleMotionPlanner,
        SimplePlannerMoveProfile,
    )

    JOINTS = [f"joint_{i + 1}" for i in range(6)]
    END = [0.5, 0.2, -0.3, 0.4, 0.6, -0.2]

    def make_env():
        locator = GeneralResourceLocator()
        support = "package://tesseract/support/urdf/abb_irb2400"
        env = Environment()
        assert env.init(
            Path(locator.locateResource(support + ".urdf").getFilePath()),
            Path(locator.locateResource(support + ".srdf").getFilePath()),
            locator,
        )
        env.setState(JOINTS, np.zeros(6))
        return env

    def joint_move(position):
        return MoveInstructionPoly_wrap_MoveInstruction(
            MoveInstruction(
                JointWaypointPoly_wrap_JointWaypoint(JointWaypoint(JOINTS, np.array(position))),
                MoveInstructionType_FREESPACE,
                "PY_PROFILE",
            )
        )

    def position(instruction):
        return list(WaypointPoly_as_JointWaypointPoly(instruction.getWaypoint()).getPosition())
    """
)

_INFO_KEEPS_ALIVE = textwrap.dedent(
    """
    manip_info = ManipulatorInfo("manipulator", "base_link", "tool0")
    for cls in (KinematicGroupInstructionInfo, JointGroupInstructionInfo):
        env = make_env()
        info = cls(joint_move(END), env, manip_info)  # the instruction has no other name
        del env  # the info's group came from the env's kinematics plugins
        gc.collect()
        assert position(info.instruction) == END, position(info.instruction)
        assert list(info.extractJointPosition()) == END
        info.calcCartesianPose(np.array(END))
        manip = info.manip
        del info
        gc.collect()
        manip.calcFwdKin(np.array(END))
        del manip
        gc.collect()
    print("OK")
    """
)

_STORED_BASE_INSTRUCTION = textwrap.dedent(
    """
    class Keeper(SimplePlannerMoveProfile):
        def __init__(self):
            super().__init__()
            self.kept = []

        def generate(self, prev_instruction, prev_seed, base_instruction, next_instruction, env, mi):
            self.kept.append((prev_instruction, prev_seed, base_instruction, next_instruction, mi))
            return [base_instruction]

    env = make_env()
    manip_info = ManipulatorInfo("manipulator", "base_link", "tool0")
    keeper = Keeper()
    program = CompositeInstruction("PY_PROFILE")
    program.setManipulatorInfo(manip_info)
    program.appendMoveInstruction(joint_move([0.0] * 6))
    program.appendMoveInstruction(joint_move(END))
    profiles = ProfileDictionary()
    profiles.addProfile("SIMPLE", "PY_PROFILE", keeper)
    request = PlannerRequest()
    request.env = env
    request.instructions = program
    request.profiles = profiles
    response = SimpleMotionPlanner("SIMPLE").solve(request)
    assert response.successful, response.message
    del response, request, program, profiles
    gc.collect()
    prev, seed, base, nxt, mi = keeper.kept[0]
    assert position(base) == END, position(base)
    assert position(prev) == [0.0] * 6
    assert nxt.isNull()
    assert mi.manipulator == "manipulator"
    base.setProfile("TOUCHED")  # writable: a copy, not the planner's const instruction
    assert base.getProfile() == "TOUCHED"
    print("OK")
    """
)


# generate is pure virtual: super().generate from a Python override has nothing to delegate to and
# must raise, not dispatch back into the override until the stack runs out.
_SUPER_GENERATE = textwrap.dedent(
    """
    from tesseract_robotics.tesseract_command_language import InstructionPoly
    from tesseract_robotics.tesseract_motion_planners_simple import (
        SimplePlannerLVSMoveProfile,
        SimplePlannerPureVirtualCallError,
    )

    class CallsUp(SimplePlannerMoveProfile):
        def generate(self, prev_instruction, prev_seed, base_instruction, next_instruction, env, mi):
            return super().generate(
                prev_instruction, prev_seed, base_instruction, next_instruction, env, mi
            )

    class ExtendsLVS(SimplePlannerLVSMoveProfile):
        def generate(self, prev_instruction, prev_seed, base_instruction, next_instruction, env, mi):
            return super().generate(
                prev_instruction, prev_seed, base_instruction, next_instruction, env, mi
            )

    env = make_env()
    manip_info = ManipulatorInfo("manipulator", "base_link", "tool0")
    start, end = joint_move([0.0] * 6), joint_move(END)
    args = (start, start, end, InstructionPoly(), env, manip_info)

    try:
        CallsUp().generate(*args)
    except SimplePlannerPureVirtualCallError as error:
        assert isinstance(error, NotImplementedError)
        assert "generate" in str(error), str(error)
    else:
        raise AssertionError("super().generate did not raise")

    # a Python subclass of a concrete C++ profile delegates to that profile's generate
    assert len(ExtendsLVS().generate(*args)) > 1

    # through solve, the error comes out of SimpleMotionPlanner.solve with its type
    program = CompositeInstruction("PY_PROFILE")
    program.setManipulatorInfo(manip_info)
    program.appendMoveInstruction(start)
    program.appendMoveInstruction(end)
    profiles = ProfileDictionary()
    profiles.addProfile("SIMPLE", "PY_PROFILE", CallsUp())
    request = PlannerRequest()
    request.env = env
    request.instructions = program
    request.profiles = profiles
    try:
        SimpleMotionPlanner("SIMPLE").solve(request)
    except SimplePlannerPureVirtualCallError:
        pass
    else:
        raise AssertionError("solve with super().generate did not raise")
    print("OK")
    """
)


def _run_child(body):
    proc = subprocess.run(
        [sys.executable, "-c", _CHILD_PREAMBLE + body],
        capture_output=True,
        text=True,
        check=False,
        timeout=CHILD_TIMEOUT_S,
    )
    traceback_at = proc.stderr.find("Traceback")
    assert proc.returncode == 0, (
        f"child died (rc={proc.returncode}, SIGSEGV is -11/139): "
        f"{proc.stderr[traceback_at:] if traceback_at >= 0 else proc.stderr}"
    )
    assert "OK" in proc.stdout.splitlines(), proc.stdout


def test_instruction_info_keeps_instruction_and_env_alive():
    _run_child(_INFO_KEEPS_ALIVE)


def test_stored_generate_arguments_outlive_solve():
    _run_child(_STORED_BASE_INSTRUCTION)


def test_super_generate_raises_pure_virtual_call_error():
    _run_child(_SUPER_GENERATE)
