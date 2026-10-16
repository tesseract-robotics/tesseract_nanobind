"""Tests for tesseract_motion_planners_simple profile bindings.

Covers the 2 profile bases and 7 concrete move profiles, the
ProfileDictionary registration helpers, the interpolation functions and
SimplePlannerMoveProfile.generate.
"""

import math
from pathlib import Path

import numpy as np
import pytest

from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    InstructionPoly,
    JointWaypoint,
    JointWaypointPoly_wrap_JointWaypoint,
    MoveInstruction,
    MoveInstructionPoly,
    MoveInstructionPoly_wrap_MoveInstruction,
    MoveInstructionType_FREESPACE,
    ProfileDictionary,
    WaypointPoly,
    WaypointPoly_as_CartesianWaypointPoly,
    WaypointPoly_as_JointWaypointPoly,
    WaypointPoly_as_StateWaypointPoly,
)
from tesseract_robotics.tesseract_common import (
    GeneralResourceLocator,
    Isometry3d,
    ManipulatorInfo,
    Quaterniond,
    VectorIsometry3d,
)
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_motion_planners import PlannerRequest
from tesseract_robotics.tesseract_motion_planners_simple import (
    SimpleMotionPlanner,
    SimplePlannerCompositeProfile,
    SimplePlannerFixedSizeAssignMoveProfile,
    SimplePlannerFixedSizeAssignNoIKMoveProfile,
    SimplePlannerFixedSizeMoveProfile,
    SimplePlannerLVSAssignMoveProfile,
    SimplePlannerLVSAssignNoIKMoveProfile,
    SimplePlannerLVSMoveProfile,
    SimplePlannerLVSNoIKMoveProfile,
    SimplePlannerMoveProfile,
    getInterpolatedInstructions,
    interpolate,
    interpolate_waypoint,
)

# ---- Fixed-size profiles ---------------------------------------------------

FIXED_SIZE_CLASSES = [
    SimplePlannerFixedSizeMoveProfile,
    SimplePlannerFixedSizeAssignMoveProfile,
    SimplePlannerFixedSizeAssignNoIKMoveProfile,
]


@pytest.mark.parametrize("cls", FIXED_SIZE_CLASSES)
class TestFixedSizeProfiles:
    def test_default_construction(self, cls):
        p = cls()
        assert p is not None
        # C++ defaults: freespace_steps=10, linear_steps=10
        assert p.freespace_steps == 10
        assert p.linear_steps == 10

    def test_custom_construction(self, cls):
        p = cls(freespace_steps=25, linear_steps=7)
        assert p.freespace_steps == 25
        assert p.linear_steps == 7

    def test_member_assignment(self, cls):
        p = cls()
        p.freespace_steps = 42
        p.linear_steps = 3
        assert p.freespace_steps == 42
        assert p.linear_steps == 3

    def test_is_simple_move_profile(self, cls):
        p = cls()
        assert isinstance(p, SimplePlannerMoveProfile)


# ---- LVS profiles ----------------------------------------------------------

LVS_CLASSES = [
    SimplePlannerLVSMoveProfile,
    SimplePlannerLVSNoIKMoveProfile,
    SimplePlannerLVSAssignMoveProfile,
    SimplePlannerLVSAssignNoIKMoveProfile,
]


@pytest.mark.parametrize("cls", LVS_CLASSES)
class TestLVSProfiles:
    def test_default_construction(self, cls):
        p = cls()
        assert p is not None
        # C++ defaults: 5°, 0.1m, 5°, min_steps=1, max_steps=INT_MAX
        expected_rad = 5.0 * math.pi / 180.0
        assert p.state_longest_valid_segment_length == pytest.approx(expected_rad)
        assert p.translation_longest_valid_segment_length == pytest.approx(0.1)
        assert p.rotation_longest_valid_segment_length == pytest.approx(expected_rad)
        assert p.min_steps == 1
        # max_steps defaults to INT_MAX — just make sure it's big
        assert p.max_steps > 1000

    def test_custom_construction(self, cls):
        p = cls(
            state_longest_valid_segment_length=0.05,
            translation_longest_valid_segment_length=0.2,
            rotation_longest_valid_segment_length=0.1,
            min_steps=3,
            max_steps=99,
        )
        assert p.state_longest_valid_segment_length == pytest.approx(0.05)
        assert p.translation_longest_valid_segment_length == pytest.approx(0.2)
        assert p.rotation_longest_valid_segment_length == pytest.approx(0.1)
        assert p.min_steps == 3
        assert p.max_steps == 99

    def test_member_assignment(self, cls):
        p = cls()
        p.min_steps = 5
        p.max_steps = 50
        p.translation_longest_valid_segment_length = 0.25
        assert p.min_steps == 5
        assert p.max_steps == 50
        assert p.translation_longest_valid_segment_length == pytest.approx(0.25)

    def test_is_simple_move_profile(self, cls):
        p = cls()
        assert isinstance(p, SimplePlannerMoveProfile)


# ---- Composite base --------------------------------------------------------


class TestSimplePlannerCompositeProfile:
    def test_default_construction(self):
        p = SimplePlannerCompositeProfile()
        assert p is not None


# ---- ProfileDictionary helpers ---------------------------------------------


class TestProfileDictionaryHelpers:
    def test_add_move_profile(self):
        d = ProfileDictionary()
        profile = SimplePlannerFixedSizeMoveProfile(5, 8)
        d.addProfile("ns", "DEFAULT", profile)
        assert d.hasProfile(profile.getKey(), "ns", "DEFAULT") is True

    def test_add_composite_profile(self):
        d = ProfileDictionary()
        profile = SimplePlannerCompositeProfile()
        d.addProfile("ns", "DEFAULT", profile)
        assert d.hasProfile(profile.getKey(), "ns", "DEFAULT") is True


# ---- value equality (#261) ------------------------------------------------

ALL_MOVE_CLASSES = FIXED_SIZE_CLASSES + LVS_CLASSES


def _int_field(cls):
    return "linear_steps" if cls in FIXED_SIZE_CLASSES else "min_steps"


@pytest.mark.parametrize("cls", ALL_MOVE_CLASSES)
class TestValueEquality:
    def test_equal_when_fields_equal(self, cls):
        assert cls() == cls()
        assert not (cls() != cls())

    def test_unequal_when_a_field_differs(self, cls):
        other = cls()
        setattr(other, _int_field(cls), 3)
        assert cls() != other
        assert not (cls() == other)

    def test_unhashable(self, cls):
        with pytest.raises(TypeError):
            hash(cls())


# ---- createKey (#273) ------------------------------------------------------

KEYED_CLASSES = [SimplePlannerCompositeProfile, *ALL_MOVE_CLASSES]


@pytest.mark.parametrize("cls", KEYED_CLASSES)
def test_create_key_matches_instance_key(cls):
    assert cls.createKey() == cls().getKey()


def test_create_key_distinguishes_bases():
    assert SimplePlannerMoveProfile.createKey() != SimplePlannerCompositeProfile.createKey()


def test_move_profiles_share_the_base_key():
    # Every move profile registers under SimplePlannerMoveProfile's key (upstream ctor).
    assert SimplePlannerLVSMoveProfile.createKey() == SimplePlannerMoveProfile.createKey()


# ---- from_yaml (#331) ------------------------------------------------------


@pytest.mark.parametrize("cls", FIXED_SIZE_CLASSES)
def test_fixed_size_from_yaml(cls):
    assert cls.from_yaml("linear_steps: 7") == cls(linear_steps=7)


@pytest.mark.parametrize("cls", LVS_CLASSES)
def test_lvs_from_yaml(cls):
    assert cls.from_yaml("min_steps: 4") == cls(min_steps=4)


@pytest.mark.parametrize("cls", ALL_MOVE_CLASSES)
def test_from_yaml_malformed_raises_value_error(cls):
    with pytest.raises(ValueError):
        cls.from_yaml("min_steps: [1, 2")


@pytest.mark.parametrize("cls", ALL_MOVE_CLASSES)
def test_from_yaml_wrong_type_raises(cls):
    field = _int_field(cls)
    with pytest.raises(RuntimeError, match=cls.__name__):
        cls.from_yaml(f"{field}: not-a-number")


# ---- interpolation functions (#305) ----------------------------------------

STEPS = 4


def test_interpolate_poses():
    q_a = Quaterniond.Identity()
    q_b = Quaterniond.from_rpy(0.0, 0.0, math.pi / 2)
    a = Isometry3d([0.0, 0.0, 0.0], q_a)
    b = Isometry3d([1.0, 2.0, 3.0], q_b)
    poses = interpolate(a, b, STEPS)
    assert isinstance(poses, VectorIsometry3d)
    assert len(poses) == STEPS + 1
    np.testing.assert_allclose(poses[0].matrix, a.matrix, atol=1e-12)
    np.testing.assert_allclose(poses[STEPS].matrix, b.matrix, atol=1e-12)
    mid = poses[STEPS // 2]
    np.testing.assert_allclose(mid.translation, [0.5, 1.0, 1.5], atol=1e-12)
    np.testing.assert_allclose(mid.linear, q_a.slerp(0.5, q_b).toRotationMatrix(), atol=1e-12)


def test_interpolate_joints_is_linspace():
    q0 = np.array([0.0, 1.0, -1.0])
    q1 = np.array([1.0, 0.0, 2.0])
    np.testing.assert_allclose(interpolate(q0, q1, STEPS), np.linspace(q0, q1, STEPS + 1).T)


def _joint_wp(values, names=("a", "b")):
    return JointWaypointPoly_wrap_JointWaypoint(JointWaypoint(list(names), values))


def test_interpolate_waypoint_joints():
    waypoints = interpolate_waypoint(
        _move([0.0, 0.0]).getWaypoint(), _move([1.0, 2.0]).getWaypoint(), STEPS
    )
    assert len(waypoints) == STEPS + 1
    assert all(isinstance(w, WaypointPoly) and w.isJointWaypoint() for w in waypoints)
    np.testing.assert_allclose(
        WaypointPoly_as_JointWaypointPoly(waypoints[2]).getPosition(), [0.5, 1.0]
    )


def _move(values, profile="RASTER"):
    return MoveInstructionPoly_wrap_MoveInstruction(
        MoveInstruction(_joint_wp(values), MoveInstructionType_FREESPACE, profile)
    )


def test_get_interpolated_instructions_joint_states():
    base = _move([1.0, 2.0])
    states = np.linspace([0.0, 0.0], [1.0, 2.0], STEPS + 1).T
    instructions = getInterpolatedInstructions(["a", "b"], states, base)
    # Upstream skips the first column (the previous instruction's state).
    assert len(instructions) == STEPS
    assert all(isinstance(mi, MoveInstructionPoly) for mi in instructions)
    assert all(mi.getProfile() == "RASTER" for mi in instructions)
    for mi, col in zip(instructions[:-1], states.T[1:-1]):
        np.testing.assert_allclose(
            WaypointPoly_as_JointWaypointPoly(mi.getWaypoint()).getPosition(), col
        )
    np.testing.assert_allclose(
        WaypointPoly_as_JointWaypointPoly(instructions[-1].getWaypoint()).getPosition(),
        [1.0, 2.0],
    )


def test_get_interpolated_instructions_poses():
    base = _move([1.0, 2.0])
    states = np.linspace([0.0, 0.0], [1.0, 2.0], STEPS + 1).T
    poses = interpolate(
        Isometry3d.Identity(), Isometry3d([1.0, 0.0, 0.0], Quaterniond.Identity()), STEPS
    )
    instructions = getInterpolatedInstructions(poses, ["a", "b"], states, base)
    assert len(instructions) == STEPS
    # A joint base instruction gets Cartesian children seeded with the joint states.
    child = WaypointPoly_as_CartesianWaypointPoly(instructions[1].getWaypoint())
    np.testing.assert_allclose(child.getTransform().translation, [0.5, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(child.getSeed().position, states[:, 2])


# ---- SimplePlannerMoveProfile.generate (#339) -----------------------------

IRB2400_JOINTS = [f"joint_{i + 1}" for i in range(6)]
IRB2400_START = np.zeros(6)
IRB2400_END = np.array([0.5, 0.2, -0.3, 0.4, 0.6, -0.2])
PLANNER_NAME = "SIMPLE"


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


def _positions(instructions):
    return [
        WaypointPoly_as_JointWaypointPoly(mi.getWaypoint()).getPosition()
        if mi.getWaypoint().isJointWaypoint()
        else WaypointPoly_as_StateWaypointPoly(mi.getWaypoint()).getPosition()
        for mi in instructions
    ]


def test_generate_matches_planner_segment(irb2400_env):
    manip_info = ManipulatorInfo("manipulator", "base_link", "tool0")
    start = MoveInstructionPoly_wrap_MoveInstruction(
        MoveInstruction(
            JointWaypointPoly_wrap_JointWaypoint(JointWaypoint(IRB2400_JOINTS, IRB2400_START)),
            MoveInstructionType_FREESPACE,
            "DEFAULT",
        )
    )
    end = MoveInstructionPoly_wrap_MoveInstruction(
        MoveInstruction(
            JointWaypointPoly_wrap_JointWaypoint(JointWaypoint(IRB2400_JOINTS, IRB2400_END)),
            MoveInstructionType_FREESPACE,
            "DEFAULT",
        )
    )
    profile = SimplePlannerLVSMoveProfile()
    generated = profile.generate(start, start, end, InstructionPoly(), irb2400_env, manip_info)
    assert len(generated) > 1

    program = CompositeInstruction("DEFAULT")
    program.setManipulatorInfo(manip_info)
    program.appendMoveInstruction(start)
    program.appendMoveInstruction(end)
    profiles = ProfileDictionary()
    profiles.addProfile(PLANNER_NAME, "DEFAULT", profile)
    request = PlannerRequest()
    request.env = irb2400_env
    request.instructions = program
    request.profiles = profiles
    response = SimpleMotionPlanner(PLANNER_NAME).solve(request)
    assert response.successful, response.message

    planned = [i.asMoveInstruction() for i in response.results.getInstructions()][1:]
    assert len(planned) == len(generated)
    for got, want in zip(_positions(generated), _positions(planned)):
        np.testing.assert_allclose(got, want, atol=1e-12)
