"""Command-language constructors that wrap or fully initialise a value (#264)."""

import pytest

from tesseract_robotics.tesseract_command_language import (
    DEFAULT_PROFILE_KEY,
    CartesianWaypoint,
    CartesianWaypointPoly,
    CompositeInstruction,
    InstructionPoly,
    JointWaypoint,
    JointWaypointPoly,
    MoveInstruction,
    MoveInstructionPoly,
    MoveInstructionType,
    StateWaypoint,
    StateWaypointPoly,
    WaypointPoly,
)
from tesseract_robotics.tesseract_common import Isometry3d, ManipulatorInfo


def _joint_waypoint():
    return JointWaypoint(["j1", "j2"], [0.1, 0.2])


def _state_waypoint():
    return StateWaypoint(["j1", "j2"], [0.1, 0.2])


WAYPOINT_POLY_SOURCES = {
    "CartesianWaypoint": (lambda: CartesianWaypoint(Isometry3d.Identity()), "isCartesianWaypoint"),
    "JointWaypoint": (_joint_waypoint, "isJointWaypoint"),
    "StateWaypoint": (_state_waypoint, "isStateWaypoint"),
    "CartesianWaypointPoly": (
        lambda: CartesianWaypointPoly(CartesianWaypoint(Isometry3d.Identity())),
        "isCartesianWaypoint",
    ),
    "JointWaypointPoly": (lambda: JointWaypointPoly(_joint_waypoint()), "isJointWaypoint"),
    "StateWaypointPoly": (lambda: StateWaypointPoly(_state_waypoint()), "isStateWaypoint"),
}


@pytest.mark.parametrize(
    ("make", "predicate"), WAYPOINT_POLY_SOURCES.values(), ids=WAYPOINT_POLY_SOURCES.keys()
)
def test_waypoint_poly_wraps_each_alternative(make, predicate):
    wp = WaypointPoly(make())
    assert not wp.isNull()
    assert getattr(wp, predicate)()


def _move():
    return MoveInstruction(JointWaypointPoly(_joint_waypoint()), MoveInstructionType.FREESPACE, "P")


def test_move_instruction_poly_from_move_instruction():
    mi = _move()
    assert MoveInstructionPoly(mi).getProfile() == mi.getProfile()


def test_instruction_poly_from_move_instruction():
    assert InstructionPoly(_move()).isMoveInstruction()


def test_push_back_bare_move_instruction():
    composite = CompositeInstruction()
    composite.push_back(_move())
    assert len(composite) == 1
    assert composite[0].isMoveInstruction()


def test_move_instruction_full_constructor_with_path_profile():
    info = ManipulatorInfo("manip", "base", "tool0")
    mi = MoveInstruction(
        JointWaypointPoly(_joint_waypoint()), MoveInstructionType.FREESPACE, "P", "PP", info
    )
    assert mi.getProfile() == "P"
    assert mi.getPathProfile() == "PP"
    assert mi.getManipulatorInfo() == info


def test_move_instruction_manipulator_info_keyword():
    info = ManipulatorInfo("manip", "base", "tool0")
    mi = MoveInstruction(
        CartesianWaypointPoly(CartesianWaypoint(Isometry3d.Identity())),
        MoveInstructionType.LINEAR,
        manipulator_info=info,
    )
    assert mi.getProfile() == DEFAULT_PROFILE_KEY
    assert mi.getManipulatorInfo() == info


def test_move_instruction_from_waypoint_poly():
    wp = WaypointPoly(_joint_waypoint())
    mi = MoveInstruction(wp, MoveInstructionType.FREESPACE)
    assert mi.getWaypoint().isJointWaypoint()


def test_state_waypoint_full_constructor():
    wp = StateWaypoint(["j1", "j2"], [0.0, 1.0], [0.0, 0.0], [0.0, 0.0], 2.5)
    assert wp.getTime() == 2.5
    assert list(wp.getPosition()) == [0.0, 1.0]
