"""Waypoint-Poly tolerances, seed and effort (#267); move-instruction members (#266)."""

import pytest

from tesseract_robotics.tesseract_command_language import (
    CartesianWaypoint,
    CartesianWaypointPoly,
    JointWaypoint,
    JointWaypointPoly,
    MoveInstruction,
    MoveInstructionPoly,
    MoveInstructionType,
    StateWaypoint,
    StateWaypointPoly,
)
from tesseract_robotics.tesseract_common import Isometry3d, JointState

# ---- #267 ----


def test_cartesian_waypoint_poly_tolerances():
    wp = CartesianWaypointPoly(CartesianWaypoint(Isometry3d.Identity()))
    assert not wp.isToleranced()
    lower = [-0.1, -0.1, -0.1, -0.2, -0.2, -0.2]
    upper = [0.1, 0.1, 0.1, 0.2, 0.2, 0.2]
    wp.setLowerTolerance(lower)
    wp.setUpperTolerance(upper)
    assert list(wp.getLowerTolerance()) == lower
    assert list(wp.getUpperTolerance()) == upper
    assert wp.isToleranced()


def test_cartesian_waypoint_poly_set_seed():
    wp = CartesianWaypointPoly(CartesianWaypoint(Isometry3d.Identity()))
    assert not wp.hasSeed()
    wp.setSeed(JointState(["a", "b"], [0.5, 0.25]))
    assert wp.hasSeed()
    assert list(wp.getSeed().position) == [0.5, 0.25]


def test_joint_waypoint_poly_tolerances():
    wp = JointWaypointPoly(JointWaypoint(["a", "b"], [0.0, 0.0]))
    assert not wp.isToleranced()
    wp.setLowerTolerance([-0.1, -0.1])
    wp.setUpperTolerance([0.1, 0.1])
    assert list(wp.getLowerTolerance()) == [-0.1, -0.1]
    assert list(wp.getUpperTolerance()) == [0.1, 0.1]
    assert wp.isToleranced()


def test_state_waypoint_poly_effort():
    wp = StateWaypointPoly(StateWaypoint(["a", "b"], [0.0, 0.0]))
    wp.setEffort([1.5, -2.5])
    assert list(wp.getEffort()) == [1.5, -2.5]


# ---- #266 ----


def _move(move_type=MoveInstructionType.FREESPACE):
    return MoveInstruction(JointWaypointPoly(JointWaypoint(["a"], [1.0])), move_type)


@pytest.mark.parametrize(
    "wrap", [lambda mi: mi, MoveInstructionPoly], ids=["MoveInstruction", "MoveInstructionPoly"]
)
def test_create_waypoints(wrap):
    mi = wrap(_move())
    for method, cls in (
        ("createCartesianWaypoint", CartesianWaypointPoly),
        ("createJointWaypoint", JointWaypointPoly),
        ("createStateWaypoint", StateWaypointPoly),
    ):
        wp = getattr(mi, method)()
        assert isinstance(wp, cls)
        assert not wp.isNull()


def test_create_child():
    mip = MoveInstructionPoly(_move())
    child = mip.createChild()
    assert isinstance(child, MoveInstructionPoly)
    assert child.isChild()
    assert not mip.isChild()
    assert child.getParentUUID() == mip.getUUID()
    assert child.getUUID() != mip.getUUID()


@pytest.mark.parametrize("move_type", list(MoveInstructionType.__members__.values()))
def test_move_type_predicates(move_type):
    mip = MoveInstructionPoly(_move(move_type))
    assert mip.isLinear() == (move_type == MoveInstructionType.LINEAR)
    assert mip.isFreespace() == (move_type == MoveInstructionType.FREESPACE)
    assert mip.isCircular() == (move_type == MoveInstructionType.CIRCULAR)
