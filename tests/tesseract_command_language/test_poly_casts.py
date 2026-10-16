"""Named `as<T>()` casts on the waypoint and move-instruction Polys (#268).

Each cast returns a reference into the Poly, so an edit through it changes the stored object.
"""

import numpy as np
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
    WaypointPoly,
)
from tesseract_robotics.tesseract_common import Isometry3d, Translation3d


def _cartesian_move():
    return MoveInstruction(
        CartesianWaypointPoly(CartesianWaypoint(Isometry3d.Identity())),
        MoveInstructionType.LINEAR,
        "P",
    )


def test_waypoint_poly_cast_edits_reach_the_instruction():
    mi = _cartesian_move()
    target = Isometry3d.Identity() * Translation3d(0.1, 0.2, 0.3)
    mi.getWaypoint().asCartesianWaypoint().setTransform(target)
    np.testing.assert_allclose(
        mi.getWaypoint().asCartesianWaypoint().getTransform().matrix, target.matrix
    )


def test_waypoint_poly_casts_return_typed_polys():
    assert isinstance(
        WaypointPoly(JointWaypoint(["a"], [1.0])).asJointWaypoint(), JointWaypointPoly
    )
    assert isinstance(
        WaypointPoly(StateWaypoint(["a"], [1.0])).asStateWaypoint(), StateWaypointPoly
    )
    assert isinstance(
        WaypointPoly(CartesianWaypoint(Isometry3d.Identity())).asCartesianWaypoint(),
        CartesianWaypointPoly,
    )


def test_wrong_cast_raises():
    with pytest.raises(RuntimeError):
        _cartesian_move().getWaypoint().asJointWaypoint()


def test_typed_poly_casts_return_concrete_waypoints():
    cwp = CartesianWaypointPoly(CartesianWaypoint(Isometry3d.Identity()))
    assert isinstance(cwp.asCartesianWaypoint(), CartesianWaypoint)
    jwp = JointWaypointPoly(JointWaypoint(["a"], [1.0]))
    jwp.asJointWaypoint().setPosition([2.0])
    assert list(jwp.getPosition()) == [2.0]
    swp = StateWaypointPoly(StateWaypoint(["a"], [1.0]))
    swp.asStateWaypoint().setTime(4.0)
    assert swp.getTime() == 4.0


def test_move_instruction_poly_cast():
    mi = _cartesian_move()
    mip = MoveInstructionPoly(mi)
    assert mip.asMoveInstruction().getProfile() == mi.getProfile()
    mip.asMoveInstruction().setProfile("Q")
    assert mip.getProfile() == "Q"
