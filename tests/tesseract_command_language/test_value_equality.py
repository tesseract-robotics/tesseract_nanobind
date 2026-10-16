"""Value equality on the command-language types (#261).

Each class binds its C++ operator==/operator!= as __eq__/__ne__ and sets __hash__ = None.
Instruction equality includes the UUID and the parent UUID, so two instructions built
separately are unequal until their UUIDs are copied over (#262).
"""

import pytest

from tesseract_robotics.tesseract_command_language import (
    CartesianWaypoint,
    CartesianWaypointPoly,
    CompositeInstruction,
    InstructionPoly,
    JointWaypoint,
    JointWaypointPoly,
    MoveInstruction,
    MoveInstructionPoly,
    MoveInstructionType,
    SetAnalogInstruction,
    SetDigitalInstruction,
    SetToolInstruction,
    StateWaypoint,
    StateWaypointPoly,
    TimerInstruction,
    TimerInstructionType,
    WaitInstruction,
    WaypointPoly,
)
from tesseract_robotics.tesseract_common import Isometry3d, Translation3d

NIL_UUID = "00000000-0000-0000-0000-000000000000"
SHIFTED = Isometry3d.Identity() * Translation3d(0.0, 0.0, 1.0)


def _move(position=1.0):
    return MoveInstruction(
        JointWaypointPoly(JointWaypoint(["a"], [position])), MoveInstructionType.FREESPACE
    )


def _composite(profile="P"):
    c = CompositeInstruction(profile)
    c.push_back(WaitInstruction(1.0))
    return c


# (make, make_other): two make() calls give equal values once their UUIDs match;
# make_other() differs in one field.
CASES = {
    "CartesianWaypoint": (
        lambda: CartesianWaypoint(Isometry3d.Identity()),
        lambda: CartesianWaypoint(SHIFTED),
    ),
    "JointWaypoint": (lambda: JointWaypoint(["a"], [1.0]), lambda: JointWaypoint(["a"], [2.0])),
    "StateWaypoint": (lambda: StateWaypoint(["a"], [1.0]), lambda: StateWaypoint(["a"], [2.0])),
    "WaypointPoly": (
        lambda: WaypointPoly(CartesianWaypoint(Isometry3d.Identity())),
        lambda: WaypointPoly(CartesianWaypoint(SHIFTED)),
    ),
    "CartesianWaypointPoly": (
        lambda: CartesianWaypointPoly(CartesianWaypoint(Isometry3d.Identity())),
        lambda: CartesianWaypointPoly(CartesianWaypoint(SHIFTED)),
    ),
    "JointWaypointPoly": (
        lambda: JointWaypointPoly(JointWaypoint(["a"], [1.0])),
        lambda: JointWaypointPoly(JointWaypoint(["a"], [2.0])),
    ),
    "StateWaypointPoly": (
        lambda: StateWaypointPoly(StateWaypoint(["a"], [1.0])),
        lambda: StateWaypointPoly(StateWaypoint(["a"], [2.0])),
    ),
    "MoveInstruction": (_move, lambda: _move(2.0)),
    "MoveInstructionPoly": (
        lambda: MoveInstructionPoly(_move()),
        lambda: MoveInstructionPoly(_move(2.0)),
    ),
    "InstructionPoly": (lambda: InstructionPoly(_move()), lambda: InstructionPoly(_move(2.0))),
    "CompositeInstruction": (_composite, lambda: _composite("Q")),
    "WaitInstruction": (lambda: WaitInstruction(1.0), lambda: WaitInstruction(2.0)),
    "TimerInstruction": (
        lambda: TimerInstruction(TimerInstructionType.DIGITAL_OUTPUT_HIGH, 1.0, 0),
        lambda: TimerInstruction(TimerInstructionType.DIGITAL_OUTPUT_HIGH, 2.0, 0),
    ),
    "SetAnalogInstruction": (
        lambda: SetAnalogInstruction("k", 0, 1.0),
        lambda: SetAnalogInstruction("k", 0, 2.0),
    ),
    "SetDigitalInstruction": (
        lambda: SetDigitalInstruction("k", 0, True),
        lambda: SetDigitalInstruction("k", 0, False),
    ),
    "SetToolInstruction": (lambda: SetToolInstruction(1), lambda: SetToolInstruction(2)),
}

parametrize = pytest.mark.parametrize(("make", "make_other"), CASES.values(), ids=CASES.keys())


def _align_uuids(src, dst):
    """Copy the UUIDs of `src` onto `dst`, for the types that carry them."""
    # WaitInstruction's constructors leave the UUID nil and setUUID rejects nil (upstream 0.35.0);
    # two nil UUIDs already match.
    if hasattr(src, "setUUID") and src.getUUID() != NIL_UUID:
        dst.setUUID(src.getUUID())
        dst.setParentUUID(src.getParentUUID())
    return dst


@parametrize
def test_equal_when_built_alike(make, make_other):
    a = make()
    b = _align_uuids(a, make())
    assert a is not b
    assert a == b
    assert not (a != b)


@parametrize
def test_unequal_when_one_field_differs(make, make_other):
    a = make()
    b = _align_uuids(a, make_other())
    assert a != b
    assert not (a == b)


@parametrize
def test_unhashable(make, make_other):
    obj = make()
    assert type(obj).__hash__ is None
    with pytest.raises(TypeError, match="unhashable"):
        hash(obj)


@parametrize
def test_foreign_operand_is_unequal_without_raising(make, make_other):
    obj = make()
    assert (obj == object()) is False
    assert (obj != object()) is True


def test_separately_built_moves_differ_by_uuid():
    a, b = _move(), _move()
    assert a.getUUID() != b.getUUID()
    assert a != b
    b.setUUID(a.getUUID())
    assert a == b
    b.setParentUUID("12345678-1234-5678-1234-567812345678")
    assert a != b


def test_null_polys_are_equal():
    for cls in (
        WaypointPoly,
        CartesianWaypointPoly,
        JointWaypointPoly,
        StateWaypointPoly,
        InstructionPoly,
        MoveInstructionPoly,
    ):
        assert cls() == cls()


def test_waypoint_polys_from_equal_waypoints_are_equal():
    assert WaypointPoly(CartesianWaypoint(Isometry3d.Identity())) == WaypointPoly(
        CartesianWaypoint(Isometry3d.Identity())
    )
