"""Instruction UUIDs as canonical strings (#262)."""

import uuid

import pytest

from tesseract_robotics.tesseract_command_language import (
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
    TimerInstruction,
    TimerInstructionType,
    WaitInstruction,
)

NIL = str(uuid.UUID(int=0))


def _move():
    return MoveInstruction(
        JointWaypointPoly(JointWaypoint(["a"], [1.0])), MoveInstructionType.FREESPACE
    )


INSTRUCTIONS = {
    "MoveInstruction": _move,
    "MoveInstructionPoly": lambda: MoveInstructionPoly(_move()),
    "InstructionPoly": lambda: InstructionPoly(_move()),
    "CompositeInstruction": CompositeInstruction,
    "SetAnalogInstruction": lambda: SetAnalogInstruction("k", 0, 1.0),
    "SetDigitalInstruction": lambda: SetDigitalInstruction("k", 0, True),
    "SetToolInstruction": lambda: SetToolInstruction(1),
    "TimerInstruction": lambda: TimerInstruction(TimerInstructionType.DIGITAL_OUTPUT_HIGH, 1.0, 0),
    "WaitInstruction": lambda: WaitInstruction(1.0),
}

parametrize = pytest.mark.parametrize("make", INSTRUCTIONS.values(), ids=INSTRUCTIONS.keys())


@parametrize
def test_get_uuid_is_canonical_string(make):
    s = make().getUUID()
    assert isinstance(s, str)
    assert str(uuid.UUID(s)) == s


@parametrize
def test_set_uuid_round_trips(make):
    x = make()
    s = str(uuid.uuid4())
    x.setUUID(s)
    assert x.getUUID() == s
    x.regenerateUUID()
    assert x.getUUID() != s
    assert x.getUUID() != NIL


@parametrize
def test_parent_uuid_round_trips(make):
    x = make()
    assert x.getParentUUID() == NIL
    s = str(uuid.uuid4())
    x.setParentUUID(s)
    assert x.getParentUUID() == s


@parametrize
def test_malformed_uuid_raises_value_error_and_keeps_old(make):
    x = make()
    s = str(uuid.uuid4())
    x.setUUID(s)
    with pytest.raises(ValueError, match="not-a-uuid"):
        x.setUUID("not-a-uuid")
    assert x.getUUID() == s
    with pytest.raises(ValueError, match="not-a-uuid"):
        x.setParentUUID("not-a-uuid")


@parametrize
def test_non_str_uuid_raises_type_error(make):
    with pytest.raises(TypeError):
        make().setUUID(uuid.uuid4())


def test_poly_keeps_the_wrapped_uuid():
    mi = _move()
    assert MoveInstructionPoly(mi).getUUID() == mi.getUUID()
    assert InstructionPoly(mi).getUUID() == mi.getUUID()
