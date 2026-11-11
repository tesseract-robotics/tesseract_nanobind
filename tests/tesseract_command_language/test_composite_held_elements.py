"""CompositeInstruction references stay valid: a call that would move or free a held one raises.

`c[i]`, front/back/at, iteration, flatten and getFirst/Last* return references into the
composite's `std::vector<InstructionPoly>`. A reallocating or erasing call while Python holds
one used to leave it on freed memory; now it raises `InstructionHeldError` and changes nothing.
"""

import copy

import numpy as np
import pytest

from tesseract_robotics.command_language import (
    CompositeInstruction,
    InstructionHeldError,
    JointWaypoint,
    JointWaypointPoly,
    MoveInstruction,
    MoveInstructionType,
    PolyTypeError,
    WaitInstruction,
)

JOINTS = ["j1", "j2"]


def _move(value: float) -> MoveInstruction:
    waypoint = JointWaypointPoly(JointWaypoint(JOINTS, np.array([value, value])))
    return MoveInstruction(waypoint, MoveInstructionType.FREESPACE, "DEFAULT")


def _program(n: int) -> CompositeInstruction:
    program = CompositeInstruction("DEFAULT")
    for i in range(n):
        program.push_back(_move(float(i)))
    program.shrink_to_fit()  # capacity == size: the next push_back reallocates
    return program


def _position(instruction) -> list[float]:
    return list(instruction.asMoveInstruction().getWaypoint().asJointWaypoint().getPosition())


def test_push_back_that_reallocates_refuses_while_an_element_is_held():
    program = _program(3)
    held = program[0]
    with pytest.raises(InstructionHeldError, match="element 0"):
        program.push_back(_move(9.0))
    assert len(program) == 3
    assert _position(held) == [0.0, 0.0]
    del held
    program.push_back(_move(9.0))
    assert len(program) == 4


def test_push_back_within_capacity_keeps_held_elements_valid():
    program = _program(3)
    program.reserve(8)
    held = program[1]
    program.push_back(_move(9.0))
    assert _position(held) == [1.0, 1.0]


@pytest.mark.parametrize(
    "mutate",
    [
        lambda p: p.clear(),
        lambda p: p.pop_back(),
        lambda p: p.__delitem__(2),
        lambda p: p.__delitem__(slice(0, 3)),
        lambda p: p.insert(0, _move(9.0)),
        lambda p: p.setInstructions([]),
        lambda p: p.reserve(100),
    ],
    ids=["clear", "pop_back", "delitem", "delslice", "insert", "setInstructions", "reserve"],
)
def test_mutators_refuse_while_the_last_element_is_held(mutate):
    program = _program(3)
    held = program.back()
    with pytest.raises(InstructionHeldError):
        mutate(program)
    assert len(program) == 3
    assert _position(held) == [2.0, 2.0]


def test_destroying_a_child_composite_refuses_while_a_reference_into_it_is_held():
    child = CompositeInstruction("DEFAULT")
    child.push_back(_move(5.0))
    program = CompositeInstruction("DEFAULT")
    program.push_back(child)
    first_move = program.getFirstMoveInstruction()  # a reference two levels down
    with pytest.raises(InstructionHeldError, match="reference into it"):
        program.clear()
    waypoint = first_move.getWaypoint()
    del first_move
    with pytest.raises(InstructionHeldError):  # the waypoint keeps its move instruction alive
        program.clear()
    del waypoint
    program.clear()
    assert len(program) == 0


def test_erasing_after_a_held_element_leaves_it_alone():
    program = _program(3)
    held = program[0]
    del program[2]
    assert _position(held) == [0.0, 0.0]


def test_a_copy_does_not_pin_the_composite():
    program = _program(3)
    kept = copy.copy(program[0])
    program.clear()
    assert kept.isMoveInstruction()


def test_iteration_sees_appends_and_survives_reallocation():
    program = _program(2)
    seen = []
    for instruction in program:
        seen.append(_position(instruction)[0])
        del instruction
        if len(program) < 4:
            program.push_back(_move(float(len(program))))
    assert seen == [0.0, 1.0, 2.0, 3.0]


def test_as_views_write_through():
    program = _program(1)
    move = program[0].asMoveInstruction()
    move.setWaypoint(JointWaypointPoly(JointWaypoint(JOINTS, np.array([7.0, 8.0]))))
    del move
    assert _position(program[0]) == [7.0, 8.0]


def test_as_view_of_a_child_composite_writes_through():
    program = CompositeInstruction("DEFAULT")
    program.push_back(CompositeInstruction("DEFAULT"))
    program[0].asCompositeInstruction().push_back(WaitInstruction(1.0))
    assert len(program[0].asCompositeInstruction()) == 1


def test_wrong_alternative_raises_poly_type_error_naming_both():
    program = CompositeInstruction("DEFAULT")
    program.push_back(WaitInstruction(1.0))
    with pytest.raises(PolyTypeError, match="holds a WaitInstruction, not a MoveInstructionPoly"):
        program[0].asMoveInstruction()
    assert issubclass(PolyTypeError, TypeError)
    assert issubclass(InstructionHeldError, BufferError)
