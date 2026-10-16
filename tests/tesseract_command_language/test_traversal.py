"""`CompositeInstruction` traversal: flatten, locate, counts, moveFilter (#270)."""

import gc

import pytest

from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    JointWaypoint,
    JointWaypointPoly,
    MoveInstruction,
    MoveInstructionPoly,
    MoveInstructionType,
    WaitInstruction,
    moveFilter,
)


def _move(profile):
    return MoveInstruction(
        JointWaypointPoly(JointWaypoint(["a"], [0.0])), MoveInstructionType.FREESPACE, profile
    )


def _nested():
    """[child[move c0, wait, move c1], wait, move top]: 5 leaves, 3 moves."""
    child = CompositeInstruction("CHILD")
    child.push_back(_move("c0"))
    child.push_back(WaitInstruction(1.0))
    child.push_back(_move("c1"))
    top = CompositeInstruction("TOP")
    top.push_back(child)
    top.push_back(WaitInstruction(2.0))
    top.push_back(_move("top"))
    return top


def test_flatten_counts_leaves_of_both_levels():
    assert len(_nested().flatten()) == 5


def test_flatten_with_move_filter():
    moves = _nested().flatten(moveFilter)
    assert [ip.asMoveInstruction().getProfile() for ip in moves] == ["c0", "c1", "top"]


def test_flatten_with_python_filter():
    waits = _nested().flatten(lambda ip, parent: ip.isWaitInstruction())
    assert len(waits) == 2


def test_first_and_last_move_descend_into_children():
    c = _nested()
    first = c.getFirstMoveInstruction()
    assert isinstance(first, MoveInstructionPoly)
    assert first.getProfile() == "c0"
    assert c.getLastMoveInstruction().getProfile() == "top"


def test_first_move_on_composite_without_moves_is_none():
    c = CompositeInstruction()
    c.push_back(WaitInstruction(1.0))
    assert c.getFirstMoveInstruction() is None
    assert c.getLastMoveInstruction() is None


def test_first_and_last_instruction():
    c = _nested()
    # Without a filter every instruction matches, child composites included (upstream 0.35.0).
    assert c.getFirstInstruction().isCompositeInstruction()
    assert c.getFirstInstruction(moveFilter).asMoveInstruction().getProfile() == "c0"
    assert c.getFirstInstruction(moveFilter, False).asMoveInstruction().getProfile() == "top"
    assert c.getLastInstruction().asMoveInstruction().getProfile() == "top"
    first_wait = c.getFirstInstruction(lambda ip, parent: ip.isWaitInstruction())
    assert first_wait.asWaitInstruction().getWaitTime() == 1.0
    assert c.getFirstInstruction(lambda ip, parent: False) is None


def test_counts():
    c = _nested()
    assert c.getMoveInstructionCount() == 3
    # Without a filter the child composite counts too: 5 leaves + 1 composite.
    assert c.getInstructionCount() == 6
    assert c.getInstructionCount(None, False) == 3
    assert c.getInstructionCount(moveFilter) == 3
    assert c.getInstructionCount(moveFilter, False) == 1


def test_first_move_is_a_reference():
    c = _nested()
    c.getFirstMoveInstruction().setProfile("edited")
    assert c.flatten(moveFilter)[0].asMoveInstruction().getProfile() == "edited"


def test_flatten_elements_are_references_that_keep_the_composite_alive():
    c = _nested()
    leaves = c.flatten()
    leaves[1].setDescription("edited")
    assert c[0].asCompositeInstruction()[1].getDescription() == "edited"
    del c
    gc.collect()
    assert leaves[1].getDescription() == "edited"
    assert leaves[4].isMoveInstruction()


def test_filter_exception_propagates():
    def boom(ip, parent):
        raise ValueError("boom")

    with pytest.raises(ValueError, match="boom"):
        _nested().flatten(boom)
    with pytest.raises(ValueError, match="boom"):
        _nested().getInstructionCount(boom)
