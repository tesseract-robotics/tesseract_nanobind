"""The `std::vector` members `CompositeInstruction` forwards (#269)."""

import pytest

from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    JointWaypoint,
    JointWaypointPoly,
    MoveInstruction,
    MoveInstructionPoly,
    MoveInstructionType,
    WaitInstruction,
)


def _composite(n):
    c = CompositeInstruction()
    for i in range(n):
        c.push_back(WaitInstruction(float(i)))
    return c


def _times(c):
    return [ip.asWaitInstruction().getWaitTime() for ip in c]


def test_insert_puts_the_instruction_at_the_index():
    c = _composite(3)
    c.insert(1, WaitInstruction(9.0))
    assert _times(c) == [0.0, 9.0, 1.0, 2.0]


def test_insert_follows_list_semantics_at_the_ends():
    c = _composite(2)
    expected = [0.0, 1.0]
    for index, value in ((0, 7.0), (len(expected) + 1, 8.0), (100, 9.0), (-1, 6.0), (-100, 5.0)):
        c.insert(index, WaitInstruction(value))
        expected.insert(index, value)
        assert _times(c) == expected


def test_insert_move_instruction_overloads():
    c = _composite(1)
    mi = MoveInstruction(
        JointWaypointPoly(JointWaypoint(["a"], [1.0])), MoveInstructionType.FREESPACE
    )
    c.insert(0, mi)
    c.insert(0, MoveInstructionPoly(mi))
    c.insert(0, CompositeInstruction())
    assert c[0].isCompositeInstruction()
    assert c[1].isMoveInstruction()
    assert c[2].isMoveInstruction()
    assert len(c) == 4


def test_delitem_int_and_slice():
    c = _composite(5)
    reference = _times(c)
    del c[0]
    del reference[0]
    assert _times(c) == reference
    del c[1:3]
    del reference[1:3]
    assert _times(c) == reference
    del c[-1]
    del reference[-1]
    assert _times(c) == reference


def test_delitem_extended_slice():
    c = _composite(6)
    reference = _times(c)
    del c[::2]
    del reference[::2]
    assert _times(c) == reference


def test_delitem_out_of_range():
    with pytest.raises(IndexError):
        del _composite(2)[2]


def test_pop_back():
    c = _composite(2)
    c.pop_back()
    assert _times(c) == [0.0]
    c.pop_back()
    with pytest.raises(IndexError):
        c.pop_back()


def test_front_back_at_are_references():
    c = _composite(3)
    c.front().setDescription("first")
    assert c[0].getDescription() == "first"
    c.back().setDescription("last")
    assert c[2].getDescription() == "last"
    c.at(1).setDescription("mid")
    assert c[1].getDescription() == "mid"
    with pytest.raises(IndexError):
        c.at(3)


def test_front_back_on_empty_raise():
    c = CompositeInstruction()
    with pytest.raises(IndexError):
        c.front()
    with pytest.raises(IndexError):
        c.back()


def test_capacity_members():
    c = _composite(2)
    c.reserve(10)
    assert c.capacity() >= 10
    c.shrink_to_fit()
    assert c.capacity() >= len(c)
    assert c.max_size() > c.capacity()


def test_reversed_without_dunder_reversed():
    c = _composite(4)
    assert [ip.asWaitInstruction().getWaitTime() for ip in reversed(c)] == _times(c)[::-1]
