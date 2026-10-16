"""`copy.copy` / `copy.deepcopy` on waypoints, instructions and Polys (#263)."""

import copy

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
    getJointPosition,
    setJointPosition,
)
from tesseract_robotics.tesseract_common import Isometry3d


def _move():
    return MoveInstruction(
        JointWaypointPoly(JointWaypoint(["a"], [1.0])), MoveInstructionType.FREESPACE
    )


def _composite():
    c = CompositeInstruction("P")
    c.push_back(_move())
    return c


VALUES = {
    "CartesianWaypoint": lambda: CartesianWaypoint(Isometry3d.Identity()),
    "JointWaypoint": lambda: JointWaypoint(["a"], [1.0]),
    "StateWaypoint": lambda: StateWaypoint(["a"], [1.0]),
    "MoveInstruction": _move,
    "CompositeInstruction": _composite,
    "SetAnalogInstruction": lambda: SetAnalogInstruction("k", 0, 1.0),
    "SetDigitalInstruction": lambda: SetDigitalInstruction("k", 0, True),
    "SetToolInstruction": lambda: SetToolInstruction(1),
    "TimerInstruction": lambda: TimerInstruction(TimerInstructionType.DIGITAL_OUTPUT_HIGH, 1.0, 0),
    "WaitInstruction": lambda: WaitInstruction(1.0),
    "WaypointPoly": lambda: WaypointPoly(JointWaypoint(["a"], [1.0])),
    "CartesianWaypointPoly": lambda: CartesianWaypointPoly(
        CartesianWaypoint(Isometry3d.Identity())
    ),
    "JointWaypointPoly": lambda: JointWaypointPoly(JointWaypoint(["a"], [1.0])),
    "StateWaypointPoly": lambda: StateWaypointPoly(StateWaypoint(["a"], [1.0])),
    "InstructionPoly": lambda: InstructionPoly(_move()),
    "MoveInstructionPoly": lambda: MoveInstructionPoly(_move()),
}

parametrize = pytest.mark.parametrize("make", VALUES.values(), ids=VALUES.keys())


@parametrize
@pytest.mark.parametrize("copier", [copy.copy, copy.deepcopy], ids=["copy", "deepcopy"])
def test_copy_returns_equal_object_of_same_type(make, copier):
    x = make()
    y = copier(x)
    assert type(y) is type(x)
    assert y is not x
    assert y == x


@pytest.mark.parametrize("copier", [copy.copy, copy.deepcopy], ids=["copy", "deepcopy"])
def test_poly_copy_is_independent(copier):
    x = JointWaypointPoly(JointWaypoint(["a"], [1.0]))
    y = copier(x)
    y.setPosition([2.0])
    assert list(x.getPosition()) == [1.0]


@pytest.mark.parametrize("copier", [copy.copy, copy.deepcopy], ids=["copy", "deepcopy"])
def test_waypoint_poly_copy_is_independent(copier):
    x = WaypointPoly(JointWaypoint(["a"], [1.0]))
    y = copier(x)
    setJointPosition(y, [2.0])
    assert list(getJointPosition(x)) == [1.0]
    assert list(getJointPosition(y)) == [2.0]


@pytest.mark.parametrize("copier", [copy.copy, copy.deepcopy], ids=["copy", "deepcopy"])
def test_move_instruction_poly_copy_is_independent(copier):
    x = MoveInstructionPoly(_move())
    y = copier(x)
    setJointPosition(y.getWaypoint(), [2.0])
    assert list(getJointPosition(x.getWaypoint())) == [1.0]
    assert list(getJointPosition(y.getWaypoint())) == [2.0]


@pytest.mark.parametrize("copier", [copy.copy, copy.deepcopy], ids=["copy", "deepcopy"])
def test_composite_copy_is_independent(copier):
    x = _composite()
    y = copier(x)
    y.push_back(WaitInstruction(1.0))
    assert len(x) == 1
    assert len(y) == 2


def test_deepcopy_inside_a_container():
    x = _composite()
    ys = copy.deepcopy([x, x])
    assert ys[0] == x
    assert ys[0] is ys[1]
