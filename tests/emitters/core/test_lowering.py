"""lower(): tesseract CompositeInstruction → typed event IR (brand-free)."""

from __future__ import annotations

import numpy as np
import pytest

from tesseract_robotics.emitters.core.errors import (
    EmptyProgramError,
    UnsupportedInstructionError,
)
from tesseract_robotics.emitters.core.events import (
    CartesianMove,
    Dwell,
    MoveKind,
    SetDigital,
    ToolChange,
)
from tesseract_robotics.emitters.core.lowering import lower
from tesseract_robotics.planning import Pose
from tesseract_robotics.tesseract_command_language import (
    CartesianWaypoint,
    CartesianWaypointPoly_wrap_CartesianWaypoint,
    CompositeInstruction,
    MoveInstruction,
    MoveInstructionPoly_wrap_MoveInstruction,
    MoveInstructionType_CIRCULAR,
    MoveInstructionType_LINEAR,
    SetToolInstruction,
    TimerInstruction,
    TimerInstructionType,
    WaitInstruction,
)


def _cart_move(pose, move_type=MoveInstructionType_LINEAR, profile="P"):
    wp = CartesianWaypointPoly_wrap_CartesianWaypoint(CartesianWaypoint(pose))
    return MoveInstructionPoly_wrap_MoveInstruction(MoveInstruction(wp, move_type, profile))


def _composite(*instructions):
    c = CompositeInstruction("test")
    for i in instructions:
        c.push_back(i)
    return c


def test_linear_cartesian_lowers_to_cartesianmove() -> None:
    ir = lower(_composite(_cart_move(Pose.from_xyz(0.6, -0.1, 0.8))))
    assert isinstance(ir.events[0], CartesianMove)
    assert ir.events[0].kind is MoveKind.LINEAR
    assert ir.events[0].profile == "P"
    assert np.allclose(ir.events[0].pose.translation, [0.6, -0.1, 0.8])


def test_timer_lowers_to_two_events_dwell_then_setdigital() -> None:
    ir = lower(_composite(TimerInstruction(TimerInstructionType.DIGITAL_OUTPUT_HIGH, 2.0, 7)))
    assert isinstance(ir.events[0], Dwell) and ir.events[0].seconds == 2.0
    assert isinstance(ir.events[1], SetDigital)
    assert ir.events[1].key is None and ir.events[1].index == 7 and ir.events[1].value is True


def test_settool_lowers_to_toolchange() -> None:
    ir = lower(_composite(SetToolInstruction(7)))
    assert isinstance(ir.events[0], ToolChange) and ir.events[0].tool_id == 7


def test_wait_time_lowers_to_dwell() -> None:
    ir = lower(_composite(WaitInstruction(1.5)))
    assert isinstance(ir.events[0], Dwell) and ir.events[0].seconds == 1.5


def test_circular_raises() -> None:
    with pytest.raises(UnsupportedInstructionError, match="CIRCULAR"):
        lower(_composite(_cart_move(Pose.from_xyz(0.5, 0, 0.5), MoveInstructionType_CIRCULAR)))


def test_empty_raises() -> None:
    with pytest.raises(EmptyProgramError, match="no leaf instructions"):
        lower(CompositeInstruction("empty"))


def test_nested_composite_recurses() -> None:
    inner = _composite(_cart_move(Pose.from_xyz(0.5, 0, 0.5)))
    outer = _composite(inner, _cart_move(Pose.from_xyz(0.5, 0, 0.5)))
    ir = lower(outer)
    assert sum(isinstance(e, CartesianMove) for e in ir.events) == 2
