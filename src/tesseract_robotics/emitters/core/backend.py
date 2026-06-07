"""The backend contract and the IR-to-backend driver.

``ProgramBackend`` is the typed, PEP8 descendant of RoboDK's ``RobotPost``
surface, one method per IR event type. ``drive()`` walks a ``ProgramIR`` and
dispatches each event exhaustively — a new event type without a matching method
trips the ``_assert_never`` guard (and pyright flags the unhandled branch).
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from .events import (
    CartesianMove,
    Dwell,
    Event,
    JointMove,
    Note,
    ProgramIR,
    SetAnalog,
    SetDigital,
    ToolChange,
    WaitDigital,
)


@runtime_checkable
class ProgramBackend(Protocol):
    """One method per IR event; ``prog_finish`` returns the filename→text map.

    Inline provenance documents the attic ``RobotPost`` name each descends from.
    """

    def prog_start(self, name: str) -> None:  # attic: RobotPost.ProgStart
        ...

    def move_joint(self, m: JointMove) -> None:  # attic: RobotPost.MoveJ (pose=None)
        ...

    def move_cartesian(self, m: CartesianMove) -> None:  # attic: RobotPost.MoveJ/MoveL
        ...

    def dwell(self, e: Dwell) -> None:  # attic: RobotPost.Pause
        ...

    def wait_digital(self, e: WaitDigital) -> None:  # attic: RobotPost.waitDI
        ...

    def set_digital(self, e: SetDigital) -> None:  # attic: RobotPost.setDO
        ...

    def set_analog(self, e: SetAnalog) -> None:  # attic: RobotPost.setAO
        ...

    def tool_change(self, e: ToolChange) -> None:  # attic: RobotPost.setTool
        ...

    def note(self, e: Note) -> None:  # attic: RobotPost.RunMessage(iscomment=True)
        ...

    def prog_finish(self) -> dict[str, str]:  # attic: RobotPost.ProgFinish + ProgSave
        ...


def _dispatch(event: Event, backend: ProgramBackend) -> None:
    """Send one event to its backend method. Exhaustive over the Event union."""
    if isinstance(event, JointMove):
        backend.move_joint(event)
    elif isinstance(event, CartesianMove):
        backend.move_cartesian(event)
    elif isinstance(event, Dwell):
        backend.dwell(event)
    elif isinstance(event, WaitDigital):
        backend.wait_digital(event)
    elif isinstance(event, SetDigital):
        backend.set_digital(event)
    elif isinstance(event, SetAnalog):
        backend.set_analog(event)
    elif isinstance(event, ToolChange):
        backend.tool_change(event)
    elif isinstance(event, Note):
        backend.note(event)
    else:  # pragma: no cover
        _assert_never(event)


def _assert_never(value: object) -> None:
    raise AssertionError(f"unhandled IR event type: {type(value).__name__}")


def drive(ir: ProgramIR, backend: ProgramBackend) -> dict[str, str]:
    """Run ``ir`` through ``backend``; return the emitted filename→text mapping."""
    backend.prog_start(ir.name)
    for event in ir.events:
        _dispatch(event, backend)
    return backend.prog_finish()
