"""The backend contract and the IR-to-backend driver.

``ProgramBackend`` is the abstract contract that **each robot dialect concretely
implements** — RAPID, KRL, LS, JBI, URScript are its subclasses. The IR events
are brand-neutral *data*; a backend is the dialect that renders them. One
``@abstractmethod`` per event type means a new dialect (or a newly added event)
fails at class-definition until every case is handled — the contract is enforced
nominally, not merely hoped-for structurally.

``drive()`` is the visitor: it walks a ``ProgramIR`` and routes each event to the
matching backend method, exhaustively (a new event type without a branch trips
``_assert_never``, and the type checker flags the unhandled case). Together:
the ABC says *what* every dialect must handle; ``drive`` says *how* events reach
those handlers — and the IR stays a single brand-neutral, content-addressable
artifact that one lowering can feed to every backend.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

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


class ProgramBackend(ABC):
    """Abstract dialect contract — one concrete subclass per robot language.

    Inline provenance documents the attic ``RobotPost`` name each descends from.
    A backend receives its program/module name(s) at construction; ``prog_start``
    opens the program and ``prog_finish`` returns the emitted filename→text map.
    """

    @abstractmethod
    def prog_start(self) -> None:  # attic: RobotPost.ProgStart
        ...

    @abstractmethod
    def move_joint(self, m: JointMove) -> None:  # attic: RobotPost.MoveJ (pose=None)
        ...

    @abstractmethod
    def move_cartesian(self, m: CartesianMove) -> None:  # attic: RobotPost.MoveJ/MoveL
        ...

    @abstractmethod
    def dwell(self, e: Dwell) -> None:  # attic: RobotPost.Pause
        ...

    @abstractmethod
    def wait_digital(self, e: WaitDigital) -> None:  # attic: RobotPost.waitDI
        ...

    @abstractmethod
    def set_digital(self, e: SetDigital) -> None:  # attic: RobotPost.setDO
        ...

    @abstractmethod
    def set_analog(self, e: SetAnalog) -> None:  # attic: RobotPost.setAO
        ...

    @abstractmethod
    def tool_change(self, e: ToolChange) -> None:  # attic: RobotPost.setTool
        ...

    @abstractmethod
    def note(self, e: Note) -> None:  # attic: RobotPost.RunMessage(iscomment=True)
        ...

    @abstractmethod
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
    backend.prog_start()
    for event in ir.events:
        _dispatch(event, backend)
    return backend.prog_finish()
