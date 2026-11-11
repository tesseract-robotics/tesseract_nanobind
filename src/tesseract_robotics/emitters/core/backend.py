"""The backend contract and the IR-to-backend driver.

`ProgramBackend` is the abstract contract each robot dialect implements: RAPID,
KRL, LS, JBI and URScript are its subclasses. The IR events are brand-neutral
data; a backend is the dialect that renders them. One `@abstractmethod` per
event type means a new dialect (or a newly added event) fails at instantiation
until every case is handled.

`drive()` is the visitor: it opens the backend's writer session, walks a
`ProgramIR` and routes each event to the matching backend method, exhaustively
(a new event type without a branch trips `_assert_never`, and the type checker
flags the unhandled case). The session closes even when a handler raises, so a
failed emit never leaves its writer active.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from contextlib import AbstractContextManager

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
    """Abstract dialect contract: one concrete subclass per robot language.

    A backend receives its program name, profiles and frame binding at
    construction; `session` returns the context that activates its writer;
    `prog_start` opens the program and `prog_finish` returns the emitted
    filename-to-text map.
    """

    @abstractmethod
    def session(self) -> AbstractContextManager[object]:
        """Context that makes this backend's writer the active one while it is open."""

    @abstractmethod
    def prog_start(self) -> None: ...

    @abstractmethod
    def move_joint(self, m: JointMove) -> None: ...

    @abstractmethod
    def move_cartesian(self, m: CartesianMove) -> None: ...

    @abstractmethod
    def dwell(self, e: Dwell) -> None: ...

    @abstractmethod
    def wait_digital(self, e: WaitDigital) -> None: ...

    @abstractmethod
    def set_digital(self, e: SetDigital) -> None: ...

    @abstractmethod
    def set_analog(self, e: SetAnalog) -> None: ...

    @abstractmethod
    def tool_change(self, e: ToolChange) -> None: ...

    @abstractmethod
    def note(self, e: Note) -> None: ...

    @abstractmethod
    def prog_finish(self) -> dict[str, str]: ...


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
    """Run `ir` through `backend`; return the emitted filename-to-text mapping."""
    with backend.session():
        backend.prog_start()
        for event in ir.events:
            _dispatch(event, backend)
        return backend.prog_finish()
