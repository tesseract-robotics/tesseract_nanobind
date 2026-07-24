"""The typed event IR: a brand-independent description of a robot program.

``lower()`` produces a ``ProgramIR`` (a name + ordered tuple of frozen events);
a ``ProgramBackend`` consumes it. Events are the *semantic* vocabulary shared by
every controller — smaller than any one vendor language so the walk stays
brand-free. All quantities SI; poses are ``planning.Pose`` (metres, scalar-last).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Union

from tesseract_robotics.planning import Pose

from .external_axes import ExternalAxisValue


class MoveKind(Enum):
    """Interpolation kind (mirrors tesseract MoveInstructionType, minus CIRCULAR)."""

    LINEAR = "LINEAR"
    FREESPACE = "FREESPACE"


@dataclass(frozen=True)
class JointMove:
    """Absolute joint-space move.

    ``joints`` are the six arm axes in radians. ``external_axes`` are the SI
    external-axis values split off the coordinated joint vector by ``lower()``
    when a layout is supplied — empty for a plain (uncoupled) 6-DOF move, in
    which case ``joints`` is the full waypoint vector unchanged.
    """

    joints: tuple[float, ...]
    profile: str
    external_axes: tuple[ExternalAxisValue, ...] = ()


@dataclass(frozen=True)
class CartesianMove:
    """Cartesian move to ``pose`` (metres). ``seed_joints`` (radians) is the
    waypoint IK seed when present (used by backends that derive configuration
    from joints), else ``None``. ``external_axes`` are the SI external-axis
    values for the robtarget's ``eax`` field (empty when uncoupled)."""

    pose: Pose
    kind: MoveKind
    profile: str
    seed_joints: tuple[float, ...] | None
    external_axes: tuple[ExternalAxisValue, ...] = ()


@dataclass(frozen=True)
class Dwell:
    """Pause execution for ``seconds`` (>= 0)."""

    seconds: float


@dataclass(frozen=True)
class WaitDigital:
    """Block until digital signal reaches ``value``.

    ``is_input`` selects input vs output; ``index`` is the IO number; ``timeout``
    is seconds or ``None`` for wait-forever. Backends own signal naming.
    """

    is_input: bool
    index: int
    value: bool
    timeout: float | None


@dataclass(frozen=True)
class SetDigital:
    """Drive a digital output to ``value``.

    ``key is None`` ⇒ unnamed output number ``index`` (backend names it, e.g.
    RAPID ``do<index>``, KRL ``$OUT[index]``). ``key`` set ⇒ named signal; a
    nonzero ``index`` is an array subscript (RAPID ``key{index}``).
    """

    key: str | None
    index: int
    value: bool


@dataclass(frozen=True)
class SetAnalog:
    """Drive an analog output to ``value``. ``key`` semantics as ``SetDigital``."""

    key: str | None
    index: int
    value: float


@dataclass(frozen=True)
class ToolChange:
    """Switch active tool to ``tool_id`` (controller-defined index)."""

    tool_id: int


@dataclass(frozen=True)
class Note:
    """Free-text comment, emitted in the target language's comment syntax."""

    text: str


Event = Union[JointMove, CartesianMove, Dwell, WaitDigital, SetDigital, SetAnalog, ToolChange, Note]


def _event_repr(event: Event) -> str:
    """Stable value-based repr for canonical serialization (poses via matrix).

    ``external_axes`` contributes only when non-empty, so an uncoupled program's
    canonical form (and thus its content digest) is byte-identical to before the
    external-axis field existed.
    """
    if isinstance(event, CartesianMove):
        rows = event.pose.matrix.round(9).tolist()
        ext = f",ext={event.external_axes}" if event.external_axes else ""
        return (
            f"CartesianMove(kind={event.kind.value},profile={event.profile!r},"
            f"seed={event.seed_joints}{ext},pose={rows})"
        )
    if isinstance(event, JointMove) and not event.external_axes:
        # Match the pre-external-axes dataclass repr exactly (default `, ` spacing).
        return f"JointMove(joints={event.joints}, profile={event.profile!r})"
    return repr(event)


@dataclass(frozen=True)
class ProgramIR:
    """A lowered program: a name plus an ordered, immutable tuple of events."""

    name: str
    events: tuple[Event, ...] = field(default_factory=tuple)

    def canonical(self) -> str:
        """Deterministic serialization — the input to the content digest."""
        body = ";".join(_event_repr(e) for e in self.events)
        return f"ProgramIR(name={self.name!r},events=[{body}])"
