"""The typed event IR: a brand-independent description of a robot program.

`lower()` produces a `ProgramIR` (a name and an ordered tuple of frozen events);
a `ProgramBackend` consumes it. Events are the semantic vocabulary shared by
every controller, smaller than any one vendor language so the walk stays
brand-free. All quantities are SI. Cartesian targets are `FramedPose`s, so the
frames a pose is expressed in travel with it in the type.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

from .errors import InvalidEventError
from .external_axes import ExternalAxisValue
from .frames import FramedPose, MoveFrames


class MoveKind(Enum):
    """Interpolation kind (mirrors tesseract `MoveInstructionType`, minus CIRCULAR)."""

    LINEAR = "LINEAR"
    FREESPACE = "FREESPACE"


@dataclass(frozen=True)
class JointMove:
    """Absolute joint-space move.

    Attributes:
        joints: The arm axes in radians. Without an external-axis layout this is
            the full waypoint vector; backends that emit only six axes raise
            `ExternalAxisError` on more.
        profile: tesseract profile name.
        frames: The move's frames (the tool is still selected for joint moves).
        external_axes: SI external-axis values split off by `lower()` when a
            layout is supplied; empty otherwise.
    """

    joints: tuple[float, ...]
    profile: str
    frames: MoveFrames
    external_axes: tuple[ExternalAxisValue, ...] = ()


@dataclass(frozen=True)
class CartesianMove:
    """Cartesian move to `target`.

    Attributes:
        target: TCP pose in the working frame, tagged with its frames.
        kind: LINEAR or FREESPACE interpolation.
        profile: tesseract profile name.
        seed_joints: The waypoint's seed joints (radians, arm axes only when a
            layout split them), or `None` when the waypoint carries none. Backends
            derive the controller's configuration bits from it; it is never
            emitted as the target.
        external_axes: SI external-axis values for the target's external-axis
            field (empty when uncoupled).
    """

    target: FramedPose
    kind: MoveKind
    profile: str
    seed_joints: tuple[float, ...] | None
    external_axes: tuple[ExternalAxisValue, ...] = ()


@dataclass(frozen=True)
class Dwell:
    """Pause execution for `seconds`. Build via `Dwell.build` (validates)."""

    seconds: float

    @classmethod
    def build(cls, seconds: float) -> Dwell:
        """Validate and construct.

        Raises:
            InvalidEventError: `seconds` is NaN, infinite or negative.
        """
        if not math.isfinite(seconds) or seconds < 0.0:
            raise InvalidEventError(f"Dwell.seconds must be finite and >= 0; got {seconds}")
        return cls(seconds)


@dataclass(frozen=True)
class WaitDigital:
    """Block until a digital signal reaches `value`.

    `is_input` selects input vs output; `index` is the IO number; `timeout`
    is seconds or `None` for wait-forever. Backends own signal naming.
    """

    is_input: bool
    index: int
    value: bool
    timeout: float | None


@dataclass(frozen=True)
class SetDigital:
    """Drive a digital output to `value`.

    `key is None` means unnamed output number `index` (the backend names it,
    e.g. RAPID `do<index>`, KRL `$OUT[index]`). `key` set means a named signal;
    a nonzero `index` is then an array subscript (RAPID `key{index}`).
    """

    key: str | None
    index: int
    value: bool


@dataclass(frozen=True)
class SetAnalog:
    """Drive an analog output to `value`. `key` semantics as `SetDigital`."""

    key: str | None
    index: int
    value: float


@dataclass(frozen=True)
class ToolChange:
    """Switch the active tool to `tool_id` (controller-defined index)."""

    tool_id: int


@dataclass(frozen=True)
class Note:
    """Free-text comment, emitted in the target language's comment syntax."""

    text: str


type Event = (
    JointMove | CartesianMove | Dwell | WaitDigital | SetDigital | SetAnalog | ToolChange | Note
)


#: Decimals of every float in the canonical form: far below any emitted display
#: precision (see `core.precision`), and coarse enough that last-bit differences
#: between platform math libraries (IK seeds, composed poses) cannot change it.
_CANONICAL_DECIMALS = 9


def _rounded(values: tuple[float, ...] | None) -> tuple[float, ...] | None:
    return None if values is None else tuple(round(v, _CANONICAL_DECIMALS) for v in values)


def _event_repr(event: Event) -> str:
    """Stable value-based text for canonical serialization (poses via matrix)."""
    if isinstance(event, CartesianMove):
        rows = event.target.pose.matrix.round(_CANONICAL_DECIMALS).tolist()
        return (
            f"CartesianMove(kind={event.kind.value},profile={event.profile!r},"
            f"frames={event.target.frames.canonical()},seed={_rounded(event.seed_joints)},"
            f"ext={event.external_axes},pose={rows})"
        )
    if isinstance(event, JointMove):
        return (
            f"JointMove(joints={_rounded(event.joints)},profile={event.profile!r},"
            f"frames={event.frames.canonical()},ext={event.external_axes})"
        )
    return repr(event)


@dataclass(frozen=True)
class ProgramIR:
    """A lowered program: a name plus an ordered, immutable tuple of events."""

    name: str
    events: tuple[Event, ...] = field(default_factory=tuple)

    def canonical(self) -> str:
        """Deterministic serialization: the input to the content digest."""
        body = ";".join(_event_repr(e) for e in self.events)
        return f"ProgramIR(name={self.name!r},events=[{body}])"
