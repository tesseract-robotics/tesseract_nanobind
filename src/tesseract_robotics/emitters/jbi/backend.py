"""``JbiBackend`` — consume the core IR, emit Yaskawa Motoman INFORM (``.jbi``).

The twin of ``emitters.krl.backend.KrlBackend``, but a ``.jbi`` has **two**
sections — a ``//POS`` position table and a ``//INST`` instruction body — and
the position table is written *first* yet only discovered while walking the
instructions. So this backend accumulates two parallel buffers as the IR
streams in:

- ``_positions`` — the ``C#####`` position tuples (one per move), and
- ``_lines`` — deferred instruction emitters (closures), each capturing the
  position index it references.

``prog_finish`` then drives the ``jbi_writer`` DSL once, in file order:
``/JOB`` → ``//NAME`` → ``//POS`` (header + table) → ``//INST`` (header + ``NOP``
+ identity comments + the deferred instruction lines + ``END``).

Joint moves and FREESPACE Cartesian → ``MOVJ`` (joint speed percent); LINEAR →
``MOVL`` (mm/s). Because v1 writes the position table in ``///POSTYPE PULSE`` as
joint **degrees** (no encoder-pulse conversion — see ``jbi.targets``), a
Cartesian move's row needs joint values: ``seed_joints`` supplies them, and a
Cartesian move without a seed raises ``MissingSeedError`` (never guessed).
"""

from __future__ import annotations

from collections.abc import Callable, Mapping

from ..core.backend import ProgramBackend
from ..core.errors import EmitterError, MissingProfileError
from ..core.events import (
    CartesianMove,
    Dwell,
    JointMove,
    MoveKind,
    Note,
    SetAnalog,
    SetDigital,
    ToolChange,
    WaitDigital,
)
from ..core.identity import EmitIdentity
from ..core.naming import safe_identifier
from .jbi_writer import (
    Attr,
    Comm,
    Comment,
    Date,
    End,
    Group,
    InstHeader,
    JbiWriter,
    Job,
    Movj,
    Movl,
    Name,
    Nop,
    NPos,
    PosHeader,
    PosType,
    PosVar,
    SetOut,
    TimerInst,
    Tool,
    WaitIn,
)
from .profile import JbiProfile
from .targets import pulse_pos

#: JBI job-name length cap (controller-side names stay short).
_JBI_NAME_MAX = 32
#: FIXED job date — never a live timestamp, so identical inputs → identical bytes.
_FIXED_DATE = "2000/01/01 00:00"
#: ``///COMM`` header line.
_COMM = "tesseract emitters"
#: ``///ATTR`` header line (standard read/write job attributes).
_ATTR = "SC,RW"
#: ``///GROUP1`` control group (single 6-axis robot in v1).
_GROUP1 = "RB1"


class MissingSeedError(EmitterError):
    """A Cartesian move has no seed joints, so its PULSE position row is unknown.

    ``///POSTYPE PULSE`` rows are joint angles; a Cartesian target carries only a
    pose. Without ``seed_joints`` the joint values cannot be derived here (that
    needs IK), so emission fails loudly rather than guessing a configuration.
    """

    def __init__(self, move_index: int) -> None:
        self.move_index = move_index
        super().__init__(
            f"JBI: Cartesian move #{move_index} has no seed joints; PULSE positions are "
            f"joint angles, so seed_joints are required (or use a JointMove)"
        )


class JbiBackend(ProgramBackend):
    """Stateful JBI backend; buffers positions + instructions, assembles at finish."""

    def __init__(
        self,
        *,
        profiles: Mapping[str, JbiProfile],
        program_name: str,
        identity: EmitIdentity | None = None,
    ) -> None:
        self._profiles = profiles
        self._name = safe_identifier(program_name, brand="JBI", max_len=_JBI_NAME_MAX)
        self._identity = identity
        self._jbi = JbiWriter()
        self._tool = self._resolve_tool(profiles)
        self._positions: list[str] = []
        self._lines: list[Callable[[], None]] = []
        self._move_count = 0

    @staticmethod
    def _resolve_tool(profiles: Mapping[str, JbiProfile]) -> int:
        """The ``///TOOL`` number for the position header.

        All profiles in one emit share a tool number; if they disagree the first
        (sorted) profile's tool wins and v1 documents the single-tool assumption.
        Empty profiles ⇒ tool 0.
        """
        for _, profile in sorted(profiles.items()):
            return profile.tool
        return 0

    def _profile(self, name: str) -> JbiProfile:
        try:
            return self._profiles[name]
        except KeyError:
            raise MissingProfileError(name, list(self._profiles.keys())) from None

    def _add_position(self, tuple_text: str) -> int:
        """Append a position tuple to the ``//POS`` table; return its index."""
        index = len(self._positions)
        self._positions.append(tuple_text)
        return index

    # ProgramBackend protocol -------------------------------------------
    def prog_start(self) -> None:
        self._jbi.clear()
        self._positions = []
        self._lines = []
        self._move_count = 0
        if self._identity is not None:
            for line in self._identity.header_lines():
                self._lines.append(lambda line=line: Comment(line))

    def move_joint(self, m: JointMove) -> None:
        self._move_count += 1
        percent = self._profile(m.profile).joint_speed_percent
        index = self._add_position(pulse_pos(m.joints))
        self._lines.append(lambda: Movj(index, percent))

    def move_cartesian(self, m: CartesianMove) -> None:
        self._move_count += 1
        profile = self._profile(m.profile)
        if m.seed_joints is None:
            raise MissingSeedError(self._move_count)
        index = self._add_position(pulse_pos(m.seed_joints))
        if m.kind is MoveKind.LINEAR:
            speed = profile.linear_speed_mms
            self._lines.append(lambda: Movl(index, speed))
        else:
            percent = profile.joint_speed_percent
            self._lines.append(lambda: Movj(index, percent))

    def dwell(self, e: Dwell) -> None:
        self._lines.append(lambda: TimerInst(e.seconds))

    def wait_digital(self, e: WaitDigital) -> None:
        self._lines.append(lambda: WaitIn(e.index, e.value, is_input=e.is_input))

    def set_digital(self, e: SetDigital) -> None:
        self._lines.append(lambda: SetOut(e.index, e.value))

    def set_analog(self, e: SetAnalog) -> None:
        self._lines.append(lambda: Comment(f"analog out #{e.index} = {e.value:g} (unsupported)"))

    def tool_change(self, e: ToolChange) -> None:
        self._lines.append(lambda: Comment(f"tool change to tool id {e.tool_id}"))

    def note(self, e: Note) -> None:
        self._lines.append(lambda: Comment(e.text))

    def prog_finish(self) -> dict[str, str]:
        Job()
        Name(self._name)

        PosHeader()
        NPos(len(self._positions))
        Tool(self._tool)
        PosType()
        for index, tuple_text in enumerate(self._positions):
            PosVar(index, tuple_text)

        InstHeader()
        Date(_FIXED_DATE)
        Comm(_COMM)
        Attr(_ATTR)
        Group(_GROUP1)
        Nop()
        for emit_line in self._lines:
            emit_line()
        End()

        return {f"{self._name}.jbi": self._jbi.getvalue() + "\n"}
