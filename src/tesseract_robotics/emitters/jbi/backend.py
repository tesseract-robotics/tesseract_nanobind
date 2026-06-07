"""``JbiBackend`` — consume the core IR, emit Yaskawa Motoman INFORM (``.jbi``).

The structural twin of ``emitters.ls.backend.LsBackend``: a ``.jbi`` has two
flat sections — a ``//POS`` position table and a ``//INST`` instruction body —
and the table is written *first* yet only discovered while walking the
instructions. So, exactly like LS:

1. instruction lines are emitted **eagerly** through the ``jbi_writer`` DSL into
   the ``JbiWriter`` buffer as the IR streams in (``Movj`` / ``Movl`` / …);
2. each move's ``C#####`` position tuple is accumulated into ``self._positions``.

``prog_finish`` then assembles the file from a template — ``/JOB`` → ``//NAME``
→ ``//POS`` (header + table) → ``//INST`` (header + ``NOP``) → the buffered
instruction body → ``END`` — with a FIXED date so identical inputs produce
identical bytes. No deferred closures: the buffer already holds the instructions
in order, and ``//POS`` simply prints ahead of it from the accumulated list.

Joint moves and FREESPACE Cartesian → ``MOVJ`` (joint speed percent); LINEAR →
``MOVL`` (mm/s). Because v1 writes the table in ``///POSTYPE PULSE`` as joint
**degrees** (no encoder-pulse conversion — see ``jbi.targets``), a Cartesian
move's row needs joint values: ``seed_joints`` supplies them, and a Cartesian
move without a seed raises ``MissingSeedError`` (never guessed).
"""

from __future__ import annotations

from collections.abc import Mapping

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
from .jbi_writer import Comment, JbiWriter, Movj, Movl, SetOut, TimerInst, WaitIn, pos_var_name
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
#: ``///NPOS`` trailing fixed-zero fields (aux-axis position counts, unused in v1).
_NPOS_TRAILING = "0,0,0,0,0"


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
    """Stateful JBI backend; eager ``//INST`` DSL buffer + ``//POS`` list, assembled at finish."""

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
        self._move_count = 0
        if self._identity is not None:
            for line in self._identity.header_lines():
                Comment(line)

    def move_joint(self, m: JointMove) -> None:
        self._move_count += 1
        percent = self._profile(m.profile).joint_speed_percent
        Movj(self._add_position(pulse_pos(m.joints)), percent)

    def move_cartesian(self, m: CartesianMove) -> None:
        self._move_count += 1
        profile = self._profile(m.profile)
        if m.seed_joints is None:
            raise MissingSeedError(self._move_count)
        index = self._add_position(pulse_pos(m.seed_joints))
        if m.kind is MoveKind.LINEAR:
            Movl(index, profile.linear_speed_mms)
        else:
            Movj(index, profile.joint_speed_percent)

    def dwell(self, e: Dwell) -> None:
        TimerInst(e.seconds)

    def wait_digital(self, e: WaitDigital) -> None:
        WaitIn(e.index, e.value, is_input=e.is_input)

    def set_digital(self, e: SetDigital) -> None:
        SetOut(e.index, e.value)

    def set_analog(self, e: SetAnalog) -> None:
        Comment(f"analog out #{e.index} = {e.value:g} (unsupported)")

    def tool_change(self, e: ToolChange) -> None:
        Comment(f"tool change to tool id {e.tool_id}")

    def note(self, e: Note) -> None:
        Comment(e.text)

    def prog_finish(self) -> dict[str, str]:
        return {f"{self._name}.jbi": "\n".join(self._assemble()) + "\n"}

    # File assembly -----------------------------------------------------
    def _assemble(self) -> list[str]:
        """Build the full ``.jbi`` line list: ``/JOB`` + ``//POS`` table + ``//INST`` body."""
        lines = [
            "/JOB",
            f"//NAME {self._name}",
            "//POS",
            f"///NPOS {len(self._positions)},{_NPOS_TRAILING}",
            f"///TOOL {self._tool}",
            "///POSTYPE PULSE",
            "///PULSE",
        ]
        lines.extend(
            f"{pos_var_name(i)}={tuple_text}" for i, tuple_text in enumerate(self._positions)
        )
        lines.extend(
            [
                "//INST",
                f"///DATE {_FIXED_DATE}",
                f"///COMM {_COMM}",
                f"///ATTR {_ATTR}",
                f"///GROUP1 {_GROUP1}",
                "NOP",
            ]
        )
        inst_body = self._jbi.getvalue()
        if inst_body:
            lines.extend(inst_body.split("\n"))
        lines.append("END")
        return lines
