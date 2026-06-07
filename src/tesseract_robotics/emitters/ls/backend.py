"""``LsBackend`` — consume the core IR, emit Fanuc LS (TP ASCII) ``.ls`` text.

The twin of ``emitters.krl.backend.KrlBackend`` with one structural twist: a Fanuc
program is split into a logic section ``/MN`` (numbered instruction lines) and a
``/POS`` position table. So the backend does two things as it walks the IR:

1. drives the ``ls_writer`` DSL for the ``/MN`` body — every move emits one numbered
   instruction line referencing a position by index (``J P[i]`` / ``L P[i]``);
2. accumulates a :class:`PosEntry` per move into ``self._positions``.

``prog_finish`` then assembles ``/PROG`` + ``/ATTR`` + ``/MN`` + ``/POS`` + ``/END``
into a single ``.ls`` file. The ``/ATTR`` block is minimal and uses a **fixed** date
so byte-identical inputs produce byte-identical output (goldens diff meaningfully).

Joint moves and FREESPACE Cartesian moves → ``J`` (joint-interpolated, percentage
speed); LINEAR Cartesian moves → ``L`` (linear, mm/sec speed). Joint targets get a
joint-style ``/POS`` entry (J1..J6 deg); Cartesian targets get a Cartesian ``/POS``
entry (``CONFIG`` + X/Y/Z mm + W/P/R deg). v1 fixes the arm configuration string, so
Cartesian moves need no IK-derived configuration.
"""

from __future__ import annotations

from collections.abc import Mapping

from ..core.backend import ProgramBackend
from ..core.errors import MissingProfileError
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
from .ls_writer import (
    CommentLine,
    InstructionLine,
    JointMoveLine,
    LinearMoveLine,
    LsWriter,
    PosEntry,
)
from .profile import LsProfile
from .targets import cartesian_pos_block, joint_pos_block

#: Fanuc TP program-name length cap (uppercased, alnum + underscore).
_LS_NAME_MAX = 36
#: Fixed ``/ATTR`` date/time so emission is deterministic (no wall-clock in body).
_FIXED_DATE = "DATE 31-12-14  TIME 12:00:00"
#: ``/ATTR`` owner/editor identity Fanuc writes for hand-edited programs.
_OWNER = "MNEDITOR"
#: Free-text ``/ATTR`` comment shown in the teach pendant program list.
_COMMENT = "tesseract emitters"


class LsBackend(ProgramBackend):
    """Stateful Fanuc LS backend; drives the ``/MN`` DSL and a ``/POS`` table.

    Clears the shared ``LsWriter`` buffer on ``prog_start`` and rebuilds the
    position table, so a single backend instance is safe to reuse across emits.
    """

    def __init__(
        self,
        *,
        profiles: Mapping[str, LsProfile],
        program_name: str,
        identity: EmitIdentity | None = None,
    ) -> None:
        self._profiles = profiles
        self._name = safe_identifier(program_name, brand="LS", max_len=_LS_NAME_MAX).upper()
        self._identity = identity
        self._ls = LsWriter()
        #: 1-based ``/MN`` instruction line counter (identity comments consume lines too).
        self._line_no = 0
        #: 1-based ``P[i]`` position index, shared across joint + Cartesian moves.
        self._pos_index = 0
        self._positions: list[PosEntry] = []

    def _profile(self, name: str) -> LsProfile:
        try:
            return self._profiles[name]
        except KeyError:
            raise MissingProfileError(name, list(self._profiles.keys())) from None

    def _next_line(self) -> int:
        self._line_no += 1
        return self._line_no

    def _next_pos(self) -> int:
        self._pos_index += 1
        return self._pos_index

    # ProgramBackend protocol -------------------------------------------
    def prog_start(self) -> None:
        self._ls.clear()
        self._line_no = 0
        self._pos_index = 0
        self._positions = []
        if self._identity is not None:
            for line in self._identity.header_lines():
                CommentLine(self._next_line(), line)

    def move_joint(self, m: JointMove) -> None:
        profile = self._profile(m.profile)
        index = self._next_pos()
        JointMoveLine(
            self._next_line(),
            index=index,
            speed_percent=profile.joint_speed_percent,
            termination=profile.termination,
        )
        self._positions.append(PosEntry(index, joint_pos_block(index, m.joints)))

    def move_cartesian(self, m: CartesianMove) -> None:
        profile = self._profile(m.profile)
        index = self._next_pos()
        if m.kind is MoveKind.LINEAR:
            LinearMoveLine(
                self._next_line(),
                index=index,
                speed_mms=profile.linear_speed_mms,
                termination=profile.termination,
            )
        else:
            JointMoveLine(
                self._next_line(),
                index=index,
                speed_percent=profile.joint_speed_percent,
                termination=profile.termination,
            )
        self._positions.append(PosEntry(index, cartesian_pos_block(index, m.pose)))

    def dwell(self, e: Dwell) -> None:
        # Fanuc ``WAIT <s>(sec)`` — a numbered ``/MN`` line, no position.
        InstructionLine(self._next_line(), f"WAIT {e.seconds:g}(sec)")

    def wait_digital(self, e: WaitDigital) -> None:
        # ``WAIT DI[i]=ON`` / ``WAIT DO[i]=OFF`` — DI for inputs, DO for outputs.
        signal = "DI" if e.is_input else "DO"
        state = "ON" if e.value else "OFF"
        InstructionLine(self._next_line(), f"WAIT {signal}[{e.index}]={state}")

    def set_digital(self, e: SetDigital) -> None:
        state = "ON" if e.value else "OFF"
        InstructionLine(self._next_line(), f"DO[{e.index}]={state}")

    def set_analog(self, e: SetAnalog) -> None:
        InstructionLine(self._next_line(), f"AO[{e.index}]=({e.value:g})")

    def tool_change(self, e: ToolChange) -> None:
        # No standard TP tool-change opcode; record intent as a remark + UTOOL_NUM.
        CommentLine(self._next_line(), f"tool change to tool id {e.tool_id}")
        InstructionLine(self._next_line(), f"UTOOL_NUM={e.tool_id}")

    def note(self, e: Note) -> None:
        CommentLine(self._next_line(), e.text)

    def prog_finish(self) -> dict[str, str]:
        text = "\n".join(self._assemble()) + "\n"
        return {f"{self._name}.ls": text}

    # File assembly -----------------------------------------------------
    def _assemble(self) -> list[str]:
        """Build the full ``.ls`` line list: ``/PROG`` + ``/ATTR`` + ``/MN`` + ``/POS`` + ``/END``."""
        lines = [f"/PROG  {self._name}"]
        lines.extend(self._attr_section())
        lines.append("/MN")
        mn_body = self._ls.getvalue()
        if mn_body:
            lines.extend(mn_body.split("\n"))
        lines.append("/POS")
        for entry in self._positions:
            lines.extend(entry.block.split("\n"))
        lines.append("/END")
        return lines

    def _attr_section(self) -> list[str]:
        """The minimal, deterministic ``/ATTR`` block (fixed date, zeroed sizes)."""
        return [
            "/ATTR",
            f"OWNER\t\t= {_OWNER};",
            f'COMMENT\t\t= "{_COMMENT}";',
            "PROG_SIZE\t= 0;",
            f"CREATE\t\t= {_FIXED_DATE};",
            f"MODIFIED\t= {_FIXED_DATE};",
            f"FILE_NAME\t= {self._name};",
            "VERSION\t\t= 0;",
            f"LINE_COUNT\t= {self._line_no};",
            "MEMORY_SIZE\t= 0;",
            "PROTECT\t\t= READ_WRITE;",
            "TCD:  STACK_SIZE\t= 0,",
            "      TASK_PRIORITY\t= 50,",
            "      TIME_SLICE\t= 0,",
            "      BUSY_LAMP_OFF\t= 0,",
            "      ABORT_REQUEST\t= 0,",
            "      PAUSE_REQUEST\t= 0;",
            "DEFAULT_GROUP\t= 1,*,*,*,*,*,*;",
            "CONTROL_CODE\t= 00000000 00000000;",
        ]
