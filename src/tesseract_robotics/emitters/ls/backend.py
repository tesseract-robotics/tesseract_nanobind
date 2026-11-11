"""`LsBackend`: consume the core IR, emit Fanuc LS (TP ASCII) `.ls` text.

A Fanuc program is a logic section `/MN` (numbered instruction lines) and a
`/POS` position table. As the backend walks the IR it

1. drives the `ls_writer` DSL for the `/MN` body: every move emits one numbered
   instruction line referencing a position by index (`J P[i]` / `L P[i]`);
2. accumulates a `PosEntry` per move.

`prog_finish` assembles `/PROG` + `/ATTR` + `/MN` + `/POS` + `/END` into one
`.ls` file. The `/ATTR` block uses a fixed date so identical inputs give
identical bytes.

Frames: each move's tool and user frame come from the frame binding; the
backend emits `UTOOL_NUM=` / `UFRAME_NUM=` whenever they change, and every
`/POS` entry carries the same `UF` / `UT`, so the active frames always match
the positions. Joint moves and FREESPACE Cartesian moves become `J`
(joint-interpolated, percent speed); LINEAR Cartesian moves become `L` (mm/sec).
Cartesian entries carry an exact `CONFIG`: placement letters from the caller's
`FanucConfigResolver` and turn numbers computed from the seed joints; without
either the move raises `MissingConfigurationError`.
"""

from __future__ import annotations

from collections.abc import Mapping
from contextlib import AbstractContextManager

from ..core.backend import ProgramBackend
from ..core.errors import (
    MissingConfigurationError,
    MissingProfileError,
    UnsupportedInstructionError,
)
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
from ..core.frames import FrameBinding, MoveFrames
from ..core.identity import EmitIdentity
from .ls_writer import (
    CommentLine,
    InstructionLine,
    JointMoveLine,
    LinearMoveLine,
    LsWriter,
    PosEntry,
)
from .profile import LsProfile
from .syntax import BRAND, LS_IDENTIFIERS, FanucConfigResolver
from .targets import cartesian_pos_block, config_string, joint_pos_block

#: Fixed `/ATTR` date/time so emission is deterministic (no wall-clock in body).
_FIXED_DATE = "DATE 31-12-14  TIME 12:00:00"
#: `/ATTR` owner/editor identity Fanuc writes for hand-edited programs.
_OWNER = "MNEDITOR"
#: Free-text `/ATTR` comment shown in the teach pendant program list.
_COMMENT = "tesseract emitters"


def _numbered_only(key: str | None, what: str) -> None:
    if key is not None:
        raise UnsupportedInstructionError(
            f"LS: {what} {key!r} is a named signal; TP IO is addressed by number "
            "(DO[i], AO[i]), so a key cannot be mapped without guessing"
        )


class LsBackend(ProgramBackend):
    """Stateful Fanuc LS backend; drives the `/MN` DSL and a `/POS` table."""

    def __init__(
        self,
        *,
        profiles: Mapping[str, LsProfile],
        frames: FrameBinding[int, int],
        config: FanucConfigResolver | None,
        program_name: str,
        identity: EmitIdentity,
    ) -> None:
        self._profiles = profiles
        self._frames = frames
        self._config = config
        self._name = LS_IDENTIFIERS.validate(program_name.upper())
        self._identity = identity
        self._ls = LsWriter()
        #: 1-based `/MN` instruction line counter (identity comments consume lines too).
        self._line_no = 0
        #: 1-based `P[i]` position index, shared across joint and Cartesian moves.
        self._pos_index = 0
        self._positions: list[PosEntry] = []
        self._move_index = 0
        self._uf: int | None = None
        self._ut: int | None = None

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

    def _select_tool(self, frames: MoveFrames) -> int:
        frames.require_no_offset(BRAND)
        ut = self._frames.tool(frames, BRAND)
        if ut != self._ut:
            InstructionLine(self._next_line(), f"UTOOL_NUM={ut}")
            self._ut = ut
        return ut

    def _select_user_frame(self, frames: MoveFrames) -> int:
        uf = self._frames.work(frames, BRAND)
        if uf != self._uf:
            InstructionLine(self._next_line(), f"UFRAME_NUM={uf}")
            self._uf = uf
        return uf

    # ProgramBackend protocol -------------------------------------------
    def session(self) -> AbstractContextManager[object]:
        return self._ls

    def prog_start(self) -> None:
        for line in self._identity.header_lines():
            CommentLine(self._next_line(), line)

    def move_joint(self, m: JointMove) -> None:
        self._move_index += 1
        profile = self._profile(m.profile)
        block_frames = (self._select_user_frame(m.frames), self._select_tool(m.frames))
        index = self._next_pos()
        block = joint_pos_block(index, m.joints, uf=block_frames[0], ut=block_frames[1])
        JointMoveLine(
            self._next_line(),
            index=index,
            speed_percent=profile.joint_speed_percent,
            termination=profile.termination,
        )
        self._positions.append(PosEntry(index, block))

    def move_cartesian(self, m: CartesianMove) -> None:
        self._move_index += 1
        profile = self._profile(m.profile)
        config = self._config_string(m)
        frames = m.target.frames
        uf = self._select_user_frame(frames)
        ut = self._select_tool(frames)
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
        block = cartesian_pos_block(index, m.target.pose, uf=uf, ut=ut, config=config)
        self._positions.append(PosEntry(index, block))

    def _config_string(self, m: CartesianMove) -> str:
        if m.seed_joints is None:
            raise MissingConfigurationError(
                self._move_index, BRAND, "the waypoint has no seed joints for the turn numbers"
            )
        if self._config is None:
            raise MissingConfigurationError(
                self._move_index,
                BRAND,
                "the FLIP/UP/FRONT letters need emit_ls(config=...) for this robot model",
            )
        return config_string(self._config(m.seed_joints), m.seed_joints)

    def dwell(self, e: Dwell) -> None:
        # Fanuc `WAIT <s>(sec)`: a numbered `/MN` line, no position.
        InstructionLine(self._next_line(), f"WAIT {e.seconds:g}(sec)")

    def wait_digital(self, e: WaitDigital) -> None:
        # `WAIT DI[i]=ON` / `WAIT DO[i]=OFF`: DI for inputs, DO for outputs.
        signal = "DI" if e.is_input else "DO"
        state = "ON" if e.value else "OFF"
        InstructionLine(self._next_line(), f"WAIT {signal}[{e.index}]={state}")

    def set_digital(self, e: SetDigital) -> None:
        _numbered_only(e.key, "digital output")
        state = "ON" if e.value else "OFF"
        InstructionLine(self._next_line(), f"DO[{e.index}]={state}")

    def set_analog(self, e: SetAnalog) -> None:
        _numbered_only(e.key, "analog output")
        InstructionLine(self._next_line(), f"AO[{e.index}]=({e.value:g})")

    def tool_change(self, e: ToolChange) -> None:
        raise UnsupportedInstructionError(
            f"LS: SetTool({e.tool_id}) conflicts with frame-bound tools; UTOOL_NUM follows "
            "each move's tcp_frame through the frame binding, and every /POS entry's UT must "
            "match it"
        )

    def note(self, e: Note) -> None:
        CommentLine(self._next_line(), e.text)

    def prog_finish(self) -> dict[str, str]:
        text = "\n".join(self._assemble()) + "\n"
        return {f"{self._name}.ls": text}

    # File assembly -----------------------------------------------------
    def _assemble(self) -> list[str]:
        """Build the full `.ls` line list: `/PROG` + `/ATTR` + `/MN` + `/POS` + `/END`."""
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
        """The minimal, deterministic `/ATTR` block (fixed date, zeroed sizes)."""
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
