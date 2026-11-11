"""`JbiBackend`: consume the core IR, emit Yaskawa Motoman INFORM (`.jbi`).

A `.jbi` has two flat sections, a `//POS` position table and a `//INST`
instruction body, and the table is written first yet only discovered while
walking the instructions. So:

1. instruction lines are emitted eagerly through the `jbi_writer` DSL into the
   backend's own `JbiWriter` as the IR streams in (`Movj` / `Movl` / ...);
2. each move's `C#####` pulse row is accumulated into `self._positions`.

`prog_finish` assembles `/JOB`, `//NAME`, `//POS` (header and table),
`//INST` (header and `NOP`), the buffered body and `END`, with a fixed date so
identical inputs give identical bytes.

Positions are joint pulses (`///POSTYPE PULSE`), converted exactly through the
caller's `JbiPulseCalibration`. Joint moves become `MOVJ`. Cartesian moves raise
`UnsupportedInstructionError`: a Cartesian JBI position needs `///RCONF`
configuration flags and a user/robot coordinate frame, and a pulse row from the
waypoint's IK seed would move to the seed, not to the planned pose.

The position table carries one `///TOOL`, taken from the moves' `tcp_frame`
through the frame binding; moves that bind to different tools raise
`ToolConflictError`.

The content-addressed identity is not written into the file: its lines exceed
the 32-character INFORM comment limit. It travels on `EmittedProgram.identity`.
"""

from __future__ import annotations

from collections.abc import Mapping
from contextlib import AbstractContextManager

from ..core.backend import ProgramBackend
from ..core.errors import MissingProfileError, ToolConflictError, UnsupportedInstructionError
from ..core.events import (
    CartesianMove,
    Dwell,
    JointMove,
    Note,
    SetAnalog,
    SetDigital,
    ToolChange,
    WaitDigital,
)
from ..core.frames import FrameBinding
from .jbi_writer import (
    AnalogOut,
    Comment,
    JbiWriter,
    Movj,
    SetOut,
    TimerInst,
    WaitIn,
    pos_var_name,
)
from .profile import JbiProfile
from .syntax import ANALOG_MAX_V, ANALOG_PORTS, BRAND, JBI_IDENTIFIERS, JbiPulseCalibration
from .targets import pulse_pos

#: FIXED job date: never a live timestamp, so identical inputs give identical bytes.
_FIXED_DATE = "2000/01/01 00:00"
#: `///COMM` header line.
_COMM = "tesseract emitters"
#: `///ATTR` header line (standard read/write job attributes).
_ATTR = "SC,RW"
#: `///GROUP1` control group (single six-axis robot).
_GROUP1 = "RB1"
#: `///NPOS` trailing fields: base, station, P, BP, EX variable counts (none used).
_NPOS_TRAILING = "0,0,0,0,0"


def _numbered_only(key: str | None, what: str) -> None:
    if key is not None:
        raise UnsupportedInstructionError(
            f"JBI: {what} {key!r} is a named signal; INFORM IO is addressed by number "
            "(OT#(i), AO#(i)), so a key cannot be mapped without guessing"
        )


class JbiBackend(ProgramBackend):
    """Stateful JBI backend: eager `//INST` buffer plus `//POS` list, assembled at finish."""

    def __init__(
        self,
        *,
        profiles: Mapping[str, JbiProfile],
        frames: FrameBinding[int, int],
        calibration: JbiPulseCalibration,
        program_name: str,
    ) -> None:
        self._profiles = profiles
        self._frames = frames
        self._calibration = calibration
        self._name = JBI_IDENTIFIERS.validate(program_name)
        self._jbi = JbiWriter()
        self._positions: list[str] = []
        self._tools: list[int] = []

    def _profile(self, name: str) -> JbiProfile:
        try:
            return self._profiles[name]
        except KeyError:
            raise MissingProfileError(name, list(self._profiles.keys())) from None

    def _add_position(self, row: str) -> int:
        """Append a pulse row to the `//POS` table; return its index."""
        self._positions.append(row)
        return len(self._positions) - 1

    # ProgramBackend protocol -------------------------------------------
    def session(self) -> AbstractContextManager[object]:
        return self._jbi

    def prog_start(self) -> None:
        pass

    def move_joint(self, m: JointMove) -> None:
        percent = self._profile(m.profile).joint_speed_percent
        m.frames.require_no_offset(BRAND)
        self._tools.append(self._frames.tool(m.frames, BRAND))
        Movj(self._add_position(pulse_pos(m.joints, self._calibration)), percent)

    def move_cartesian(self, m: CartesianMove) -> None:
        raise UnsupportedInstructionError(
            "JBI: Cartesian targets need ///RCONF configuration flags and a coordinate "
            "frame, which this emitter does not write; a pulse row from the IK seed would "
            "move to the seed, not the planned pose. Plan joint waypoints for JBI."
        )

    def dwell(self, e: Dwell) -> None:
        TimerInst(e.seconds)

    def wait_digital(self, e: WaitDigital) -> None:
        WaitIn(e.index, e.value, is_input=e.is_input)

    def set_digital(self, e: SetDigital) -> None:
        _numbered_only(e.key, "digital output")
        SetOut(e.index, e.value)

    def set_analog(self, e: SetAnalog) -> None:
        _numbered_only(e.key, "analog output")
        if e.index not in ANALOG_PORTS or not abs(e.value) <= ANALOG_MAX_V:
            raise UnsupportedInstructionError(
                f"JBI: AOUT AO#({e.index}) {e.value} is outside ports "
                f"{ANALOG_PORTS.start}..{ANALOG_PORTS.stop - 1} / ±{ANALOG_MAX_V} V"
            )
        AnalogOut(e.index, e.value)

    def tool_change(self, e: ToolChange) -> None:
        raise UnsupportedInstructionError(
            f"JBI: SetTool({e.tool_id}) has no INFORM statement here; the position table's "
            "///TOOL follows the moves' tcp_frame through the frame binding"
        )

    def note(self, e: Note) -> None:
        Comment(e.text)

    def prog_finish(self) -> dict[str, str]:
        return {f"{self._name}.jbi": "\n".join(self._assemble()) + "\n"}

    # File assembly -----------------------------------------------------
    def _assemble(self) -> list[str]:
        """Build the full `.jbi` line list: `/JOB` + `//POS` table + `//INST` body."""
        lines = [
            "/JOB",
            f"//NAME {self._name}",
            "//POS",
            f"///NPOS {len(self._positions)},{_NPOS_TRAILING}",
        ]
        if self._positions:
            tools = set(self._tools)
            if len(tools) != 1:
                raise ToolConflictError(BRAND, self._tools)
            lines += [f"///TOOL {tools.pop()}", "///POSTYPE PULSE", "///PULSE"]
            lines += [f"{pos_var_name(i)}={row}" for i, row in enumerate(self._positions)]
        lines += [
            "//INST",
            f"///DATE {_FIXED_DATE}",
            f"///COMM {_COMM}",
            f"///ATTR {_ATTR}",
            f"///GROUP1 {_GROUP1}",
            "NOP",
        ]
        body = self._jbi.getvalue()
        if body:
            lines.extend(body.split("\n"))
        lines.append("END")
        return lines
