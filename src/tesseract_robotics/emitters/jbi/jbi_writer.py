"""Yaskawa Motoman INFORM (JBI) code-generation DSL — class-per-line statements.

A thin specialization of the shared ``core.dsl`` skeleton (the twin of
``emitters.krl.krl_writer``): ``JbiWriter`` sets the (empty) tab — JBI files are
flat, not indented — and disables the leading newline; ``JbiCommand`` binds the
writer. Every class writes exactly one line on instantiation, so the backend
reads like the file it produces —

```python
Job()                       # /JOB
Name("SPIKE")               # //NAME SPIKE
PosHeader()                 # //POS
NPos(4)                     # ///NPOS 4,0,0,0,0,0
...
PosVar(0, "0.000,...")      # C00000=0.000,...
InstHeader()                # //INST
Nop()                       # NOP
Comment("roundtrip spike")  # 'roundtrip spike
Movj(0, 50.0)               # MOVJ C00000 VJ=50.00
End()                       # END
```

Unlike KRL there is no ``with``-block scope: a ``.jbi`` is two flat sections
(``//POS`` then ``//INST``), so the backend (``JbiBackend``) buffers the
position tuples and instruction events as it walks the IR, then emits every line
through these classes in file order from ``prog_finish``.
"""

from __future__ import annotations

from typing import ClassVar

from ..core.dsl import Command, Writer

#: ``C`` table index is zero-padded to 5 digits (``C00000``, ``C00012``, …).
_POS_INDEX_WIDTH = 5
#: ``MOVJ … VJ=`` joint speed percent is printed with 2 decimals.
_VJ_DECIMALS = 2
#: ``MOVL … V=`` linear speed (mm/s) is printed with 1 decimal.
_V_DECIMALS = 1
#: Number of position fields after ``///NPOS`` (robot count + 5 fixed zeros).
_NPOS_TRAILING_ZEROS = 5


def pos_var_name(index: int) -> str:
    """Return the zero-padded position-variable name for ``index`` (``C00000``)."""
    return f"C{index:0{_POS_INDEX_WIDTH}d}"


class JbiWriter(Writer):
    """JBI code buffer: no indentation, no leading newline (Motoman convention)."""

    _tab: ClassVar[str] = ""
    _leading_newline: ClassVar[bool] = False


class JbiCommand(Command):
    """Base for every JBI DSL class; bound to the ``JbiWriter`` singleton."""

    writer: ClassVar[JbiWriter] = JbiWriter()


class Job(JbiCommand):
    """``/JOB`` — the file's first line, marking it a Motoman job."""

    def __init__(self) -> None:
        self.writer.write("/JOB")


class Name(JbiCommand):
    """``//NAME <name>`` — the job name."""

    def __init__(self, name: str) -> None:
        self.writer.write(f"//NAME {name}")


class PosHeader(JbiCommand):
    """``//POS`` — opens the position-variable section."""

    def __init__(self) -> None:
        self.writer.write("//POS")


class NPos(JbiCommand):
    """``///NPOS <robot>,0,0,0,0,0`` — number of robot (and aux) positions."""

    def __init__(self, robot_count: int) -> None:
        zeros = ",".join(["0"] * _NPOS_TRAILING_ZEROS)
        self.writer.write(f"///NPOS {robot_count},{zeros}")


class Tool(JbiCommand):
    """``///TOOL <id>`` — active tool number for the positions."""

    def __init__(self, tool_id: int) -> None:
        self.writer.write(f"///TOOL {tool_id}")


class PosType(JbiCommand):
    """``///POSTYPE PULSE`` followed by ``///PULSE`` — declare pulse coordinates.

    v1 writes joint angles in degrees into the table under this declaration (see
    ``jbi.targets.pulse_pos``); a faithful encoder-pulse conversion is out of
    scope as it needs per-robot gear ratios.
    """

    def __init__(self) -> None:
        self.writer.write("///POSTYPE PULSE")
        self.writer.write("///PULSE")


class PosVar(JbiCommand):
    """``C#####=<tuple>`` — one position-table row."""

    def __init__(self, index: int, tuple_text: str) -> None:
        self.writer.write(f"{pos_var_name(index)}={tuple_text}")


class InstHeader(JbiCommand):
    """``//INST`` — opens the instruction section."""

    def __init__(self) -> None:
        self.writer.write("//INST")


class Date(JbiCommand):
    """``///DATE <yyyy/mm/dd hh:mm>`` — a FIXED date (determinism, never live)."""

    def __init__(self, date_text: str) -> None:
        self.writer.write(f"///DATE {date_text}")


class Comm(JbiCommand):
    """``///COMM <text>`` — the job comment line in the ``//INST`` header."""

    def __init__(self, text: str) -> None:
        self.writer.write(f"///COMM {text}")


class Attr(JbiCommand):
    """``///ATTR <attrs>`` — job attributes (e.g. ``SC,RW``)."""

    def __init__(self, attrs: str) -> None:
        self.writer.write(f"///ATTR {attrs}")


class Group(JbiCommand):
    """``///GROUP1 <robot>`` — the control group the instructions drive."""

    def __init__(self, robot: str) -> None:
        self.writer.write(f"///GROUP1 {robot}")


class Nop(JbiCommand):
    """``NOP`` — the instruction-section opener (paired with ``END``)."""

    def __init__(self) -> None:
        self.writer.write("NOP")


class End(JbiCommand):
    """``END`` — the instruction-section terminator."""

    def __init__(self) -> None:
        self.writer.write("END")


class Comment(JbiCommand):
    """``'<text>`` — INFORM line comment (apostrophe-prefixed)."""

    def __init__(self, text: str) -> None:
        self.writer.write(f"'{text}")


class Movj(JbiCommand):
    """``MOVJ C##### VJ=<percent>`` — joint-interpolated move to a position var."""

    def __init__(self, index: int, joint_speed_percent: float) -> None:
        self.writer.write(f"MOVJ {pos_var_name(index)} VJ={joint_speed_percent:.{_VJ_DECIMALS}f}")


class Movl(JbiCommand):
    """``MOVL C##### V=<mm/s>`` — linear-interpolated move to a position var."""

    def __init__(self, index: int, linear_speed_mms: float) -> None:
        self.writer.write(f"MOVL {pos_var_name(index)} V={linear_speed_mms:.{_V_DECIMALS}f}")


class WaitIn(JbiCommand):
    """``WAIT IN#(i)=ON|OFF`` — block until a digital input matches.

    ``is_input=False`` selects an output bit (``OT#``) instead.
    """

    def __init__(self, index: int, value: bool, *, is_input: bool) -> None:
        signal = "IN" if is_input else "OT"
        self.writer.write(f"WAIT {signal}#({index})={'ON' if value else 'OFF'}")


class SetOut(JbiCommand):
    """``DOUT OT#(i) ON|OFF`` — drive a digital output bit."""

    def __init__(self, index: int, value: bool) -> None:
        self.writer.write(f"DOUT OT#({index}) {'ON' if value else 'OFF'}")


class TimerInst(JbiCommand):
    """``TIMER T=<seconds>`` — dwell for a fixed duration (seconds)."""

    def __init__(self, seconds: float) -> None:
        self.writer.write(f"TIMER T={seconds:g}")
