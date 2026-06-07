"""Yaskawa Motoman INFORM (JBI) code-generation DSL — class-per-``//INST``-line.

A thin specialization of the shared ``core.dsl`` skeleton (the twin of
``emitters.ls.ls_writer``): ``JbiWriter`` sets the (empty) tab — JBI files are
flat, not indented — and disables the leading newline; ``JbiCommand`` binds the
writer. Each statement class writes one ``//INST`` instruction line on
instantiation, so ``JbiBackend`` drives them eagerly as it walks the IR —

```python
Comment("roundtrip spike")  # 'roundtrip spike
Movj(0, 50.0)               # MOVJ C00000 VJ=50.00
Movl(2, 100.0)              # MOVL C00002 V=100.0
```

A ``.jbi`` is two flat sections (``//POS`` then ``//INST``); only the
**instruction** body flows through this writer's buffer. The static frame
(``/JOB`` / ``//NAME`` / the ``//POS`` table / the ``//INST`` header / ``END``)
is assembled from a template in ``JbiBackend`` — exactly as ``LsBackend``
assembles ``/PROG`` + ``/ATTR`` + ``/POS`` around the ``/MN`` DSL buffer.
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


def pos_var_name(index: int) -> str:
    """Return the zero-padded position-variable name for ``index`` (``C00000``)."""
    return f"C{index:0{_POS_INDEX_WIDTH}d}"


class JbiWriter(Writer):
    """JBI ``//INST`` body buffer: no indentation, no leading newline (Motoman convention)."""

    _tab: ClassVar[str] = ""
    _leading_newline: ClassVar[bool] = False


class JbiCommand(Command):
    """Base for every JBI ``//INST`` DSL class; bound to the ``JbiWriter`` singleton."""

    writer: ClassVar[JbiWriter] = JbiWriter()


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
