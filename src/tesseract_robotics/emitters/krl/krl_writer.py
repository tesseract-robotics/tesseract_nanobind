"""KUKA KRL code-generation DSL: class-per-statement with `with`-block scopes.

A thin specialization of the shared `core.dsl` skeleton: `KrlWriter` sets the
tab width, `KrlCommand` binds the writer type, and `Def` is a `Block`. Every
statement writes itself into the active `KrlWriter` on instantiation:

```python
with KrlWriter() as krl:
    with Def("WELD_01"):
        Comment("approach")
        BasVelPtp(100)
        Ptp("{A1 0.0000,A2 -20.0000,A3 30.0000,A4 0.0000,A5 50.0000,A6 0.0000}")
        VelCp(0.1)
        Lin("{X 600.000,Y -100.000,Z 800.000,A 0.0000,B 0.0000,C 180.0000}")
```
"""

from __future__ import annotations

from typing import ClassVar

from ..core.dsl import Block, Command, Writer
from ..core.precision import M_DECIMALS, MM_DECIMALS, fixed
from .syntax import KRL_IDENTIFIERS, comment_text


class KrlWriter(Writer):
    """KRL code buffer: 2-space indent, no leading newline (KUKA convention)."""

    _tab: ClassVar[str] = "  "
    _leading_newline: ClassVar[bool] = False


class KrlCommand(Command):
    """Base for every KRL DSL class; writes into the active `KrlWriter`."""

    writer_type: ClassVar[type[Writer]] = KrlWriter


class Def(KrlCommand, Block):
    """`DEF <name>() ... END` program block."""

    def __init__(self, name: str) -> None:
        Block.__init__(self, f"DEF {KRL_IDENTIFIERS.validate(name)}()", "END")


class Comment(KrlCommand):
    """`; <text>`: KRL line comment (never contains a line break)."""

    def __init__(self, text: str) -> None:
        self.writer.write(f"; {comment_text(text)}")


class BasVelPtp(KrlCommand):
    """`BAS(#VEL_PTP, <percent>)`: PTP velocity as a percentage of maximum."""

    def __init__(self, percent: float) -> None:
        self.writer.write(f"BAS(#VEL_PTP, {percent:g})")


class VelCp(KrlCommand):
    """`$VEL.CP = <m/s>`: Cartesian path velocity in metres/second."""

    def __init__(self, m_per_s: float) -> None:
        self.writer.write(f"$VEL.CP = {fixed(m_per_s, M_DECIMALS)}")


class ApoCdis(KrlCommand):
    """`$APO.CDIS = <mm>`: the distance criterion `C_DIS` approximation uses."""

    def __init__(self, mm: float) -> None:
        self.writer.write(f"$APO.CDIS = {fixed(mm, MM_DECIMALS)}")


class SetBase(KrlCommand):
    """`$BASE = <frame>`: select the base (work) frame."""

    def __init__(self, frame_expr: str) -> None:
        self.writer.write(f"$BASE = {frame_expr}")


class SetTool(KrlCommand):
    """`$TOOL = <frame>`: select the tool frame."""

    def __init__(self, frame_expr: str) -> None:
        self.writer.write(f"$TOOL = {frame_expr}")


def _approx(c_dis: bool) -> str:
    return " C_DIS" if c_dis else ""


class Ptp(KrlCommand):
    """`PTP <target> [C_DIS]`: point-to-point move to an axis target literal."""

    def __init__(self, target: str, *, c_dis: bool = False) -> None:
        self.writer.write(f"PTP {target}{_approx(c_dis)}")


class Lin(KrlCommand):
    """`LIN <target> [C_DIS]`: linear move to a frame target literal."""

    def __init__(self, target: str, *, c_dis: bool = False) -> None:
        self.writer.write(f"LIN {target}{_approx(c_dis)}")


class WaitSec(KrlCommand):
    """`WAIT SEC <seconds>`: dwell for a fixed duration."""

    def __init__(self, seconds: float) -> None:
        self.writer.write(f"WAIT SEC {seconds:g}")


class WaitFor(KrlCommand):
    """`WAIT FOR ($IN[i] == TRUE)`: block until a digital signal matches.

    `is_input` selects `$IN` vs `$OUT`.
    """

    def __init__(self, index: int, value: bool, *, is_input: bool) -> None:
        signal = "$IN" if is_input else "$OUT"
        self.writer.write(f"WAIT FOR ({signal}[{index}] == {'TRUE' if value else 'FALSE'})")


class SetOut(KrlCommand):
    """`$OUT[i] = TRUE|FALSE`: drive a digital output."""

    def __init__(self, index: int, value: bool) -> None:
        self.writer.write(f"$OUT[{index}] = {'TRUE' if value else 'FALSE'}")


class SetAnOut(KrlCommand):
    """`$ANOUT[i] = <value>`: drive an analog output."""

    def __init__(self, index: int, value: float) -> None:
        self.writer.write(f"$ANOUT[{index}] = {value:g}")
