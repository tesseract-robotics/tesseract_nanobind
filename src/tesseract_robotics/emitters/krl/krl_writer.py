"""KUKA KRL code-generation DSL — class-per-statement with ``with``-block scopes.

A thin specialization of the shared ``core.dsl`` skeleton (the twin of
``emitters.rapid.rapid_writer``): ``KrlWriter`` sets the tab width, ``KrlCommand``
binds the writer, and ``Def`` is a ``Block``. Every statement writes itself on
instantiation, so the DSL reads like the target language —

```python
with Def("WELD_01"):
    Comment("approach")
    BasVelPtp(100)
    Ptp("{A1 0.0,A2 -20.0,A3 30.0,A4 0.0,A5 50.0,A6 0.0}")
    VelCp(0.1)
    Lin("{X 600.0,Y -100.0,Z 800.0,A 0.0,B 0.0,C 180.0}")
```

``KrlBackend`` drives these same classes from the brand-neutral IR, exactly as
``RapidBackend`` drives the RAPID DSL.
"""

from __future__ import annotations

from typing import ClassVar

from ..core.dsl import Block, Command, Writer

#: ``$VEL.CP`` is printed in metres/second with 5 decimals.
_VEL_CP_DECIMALS = 5


class KrlWriter(Writer):
    """KRL code buffer: 2-space indent, no leading newline (KUKA convention)."""

    _tab: ClassVar[str] = "  "
    _leading_newline: ClassVar[bool] = False


class KrlCommand(Command):
    """Base for every KRL DSL class; bound to the ``KrlWriter`` singleton."""

    writer: ClassVar[KrlWriter] = KrlWriter()


class Def(KrlCommand, Block):
    """``DEF <name>() ... END`` program block."""

    def __init__(self, name: str) -> None:
        Block.__init__(self, f"DEF {name}()", "END")


class Comment(KrlCommand):
    """``; <text>`` — KRL line comment."""

    def __init__(self, text: str) -> None:
        self.writer.write(f"; {text}")


class BasVelPtp(KrlCommand):
    """``BAS(#VEL_PTP, <percent>)`` — set PTP velocity as a percentage."""

    def __init__(self, percent: float) -> None:
        self.writer.write(f"BAS(#VEL_PTP, {percent:g})")


class VelCp(KrlCommand):
    """``$VEL.CP = <m/s>`` — set the Cartesian path velocity (metres/second)."""

    def __init__(self, m_per_s: float) -> None:
        self.writer.write(f"$VEL.CP = {m_per_s:.{_VEL_CP_DECIMALS}f}")


class Ptp(KrlCommand):
    """``PTP <target>`` — point-to-point move to an axis or frame target literal."""

    def __init__(self, target: str) -> None:
        self.writer.write(f"PTP {target}")


class Lin(KrlCommand):
    """``LIN <target>`` — linear move to a frame target literal."""

    def __init__(self, target: str) -> None:
        self.writer.write(f"LIN {target}")


class WaitSec(KrlCommand):
    """``WAIT SEC <seconds>`` — dwell for a fixed duration."""

    def __init__(self, seconds: float) -> None:
        self.writer.write(f"WAIT SEC {seconds:g}")


class WaitFor(KrlCommand):
    """``WAIT FOR ($IN[i] == TRUE)`` — block until a digital signal matches.

    ``is_input`` selects ``$IN`` vs ``$OUT``.
    """

    def __init__(self, index: int, value: bool, *, is_input: bool) -> None:
        signal = "$IN" if is_input else "$OUT"
        self.writer.write(f"WAIT FOR ({signal}[{index}] == {'TRUE' if value else 'FALSE'})")


class SetOut(KrlCommand):
    """``$OUT[i] = TRUE|FALSE`` — drive a digital output."""

    def __init__(self, index: int, value: bool) -> None:
        self.writer.write(f"$OUT[{index}] = {'TRUE' if value else 'FALSE'}")


class SetAnOut(KrlCommand):
    """``$ANOUT[i] = <value>`` — drive an analog output."""

    def __init__(self, index: int, value: float) -> None:
        self.writer.write(f"$ANOUT[{index}] = {value:g}")
