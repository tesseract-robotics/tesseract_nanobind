"""Universal Robots URScript code-generation DSL — class-per-statement.

A thin specialization of the shared ``core.dsl`` skeleton (the twin of
``emitters.krl.krl_writer``): ``UrScriptWriter`` sets the tab width,
``UrScriptCommand`` binds the writer, and ``Def`` is a ``Block``. Every statement
writes itself on instantiation, so the DSL reads like the target language —

```python
with Def("weld_01"):
    Comment("approach")
    MoveJ("[0.0, -0.349, 0.524, 0.0, 0.873, 0.0]", accel=1.4, speed=1.05)
    MoveL("p[0.6, -0.1, 0.8, 3.141593, 0.0, 0.0]", accel=1.2, speed=0.25, blend=0.0)
```

``UrScriptBackend`` drives these same classes from the brand-neutral IR, exactly
as ``KrlBackend`` drives the KRL DSL.
"""

from __future__ import annotations

from typing import ClassVar

from ..core.dsl import Block, Command, Writer

#: Motion-parameter scalars (a, v, t, r) print with this many decimals.
_PARAM_DECIMALS = 6
#: ``sleep(<s>)`` prints its duration with this many decimals.
_SLEEP_DECIMALS = 3
#: URScript blocking-time-out is disabled with ``t=0`` (move uses speed/accel).
_NO_TIMEOUT = 0


class UrScriptWriter(Writer):
    """URScript code buffer: 2-space indent, no leading newline (UR convention)."""

    _tab: ClassVar[str] = "  "
    _leading_newline: ClassVar[bool] = False


class UrScriptCommand(Command):
    """Base for every URScript DSL class; bound to the ``UrScriptWriter`` singleton."""

    writer: ClassVar[UrScriptWriter] = UrScriptWriter()


class Def(UrScriptCommand, Block):
    """``def <name>(): ... end`` program block."""

    def __init__(self, name: str) -> None:
        Block.__init__(self, f"def {name}():", "end")


class Comment(UrScriptCommand):
    """``# <text>`` — URScript line comment."""

    def __init__(self, text: str) -> None:
        self.writer.write(f"# {text}")


class MoveJ(UrScriptCommand):
    """``movej([j…], a=<accel>, v=<speed>, t=0, r=0)`` — joint-space move.

    ``joints_str`` is a pre-formatted joint vector literal ``[j1, …, j6]`` in
    radians; ``accel`` rad/s², ``speed`` rad/s.
    """

    def __init__(self, joints_str: str, accel: float, speed: float) -> None:
        self.writer.write(
            f"movej({joints_str}, a={accel:.{_PARAM_DECIMALS}f}, "
            f"v={speed:.{_PARAM_DECIMALS}f}, t={_NO_TIMEOUT}, r={_NO_TIMEOUT})"
        )


class MoveJPose(UrScriptCommand):
    """``movej(get_inverse_kin(p[…]), a=<accel>, v=<speed>, t=0, r=0)``.

    Joint-interpolated move to a Cartesian pose: the controller solves IK for the
    target so a FREESPACE Cartesian waypoint executes as a (faster, joint-space)
    ``movej`` rather than a straight-line ``movel``.
    """

    def __init__(self, pose_str: str, accel: float, speed: float) -> None:
        self.writer.write(
            f"movej(get_inverse_kin({pose_str}), a={accel:.{_PARAM_DECIMALS}f}, "
            f"v={speed:.{_PARAM_DECIMALS}f}, t={_NO_TIMEOUT}, r={_NO_TIMEOUT})"
        )


class MoveL(UrScriptCommand):
    """``movel(p[…], a=<accel>, v=<speed>, t=0, r=<blend>)`` — linear move.

    ``pose_str`` is a pre-formatted pose literal ``p[x, y, z, rx, ry, rz]``
    (metres + rotation-vector radians); ``accel`` m/s², ``speed`` m/s, ``blend``
    the blend radius in metres (0 ⇒ exact stop).
    """

    def __init__(self, pose_str: str, accel: float, speed: float, blend: float) -> None:
        self.writer.write(
            f"movel({pose_str}, a={accel:.{_PARAM_DECIMALS}f}, "
            f"v={speed:.{_PARAM_DECIMALS}f}, t={_NO_TIMEOUT}, r={blend:.{_PARAM_DECIMALS}f})"
        )


class SetDigitalOut(UrScriptCommand):
    """``set_standard_digital_out(<index>, True|False)`` — drive a digital output."""

    def __init__(self, index: int, value: bool) -> None:
        self.writer.write(f"set_standard_digital_out({index}, {'True' if value else 'False'})")


class WaitDigitalIn(UrScriptCommand):
    """``while (get_standard_digital_in(i) != True): sync() end`` — block on input.

    URScript has no atomic wait-for-IO primitive, so a digital wait is a polling
    loop with ``sync()`` to yield a control cycle each iteration.
    """

    def __init__(self, index: int, value: bool) -> None:
        target = "True" if value else "False"
        self.writer.write(f"while (get_standard_digital_in({index}) != {target}):")
        self.writer.indent()
        self.writer.write("sync()")
        self.writer.dedent()
        self.writer.write("end")


class WaitDigitalOut(UrScriptCommand):
    """``while (get_standard_digital_out(i) != True): sync() end`` — block on output."""

    def __init__(self, index: int, value: bool) -> None:
        target = "True" if value else "False"
        self.writer.write(f"while (get_standard_digital_out({index}) != {target}):")
        self.writer.indent()
        self.writer.write("sync()")
        self.writer.dedent()
        self.writer.write("end")


class Sleep(UrScriptCommand):
    """``sleep(<seconds>)`` — pause execution for a fixed duration."""

    def __init__(self, seconds: float) -> None:
        self.writer.write(f"sleep({seconds:.{_SLEEP_DECIMALS}f})")
