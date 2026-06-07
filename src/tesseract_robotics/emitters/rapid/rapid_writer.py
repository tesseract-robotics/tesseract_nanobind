"""ABB RAPID code-generation DSL — class-per-statement with ``with``-block scopes.

A specialization of the shared ``core.dsl`` skeleton (the exemplar the other
brands' ``*_writer`` modules mirror): every ``RapidCommand`` subclass writes into
a shared ``RapidWriter`` singleton on instantiation; ``with Module(): with Proc():
…`` blocks indent/dedent that singleton's buffer. Read ``RapidWriter().getvalue()``
for the accumulated bytes, ``.clear()`` to reset between emit sessions.

The data bundles (``RapidProfile`` and the typed names) live in ``profile``; the
target bundles + literal formatters live in ``targets``. This module is the DSL
surface only.
"""

from __future__ import annotations

import logging
from collections.abc import Sequence
from typing import ClassVar

from tesseract_robotics.tesseract_common import Quaterniond

from ..core.dsl import Block, Command, Writer
from .profile import (
    _DEFAULT_PROFILE,
    RapidProfile,
    SpeedName,
    ToolName,
    WobjName,
    ZoneName,
)
from .targets import (
    JointTarget,
    RapidTarget,
    _cog_kg,
    _move_line,
    _movec_line,
    _quat_wxyz,
    _robtarget_str,
    _xyz_mm,
)
from .utils import MotionType, RapidType, get_quat, get_rapid_bool

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Writer singleton + DSL classes.
# ---------------------------------------------------------------------------


class RapidWriter(Writer):
    """RAPID code buffer: 4-space indent, leading newline (ABB convention).

    Specializes the shared ``core.dsl.Writer`` with RAPID's tab width and
    leading-newline policy, plus a declared-name set used by ``AssignVariable``
    to reject assignments to undeclared variables. ``.clear()`` (inherited) resets
    everything in place via ``_setup``.
    """

    _tab: ClassVar[str] = "    "
    _leading_newline: ClassVar[bool] = True

    _declared_names: set[str]

    def _setup(self) -> None:
        super()._setup()
        self._declared_names = set()

    def __enter__(self) -> RapidWriter:
        return self

    def __exit__(self, *_: object) -> None:
        self.dedent()

    def declare(self, name: str) -> None:
        self._declared_names.add(name)

    def is_declared(self, name: str) -> bool:
        return name in self._declared_names


class RapidCommand(Command):
    """Base class for every RAPID DSL emitter; bound to the ``RapidWriter`` singleton."""

    # TODO: validate variable name length (RAPID limit is 16 chars) and
    # regex-check that the name is a legal RAPID identifier.
    writer: ClassVar[RapidWriter] = RapidWriter()


class While(RapidCommand, Block):
    """``WHILE <condition> ... ENDWHILE`` block."""

    def __init__(self, condition: str) -> None:
        self.condition = condition
        Block.__init__(self, f"WHILE {condition}", "ENDWHILE")


class For(RapidCommand, Block):
    """``FOR <i> FROM <start> TO <end> DO ... ENDFOR`` block."""

    def __init__(self, loop_counter: str, start: int, end: int) -> None:
        Block.__init__(self, f"FOR {loop_counter} FROM {start} TO {end} DO", "ENDFOR")


class Proc(RapidCommand, Block):
    """``PROC <name>() ... ENDPROC`` block."""

    def __init__(self, name: str) -> None:
        Block.__init__(self, f"PROC {name}()", "ENDPROC")


class Module(RapidCommand, Block):
    """``MODULE <name> ... ENDMODULE`` block."""

    def __init__(self, name: str) -> None:
        Block.__init__(self, f"MODULE {name}", "ENDMODULE\n")


class Robtarget(RapidCommand):
    """Emit a ``<rapid_type> robtarget <varname> := [...];`` declaration.

    Position metres → mm, quaternion scalar-last → scalar-first, both inline.
    Returns the declared ``varname`` so the call composes with ``MoveL`` / ``MoveJ``
    / ``MoveC`` which accept either a ``RapidTarget`` literal or a varname string.

    Usage:

    ```python
    target = RapidTarget(Pose.from_xyz_quat([0.6, -0.1, 0.8], [0, 0.707, 0, 0.707]))
    p1 = Robtarget("p1", target)   # emits the declaration, returns "p1"
    MoveL(p1, profile)             # references by varname
    ```

    Example output:

    ```rapid
    PERS p1 := [ [600.000, -100.000, 800.000],
                 [0.707106781, 0.000000000, 0.707106781, 0.000000000],
                 [0, 0, 0, 0],
                 [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09] ];
    ```
    """

    def __new__(
        cls,
        varname: str,
        target: RapidTarget,
        rapid_type: RapidType = RapidType.PERS,
    ) -> str:
        t = target.pose.translation
        qx, qy, qz, qw = Quaterniond(target.pose.linear).coeffs()
        x_mm, y_mm, z_mm = t[0] * 1000.0, t[1] * 1000.0, t[2] * 1000.0
        decl = (
            f"{rapid_type} {varname} := "
            f"[ [{x_mm:.3f}, {y_mm:.3f}, {z_mm:.3f}],"
            f"[{qw:.9f}, {qx:.9f}, {qy:.9f}, {qz:.9f}],"
            f"{target.config},"
            f"{target.external_axis} ];"
        )
        cls.writer.write(decl)
        cls.writer.declare(varname)
        return varname


class MoveJ(RapidCommand):
    """``MoveJ`` — joint-space path to a Cartesian target.

    Inline arguments make the generated code easier to adapt in RobotStudio and
    conserve controller memory (matters on older S4 / M94 controllers).
    ``target`` accepts a ``RapidTarget`` (inline literal) or ``str`` (varname).

    Example output (inline literal target):

    ```rapid
    MoveJ [[600.0000, -100.0000, 800.0000],
           [0.70710678, 0.00000000, 0.70710678, 0.00000000],
           [1, 0, 0, 0],
           [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],
          v200, z10, tool0 \\Wobj:=wobj0;
    ```

    Example output (varname target):

    ```rapid
    MoveJ p1, v200, z10, tool0 \\Wobj:=wobj0;
    ```
    """

    def __init__(
        self,
        target: RapidTarget | str,
        profile: RapidProfile = _DEFAULT_PROFILE,
    ) -> None:
        self.writer.write(_move_line("MoveJ", _robtarget_str(target), profile))


class MoveL(RapidCommand):
    """``MoveL`` — straight-line Cartesian path to target.

    ``target`` accepts a ``RapidTarget`` (inline literal) or ``str`` (varname).

    Example output (inline literal target):

    ```rapid
    MoveL [[500.0000, 0.0000, 500.0000],
           [1.00000000, 0.00000000, 0.00000000, 0.00000000],
           [1, 0, 0, 0],
           [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],
          v200, z10, tool0 \\Wobj:=wobj0;
    ```

    Example output (varname target):

    ```rapid
    MoveL p1, v200, z10, tool0 \\Wobj:=wobj0;
    ```
    """

    def __init__(
        self,
        target: RapidTarget | str,
        profile: RapidProfile = _DEFAULT_PROFILE,
    ) -> None:
        self.writer.write(_move_line("MoveL", _robtarget_str(target), profile))


class MoveC(RapidCommand):
    """``MoveC`` — circular interpolation through a via-point to an endpoint.

    Each of ``via`` / ``end`` can independently be a ``RapidTarget`` (inline
    literal) or a ``str`` (already-declared robtarget varname).

    Example output (declared via + end robtargets):

    ```rapid
    MoveC p_via, p_end, v200, z10, tool0 \\Wobj:=wobj0;
    ```
    """

    def __init__(
        self,
        via: RapidTarget | str,
        end: RapidTarget | str,
        profile: RapidProfile = _DEFAULT_PROFILE,
    ) -> None:
        self.writer.write(_movec_line(_robtarget_str(via), _robtarget_str(end), profile))


class MoveAbsJ(RapidCommand):
    """``MoveAbsJ`` — absolute-joints move; controller plays back exact joint angles.

    Takes a ``JointTarget`` (6 radians + optional external axis), symmetric with
    ``MoveL(target: RapidTarget, profile)``. ``profile.wobj`` is unused because
    joint space is independent of any work object.

    Example output:

    ```rapid
    MoveAbsJ [[0.00, 90.00, 0.00, 0.00, 0.00, 0.00],
              [9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],
             v200, z10, tool0;
    ```
    """

    def __init__(
        self,
        target: JointTarget,
        profile: RapidProfile = _DEFAULT_PROFILE,
    ) -> None:
        self.writer.write(f"MoveAbsJ {target}, {profile.speed}, {profile.zone}, {profile.tool};")


class Comment(RapidCommand):
    """``! <text>`` — RAPID line comment."""

    def __init__(self, comment: str) -> None:
        self.writer.write(f"! {comment}")


class TPWrite(RapidCommand):
    """``TPWrite "<text>";`` — write a string to the FlexPendant teach pendant."""

    def __init__(self, comment: str) -> None:
        self.writer.write(f'TPWrite "{comment}";')


class Stop(RapidCommand):
    """``Stop;`` — halt program execution; resumeable from FlexPendant."""

    def __init__(self) -> None:
        self.writer.write("Stop;")


# ---------------------------------------------------------------------------
# I/O instructions — added for tesseract 0.34.1.7's Wait/Timer/Set* bindings.
# ---------------------------------------------------------------------------


class WaitTime(RapidCommand):
    """``WaitTime <seconds>;`` — block program for a fixed duration.

    Example output:

    ```rapid
    WaitTime 2.5;
    ```
    """

    def __init__(self, seconds: float) -> None:
        self.writer.write(f"WaitTime {seconds};")


class WaitDI(RapidCommand):
    """``WaitDI <signal>, <0|1>;`` — block until digital input matches ``value``.

    Example output:

    ```rapid
    WaitDI di_part_present, 1;
    ```
    """

    def __init__(self, signal: str, value: bool = True) -> None:
        self.writer.write(f"WaitDI {signal}, {1 if value else 0};")


class WaitDO(RapidCommand):
    """``WaitDO <signal>, <0|1>;`` — block until digital output matches ``value``.

    Example output:

    ```rapid
    WaitDO do_torch, 1;
    ```
    """

    def __init__(self, signal: str, value: bool = True) -> None:
        self.writer.write(f"WaitDO {signal}, {1 if value else 0};")


class SetDO(RapidCommand):
    """``SetDO <signal>, <0|1>;`` — drive digital output.

    Example output:

    ```rapid
    SetDO do_torch, 1;
    ```
    """

    def __init__(self, signal: str, value: bool) -> None:
        self.writer.write(f"SetDO {signal}, {1 if value else 0};")


class SetAO(RapidCommand):
    """``SetAO <signal>, <value>;`` — drive analog output to ``value``.

    Example output:

    ```rapid
    SetAO ao_voltage, 24.5;
    ```
    """

    def __init__(self, signal: str, value: float) -> None:
        self.writer.write(f"SetAO {signal}, {value};")


class Conf(RapidCommand):
    """``ConfL/ConfJ \\On|\\Off;`` — enable or disable configuration monitoring.

    ``motion`` selects ConfL (``MotionType.LINEAR``) vs ConfJ (``MotionType.JOINT``);
    ``on`` picks ``\\On`` (True) or ``\\Off`` (False).

    Example output:

    ```rapid
    ConfL \\On;
    ```
    """

    def __init__(self, motion: MotionType, on: bool) -> None:
        if motion not in (MotionType.LINEAR, MotionType.JOINT):
            raise ValueError(
                f"Conf accepts only MotionType.LINEAR or MotionType.JOINT, got {motion!r}"
            )
        keyword = "L" if motion == MotionType.LINEAR else "J"
        switch = r"\On" if on else r"\Off"
        self.writer.write(f"Conf{keyword} {switch};")


class Tooldata(RapidCommand):
    """Emit a ``PERS tooldata`` declaration; returns a typed ``ToolName``.

    Example output:

    ```rapid
    PERS tooldata tl_:=[TRUE,[[367.71,-0.25,435.92],[0,0.5,0,0.8660254]],
                        [20,[0,0,220],[1,0,0,0],0,0,0]];
    ```
    """

    def __new__(
        cls,
        varname: str,
        x: float,
        y: float,
        z: float,
        q: Sequence[float],
        cog: Sequence[float],
        kg: float,
        stationary_tool: bool = True,
    ) -> ToolName:
        q_seq = get_quat(q)
        stationary = get_rapid_bool(stationary_tool)
        # COG must be != (0,0,0); RAPID rejects degenerate inertia frames.
        body = (
            f"[ {_xyz_mm(x, y, z)}, {_quat_wxyz(q_seq)} ], "
            f"[ {_cog_kg(cog, kg)}, [ 1, 0, 0, 0 ], 0, 0, 0 ]"
        )
        cls.writer.write(f"PERS tooldata {varname} := [ {stationary}, {body} ];")
        cls.writer.declare(varname)
        return ToolName(varname)


class Workobject(RapidCommand):
    """Emit a ``PERS wobjdata`` declaration; returns a typed ``WobjName``.

    Example output:

    ```rapid
    PERS wobjdata wob0:=[FALSE,TRUE,"",
                         [[0.00,0.00,0.00],[1,0,0,0]],
                         [[0,0,0],[1,0,0,0]]];
    ```
    """

    def __new__(
        cls,
        varname: str,
        x: float = 0,
        y: float = 0,
        z: float = 0,
        q: Sequence[float] = (1, 0, 0, 0),
        hold_workobject: bool = False,
        fixed_coord_sys: bool = True,
    ) -> WobjName:
        hold_str = get_rapid_bool(hold_workobject)
        fixed_str = get_rapid_bool(fixed_coord_sys)
        user_coord = f"[ {_xyz_mm(x, y, z)}, {_quat_wxyz(q)} ]"
        object_coord = "[ [0, 0, 0], [1, 0, 0 ,0] ]"
        cls.writer.write(
            f"PERS wobjdata {varname} := "
            f'[ {hold_str}, {fixed_str}, "", '
            f"{user_coord}, {object_coord} ];"
        )
        cls.writer.declare(varname)
        return WobjName(varname)


class Zone(RapidCommand):
    """Emit a ``PERS zonedata`` declaration; returns a typed ``ZoneName``.

    Args:
        name: RAPID variable name.
        finepoint: True → stop point (no blending), False → fly-by zone.
        pzone_tcp: TCP zone radius (mm).
        pzone_ori: tool reorientation zone (mm).
        pzone_eax: external-axes zone (mm).
        zone_ori: tool reorientation zone (degrees).
        zone_eax: linear external-axes zone (mm).
        zone_reax: rotating external-axes zone (degrees).

    Example output:

    ```rapid
    PERS zonedata zone_demo := [FALSE, 1, 1, 250, 1, 200, 1];
    ```
    """

    def __new__(
        cls,
        name: str,
        finepoint: bool = False,
        pzone_tcp: int = 1,
        pzone_ori: int = 1,
        pzone_eax: int = 250,
        zone_ori: int = 1,
        zone_eax: int = 200,
        zone_reax: int = 1,
    ) -> ZoneName:
        finepoint_str = get_rapid_bool(finepoint)
        cls.writer.write(
            f"PERS zonedata {name} := "
            f"[{finepoint_str}, {pzone_tcp}, {pzone_ori}, {pzone_eax}, "
            f"{zone_ori}, {zone_eax}, {zone_reax}];"
        )
        cls.writer.declare(name)
        return ZoneName(name)


class Speed(RapidCommand):
    """Emit a ``PERS speeddata`` declaration; returns a typed ``SpeedName``.

    Example output:

    ```rapid
    PERS speeddata v_demo := [ 200.0, 200.0, 1000.0, 1000.0 ];
    ```
    """

    def __new__(
        cls,
        name: str,
        v_tcp: float = 200.0,
        v_ori: float = 200.0,
        v_leax: float = 1000.0,
        v_reax: float = 1000.0,
    ) -> SpeedName:
        cls.writer.write(f"PERS speeddata {name} := [ {v_tcp}, {v_ori}, {v_leax}, {v_reax} ];")
        cls.writer.declare(name)
        return SpeedName(name)


class Pos(RapidCommand):
    """Emit a ``PERS pos`` declaration (x/y/z position triple, no orientation).

    Example output:

    ```rapid
    PERS pos pos_demo := [ 0, 0, 100 ];
    ```
    """

    def __new__(cls, name: str, x: float, y: float, z: float) -> str:
        cls.writer.write(f"PERS pos {name} := [ {x}, {y}, {z} ];")
        cls.writer.declare(name)
        return name


class RapidBool(RapidCommand):
    """Emit a ``PERS bool`` declaration.

    Example output:

    ```rapid
    PERS bool flag_a := FALSE;
    ```
    """

    def __new__(cls, name: str, value: bool) -> str:
        cls.writer.write(f"PERS bool {name} := {get_rapid_bool(value)};")
        cls.writer.declare(name)
        return name


class AssignVariable(RapidCommand):
    """``<name> := <value>;`` — assign to an already-declared RAPID variable.

    Raises ``NameError`` if ``name`` has not been declared via one of the
    ``PERS``-emitting classes (``Speed`` / ``Zone`` / ``Tooldata`` / ``Workobject`` /
    ``Pos`` / ``RapidBool`` / ``Robtarget``) in the current session.

    Example output:

    ```rapid
    flag_b := False;
    ```
    """

    def __new__(cls, name: str, value: object) -> str:
        if not cls.writer.is_declared(name):
            raise NameError(f"cannot assign undeclared RAPID variable {name!r}")
        cls.writer.write(f"{name} := {value};")
        return name
