"""ABB RAPID code generator — class-per-statement DSL with `with`-block scopes.

The DSL pattern: every `RapidCommand` subclass writes into a shared
`RapidWriter` singleton on instantiation; `with Module(): with Proc(): ...`
blocks indent/dedent that singleton's buffer. Output bytes are accumulated in
the singleton; call `RapidWriter().getvalue()` to read them, `.clear()` to
reset between independent emit sessions.

Type-tagged variable names — `SpeedName` / `ZoneName` / `ToolName` /
`WobjName` — are returned by the `Speed` / `Zone` / `Tooldata` / `Workobject`
declaration classes and consumed by `RapidProfile`. The type system enforces
that you cannot pass a `ToolName` where a `SpeedName` is expected, even though
both are strings at runtime.
"""

from __future__ import annotations

import logging
import math
from collections.abc import Sequence
from contextlib import AbstractContextManager
from dataclasses import dataclass, field
from io import StringIO
from typing import ClassVar

import numpy as np
from numpy.typing import ArrayLike

from tesseract_robotics.planning import Pose
from tesseract_robotics.tesseract_common import Quaterniond

from .utils import MotionType, RapidType, format_ext_axis, get_quat, get_rapid_bool

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Type-tagged variable-name strings (subclasses of `str`).
#
# Pre-built constants below name the four built-in references that exist on
# every ABB controller out of the box (no PERS declaration needed).
# ---------------------------------------------------------------------------


class SpeedName(str):
    """RAPID `speeddata` variable name. Returned by `Speed(name, …)`."""

    __slots__ = ()


class ZoneName(str):
    """RAPID `zonedata` variable name. Returned by `Zone(name, …)`."""

    __slots__ = ()


class ToolName(str):
    """RAPID `tooldata` variable name. Returned by `Tooldata(name, …)`."""

    __slots__ = ()


class WobjName(str):
    """RAPID `wobjdata` variable name. Returned by `Workobject(name, …)`."""

    __slots__ = ()


# ABB built-in references; safe defaults when no explicit declaration is needed.
v200: SpeedName = SpeedName("v200")
z10: ZoneName = ZoneName("z10")
tool0: ToolName = ToolName("tool0")
wobj0: WobjName = WobjName("wobj0")


@dataclass
class RapidProfile:
    """RAPID motion parameter bundle — typed references to declared
    `PERS speeddata` / `zonedata` / `tooldata` / `wobjdata` variables.

    Construct fields by either (a) using the built-in constants (`v200`,
    `z10`, `tool0`, `wobj0`) for variables already on every ABB controller,
    or (b) declaring custom variables via the `Speed` / `Zone` / `Tooldata` /
    `Workobject` classes inside a `Module` block — each returns a typed name
    usable here.
    """

    speed: SpeedName = v200
    zone: ZoneName = z10
    tool: ToolName = tool0
    wobj: WobjName = wobj0


@dataclass
class Config:
    """RAPID robtarget axis-quadrant config — 4 integers `[cf1, cf4, cf6, cfx]`.

    `cf1`, `cf4`, `cf6` are rotational quadrant indices for axes 1, 4, and 6:
    `0` = `[0, π/2)`, `1` = `[π/2, π)`, etc.; negative values use the inverse
    convention. `cfx` is a robot-model-specific encoded configuration for the
    remaining axes (2, 3, 5). Defaults to `[1, 0, 0, 0]` — RAPID's most common
    axis-quadrant placeholder.

    For robot-specific home configurations, use `Config.neutral_for(name)`.
    """

    cf1: int = 1
    cf4: int = 0
    cf6: int = 0
    cfx: int = 0

    def __str__(self) -> str:
        return f"[{self.cf1}, {self.cf4}, {self.cf6}, {self.cfx}]"

    @classmethod
    def neutral_for(cls, robot: str) -> Config:
        """Return a known-good neutral axis-quadrant config for the named robot.

        The "neutral" config is what the controller reports when the arm is at
        its home position with no axes wrapped past their quadrant boundaries
        — useful as a baseline before `ConfL \\Off` paths.

        Raises `ValueError` for unregistered robots; pass `Config(...)`
        explicitly instead.

        Example:
            ```python
            config = Config.neutral_for("abb_irb2400")
            target = RapidTarget(pose, config=config)
            ```
        """
        table = {
            "abb_irb1200": cls(cf1=1, cf4=0, cf6=0, cfx=0),
            "abb_irb2400": cls(cf1=1, cf4=0, cf6=0, cfx=0),
            "abb_irb4400": cls(cf1=1, cf4=0, cf6=0, cfx=0),
            "abb_irb6640": cls(cf1=1, cf4=0, cf6=0, cfx=0),
            "abb_irb6700": cls(cf1=1, cf4=0, cf6=0, cfx=0),
        }
        if robot not in table:
            raise ValueError(
                f"no neutral Config registered for robot {robot!r}; "
                f"known robots: {sorted(table.keys())}. "
                f"Pass `Config(cf1, cf4, cf6, cfx)` explicitly."
            )
        return table[robot]


@dataclass
class ExternalAxis:
    """RAPID external-axis values — up to six slots; unset slots serialize to
    the `9E+09` sentinel ABB uses for "no value"."""

    values: Sequence[float] = ()

    def __str__(self) -> str:
        return format_ext_axis(self.values)

    @classmethod
    def from_list(cls, values: Sequence[float]) -> ExternalAxis:
        return cls(values=values)


@dataclass
class RapidTarget:
    """RAPID robtarget bundle — pose + axis-quadrant config + external axes.

    The `.robtarget` property serializes this to a RAPID robtarget literal,
    handling both axis-convention conversions inline (m → mm; scalar-last
    `[qx,qy,qz,qw]` → scalar-first `[w,x,y,z]`).
    """

    pose: Pose
    config: Config = field(default_factory=Config)
    external_axis: ExternalAxis = field(default_factory=ExternalAxis)

    @property
    def robtarget(self) -> str:
        """Serialize to a RAPID robtarget literal `[[mm xyz], [wxyz quat], [config], [ext]]`."""
        t = self.pose.translation
        qx, qy, qz, qw = Quaterniond(self.pose.linear).coeffs()
        x_mm, y_mm, z_mm = t[0] * 1000.0, t[1] * 1000.0, t[2] * 1000.0
        pos = f"[{x_mm:.4f}, {y_mm:.4f}, {z_mm:.4f}]"
        rot = f"[{qw:.8f}, {qx:.8f}, {qy:.8f}, {qz:.8f}]"
        return "[" + ",".join([pos, rot, str(self.config), str(self.external_axis)]) + "]"


@dataclass
class JointTarget:
    """RAPID jointtarget bundle — 6-axis joint values (radians) + external axes.

    Mirrors `RapidTarget` for joint-space moves. `__str__` returns the RAPID
    jointtarget literal with joints converted radians → degrees and external
    axes padded to six slots with `9E+09`.
    """

    joints_rad: ArrayLike
    external_axis: ExternalAxis = field(default_factory=ExternalAxis)

    def __str__(self) -> str:
        joints = np.asarray(self.joints_rad, dtype=np.float64).ravel()
        if joints.size != 6:
            raise ValueError(
                f"JointTarget requires exactly 6 joint values; got {joints.size} ({joints!r})"
            )
        deg = [math.degrees(float(j)) for j in joints]
        joints_str = (
            f"[{deg[0]:.2f}, {deg[1]:.2f}, {deg[2]:.2f}, {deg[3]:.2f}, {deg[4]:.2f}, {deg[5]:.2f}]"
        )
        return f"[{joints_str}, {self.external_axis}]"


_DEFAULT_PROFILE = RapidProfile()


# ---------------------------------------------------------------------------
# Private formatters — consumed by the move classes below. Tooldata/Workobject
# use these inline rather than the old `_XYZ`/`_QUAT`/`_COG` `.format`-template
# chain (which required a two-pass `.format(**globals()).format(**vars())`).
# ---------------------------------------------------------------------------


def _move_line(keyword: str, robtarget: str, profile: RapidProfile) -> str:
    """Build a `MoveL` / `MoveJ` line referencing one robtarget."""
    return (
        f"{keyword} {robtarget}, "
        f"{profile.speed}, {profile.zone}, {profile.tool} \\Wobj:={profile.wobj};"
    )


def _movec_line(robtarget_via: str, robtarget_end: str, profile: RapidProfile) -> str:
    """Build a `MoveC` line referencing two robtargets (via, end)."""
    return (
        f"MoveC {robtarget_via}, {robtarget_end}, "
        f"{profile.speed}, {profile.zone}, {profile.tool} \\Wobj:={profile.wobj};"
    )


def _xyz_mm(x: float, y: float, z: float) -> str:
    """Format `[x, y, z]` for `tooldata` / `wobjdata` declarations (3-decimal mm)."""
    return f"[ {x:.3f}, {y:.3f}, {z:.3f} ]"


def _quat_wxyz(q: Sequence[float]) -> str:
    """Format scalar-first `[w, x, y, z]` quaternion (9-decimal precision)."""
    return f"[ {q[0]:.9f}, {q[1]:.9f}, {q[2]:.9f}, {q[3]:.9f} ]"


def _cog_kg(cog: Sequence[float], kg: float) -> str:
    """Format `kg, [cog_x, cog_y, cog_z]` for the `tooldata` inertia frame."""
    return f"{kg}, [ {cog[0]:.3f}, {cog[1]:.3f}, {cog[2]:.3f} ]"


def _robtarget_str(t: RapidTarget | str) -> str:
    """Accept either a `RapidTarget` (→ inline robtarget literal) or a `str`
    (→ already-declared robtarget variable name).

    Lets idiomatic RAPID workflows split between `MoveL [[...],[...],...], v200, ...`
    (one-shot, all data inline) and `CONST robtarget pN := [...]; MoveL pN, v200, ...`
    (large declared block, references-by-name in the procedure). The latter is
    preferred for long toolpaths and operator-tunable programs.
    """
    return t if isinstance(t, str) else t.robtarget


# ---------------------------------------------------------------------------
# Writer singleton + DSL classes.
# ---------------------------------------------------------------------------


class RapidWriter:
    """Shared output buffer + indent counter for the RAPID DSL.

    Singleton-via-`__new__`: the first `RapidWriter()` call creates the
    instance; subsequent calls return the same one. Every `RapidCommand`
    subclass holds a class-level reference to that instance, so nested
    `with` blocks all manipulate the same indent level and accumulate into
    the same buffer. Use `.clear()` to wipe the buffer, indent, and declaration
    cache between independent emit sessions.
    """

    _tab: ClassVar[str] = "    "
    _instance: ClassVar[RapidWriter | None] = None

    buffer: StringIO
    _indent: int
    _declared_names: set[str]

    def __new__(cls) -> RapidWriter:
        if cls._instance is None:
            instance = super().__new__(cls)
            instance.buffer = StringIO()
            instance._indent = 0
            instance._declared_names = set()
            cls._instance = instance
        return cls._instance

    def __enter__(self) -> RapidWriter:
        return self

    def __exit__(self, *_: object) -> None:
        self.dedent()

    def indent(self) -> None:
        self._indent += 1

    def dedent(self) -> None:
        self._indent -= 1

    def write(self, cmd: str) -> None:
        self.buffer.write("\n" + self._tab * self._indent + cmd)

    def declare(self, name: str) -> None:
        self._declared_names.add(name)

    def is_declared(self, name: str) -> bool:
        return name in self._declared_names

    def getvalue(self) -> str:
        return self.buffer.getvalue()

    def clear(self) -> None:
        """Reset buffer, indent, and declarations on the same singleton instance.

        `RapidCommand.rapid` captures this instance at import time, so
        `.clear()` (mutate in place) is correct; never replace the singleton
        via assignment, which would orphan the captured reference.
        """
        self.buffer = StringIO()
        self._indent = 0
        self._declared_names.clear()


class RapidCommand:
    """Base class for every DSL emitter; all subclasses share the same writer."""

    # TODO: validate variable name length (RAPID limit is 16 chars) and
    # regex-check that the name is a legal RAPID identifier.
    rapid: RapidWriter = RapidWriter()


class While(RapidCommand, AbstractContextManager["While"]):
    """`WHILE <condition> ... ENDWHILE` block."""

    def __init__(self, condition: str) -> None:
        self.condition = condition
        self.rapid.write(f"WHILE {condition}")

    def __enter__(self) -> While:
        self.rapid.indent()
        return self

    def __exit__(self, *_: object) -> None:
        self.rapid.dedent()
        self.rapid.write("ENDWHILE")


class For(RapidCommand, AbstractContextManager["For"]):
    """`FOR <i> FROM <start> TO <end> DO ... ENDFOR` block."""

    def __init__(self, loop_counter: str, start: int, end: int) -> None:
        self.rapid.write(f"FOR {loop_counter} FROM {start} TO {end} DO")

    def __enter__(self) -> For:
        self.rapid.indent()
        return self

    def __exit__(self, *_: object) -> None:
        self.rapid.dedent()
        self.rapid.write("ENDFOR")


class Proc(RapidCommand, AbstractContextManager["Proc"]):
    """`PROC <name>() ... ENDPROC` block."""

    def __init__(self, name: str) -> None:
        self.rapid.write(f"PROC {name}()")

    def __enter__(self) -> Proc:
        self.rapid.indent()
        return self

    def __exit__(self, *_: object) -> None:
        self.rapid.dedent()
        self.rapid.write("ENDPROC")


class Module(RapidCommand, AbstractContextManager["Module"]):
    """`MODULE <name> ... ENDMODULE` block."""

    def __init__(self, name: str) -> None:
        self.rapid.write(f"MODULE {name}")

    def __enter__(self) -> Module:
        self.rapid.indent()
        return self

    def __exit__(self, *_: object) -> None:
        self.rapid.dedent()
        self.rapid.write("ENDMODULE\n")


class Robtarget(RapidCommand):
    """Emit a `<rapid_type> robtarget <varname> := [...];` declaration.

    Position metres → mm, quaternion scalar-last → scalar-first, both inline.
    Returns the declared `varname` so the call composes with `MoveL`/`MoveJ`/
    `MoveC` which accept either a `RapidTarget` literal or a varname string.

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
        cls.rapid.write(decl)
        cls.rapid.declare(varname)
        return varname


class MoveJ(RapidCommand):
    """`MoveJ` — joint-space path to a Cartesian target.

    Inline arguments make the generated code easier to adapt in RobotStudio and
    conserve controller memory (matters on older S4 / M94 controllers).
    `target` accepts a `RapidTarget` (inline literal) or `str` (varname).

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
        self.rapid.write(_move_line("MoveJ", _robtarget_str(target), profile))


class MoveL(RapidCommand):
    """`MoveL` — straight-line Cartesian path to target.

    `target` accepts a `RapidTarget` (inline literal) or `str` (varname).

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
        self.rapid.write(_move_line("MoveL", _robtarget_str(target), profile))


class MoveC(RapidCommand):
    """`MoveC` — circular interpolation through a via-point to an endpoint.

    Each of `via` / `end` can independently be a `RapidTarget` (inline literal)
    or a `str` (already-declared robtarget varname).

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
        self.rapid.write(_movec_line(_robtarget_str(via), _robtarget_str(end), profile))


class MoveAbsJ(RapidCommand):
    """`MoveAbsJ` — absolute-joints move; controller plays back exact joint angles.

    Takes a `JointTarget` (6 radians + optional external axis), symmetric with
    `MoveL(target: RapidTarget, profile)`. `profile.wobj` is unused because
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
        self.rapid.write(f"MoveAbsJ {target}, {profile.speed}, {profile.zone}, {profile.tool};")


class Comment(RapidCommand):
    """`! <text>` — RAPID line comment."""

    def __init__(self, comment: str) -> None:
        self.rapid.write(f"! {comment}")


class TPWrite(RapidCommand):
    """`TPWrite "<text>";` — write a string to the FlexPendant teach pendant."""

    def __init__(self, comment: str) -> None:
        self.rapid.write(f'TPWrite "{comment}";')


class Stop(RapidCommand):
    """`Stop;` — halt program execution; resumeable from FlexPendant."""

    def __init__(self) -> None:
        self.rapid.write("Stop;")


# ---------------------------------------------------------------------------
# I/O instructions — added for tesseract 0.34.1.7's Wait/Timer/Set* bindings.
# ---------------------------------------------------------------------------


class WaitTime(RapidCommand):
    """`WaitTime <seconds>;` — block program for a fixed duration.

    Example output:

    ```rapid
    WaitTime 2.5;
    ```
    """

    def __init__(self, seconds: float) -> None:
        self.rapid.write(f"WaitTime {seconds};")


class WaitDI(RapidCommand):
    """`WaitDI <signal>, <0|1>;` — block until digital input matches `value`.

    Example output:

    ```rapid
    WaitDI di_part_present, 1;
    ```
    """

    def __init__(self, signal: str, value: bool = True) -> None:
        self.rapid.write(f"WaitDI {signal}, {1 if value else 0};")


class WaitDO(RapidCommand):
    """`WaitDO <signal>, <0|1>;` — block until digital output matches `value`.

    Example output:

    ```rapid
    WaitDO do_torch, 1;
    ```
    """

    def __init__(self, signal: str, value: bool = True) -> None:
        self.rapid.write(f"WaitDO {signal}, {1 if value else 0};")


class SetDO(RapidCommand):
    """`SetDO <signal>, <0|1>;` — drive digital output.

    Example output:

    ```rapid
    SetDO do_torch, 1;
    ```
    """

    def __init__(self, signal: str, value: bool) -> None:
        self.rapid.write(f"SetDO {signal}, {1 if value else 0};")


class SetAO(RapidCommand):
    """`SetAO <signal>, <value>;` — drive analog output to `value`.

    Example output:

    ```rapid
    SetAO ao_voltage, 24.5;
    ```
    """

    def __init__(self, signal: str, value: float) -> None:
        self.rapid.write(f"SetAO {signal}, {value};")


class Conf(RapidCommand):
    """`ConfL/ConfJ \\On|\\Off;` — enable or disable configuration monitoring.

    `motion` selects ConfL (`MotionType.LINEAR`) vs ConfJ (`MotionType.JOINT`);
    `on` picks `\\On` (True) or `\\Off` (False).

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
        self.rapid.write(f"Conf{keyword} {switch};")


class Tooldata(RapidCommand):
    """Emit a `PERS tooldata` declaration; returns a typed `ToolName`.

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
        cls.rapid.write(f"PERS tooldata {varname} := [ {stationary}, {body} ];")
        cls.rapid.declare(varname)
        return ToolName(varname)


class Workobject(RapidCommand):
    """Emit a `PERS wobjdata` declaration; returns a typed `WobjName`.

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
        cls.rapid.write(
            f"PERS wobjdata {varname} := "
            f'[ {hold_str}, {fixed_str}, "", '
            f"{user_coord}, {object_coord} ];"
        )
        cls.rapid.declare(varname)
        return WobjName(varname)


class Zone(RapidCommand):
    """Emit a `PERS zonedata` declaration; returns a typed `ZoneName`.

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
        cls.rapid.write(
            f"PERS zonedata {name} := "
            f"[{finepoint_str}, {pzone_tcp}, {pzone_ori}, {pzone_eax}, "
            f"{zone_ori}, {zone_eax}, {zone_reax}];"
        )
        cls.rapid.declare(name)
        return ZoneName(name)


class Speed(RapidCommand):
    """Emit a `PERS speeddata` declaration; returns a typed `SpeedName`.

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
        cls.rapid.write(f"PERS speeddata {name} := [ {v_tcp}, {v_ori}, {v_leax}, {v_reax} ];")
        cls.rapid.declare(name)
        return SpeedName(name)


class Pos(RapidCommand):
    """Emit a `PERS pos` declaration (x/y/z position triple, no orientation).

    Example output:

    ```rapid
    PERS pos pos_demo := [ 0, 0, 100 ];
    ```
    """

    def __new__(cls, name: str, x: float, y: float, z: float) -> str:
        cls.rapid.write(f"PERS pos {name} := [ {x}, {y}, {z} ];")
        cls.rapid.declare(name)
        return name


class RapidBool(RapidCommand):
    """Emit a `PERS bool` declaration.

    Example output:

    ```rapid
    PERS bool flag_a := FALSE;
    ```
    """

    def __new__(cls, name: str, value: bool) -> str:
        cls.rapid.write(f"PERS bool {name} := {get_rapid_bool(value)};")
        cls.rapid.declare(name)
        return name


class AssignVariable(RapidCommand):
    """`<name> := <value>;` — assign to an already-declared RAPID variable.

    Raises `NameError` if `name` has not been declared via one of the
    `PERS`-emitting classes (`Speed`/`Zone`/`Tooldata`/`Workobject`/`Pos`/
    `RapidBool`/`Robtarget`) in the current session.

    Example output:

    ```rapid
    flag_b := False;
    ```
    """

    def __new__(cls, name: str, value: object) -> str:
        if not cls.rapid.is_declared(name):
            raise NameError(f"cannot assign undeclared RAPID variable {name!r}")
        cls.rapid.write(f"{name} := {value};")
        return name
