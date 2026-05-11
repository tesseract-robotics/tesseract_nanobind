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

from .utils import (
    MOTION_TYPES,
    RapidType,
    format_ext_axis,
    get_quat,
    get_rapid_bool,
)

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Type-tagged variable-name strings.
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

    Typical usage: declare 3-4 of these at the top of an emit session and
    reuse them across all moves.
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
    remaining axes (2, 3, 5).

    Defaults to `[1, 0, 0, 0]` — RAPID's most common axis-quadrant placeholder.
    """

    cf1: int = 1
    cf4: int = 0
    cf6: int = 0
    cfx: int = 0

    def __str__(self) -> str:
        return f"[{self.cf1}, {self.cf4}, {self.cf6}, {self.cfx}]"


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
        qx, qy, qz, qw = self.pose.quaternion
        pos = (
            f"[{self.pose.x * 1000.0:.4f}, {self.pose.y * 1000.0:.4f}, {self.pose.z * 1000.0:.4f}]"
        )
        rot = f"[{qw:.8f}, {qx:.8f}, {qy:.8f}, {qz:.8f}]"
        return "[" + ",".join([pos, rot, str(self.config), str(self.external_axis)]) + "]"


_DEFAULT_PROFILE = RapidProfile()


# Format string templates for `Tooldata` / `Workobject` declarations
# (RAPID expects 3-decimal mm position and 9-decimal quaternion in PERS data).
_XYZ = "[ {x:.3f}, {y:.3f}, {z:.3f} ]"
_QUAT = "[ {q[0]:.9f}, {q[1]:.9f}, {q[2]:.9f}, {q[3]:.9f} ]"
_COG = "{kg}, [ {cog[0]:.3f}, {cog[1]:.3f}, {cog[2]:.3f} ]"


# ---------------------------------------------------------------------------
# Move-line formatters (private; consumed by Move/MoveC below).
# ---------------------------------------------------------------------------


def _move_line(keyword: str, robtarget: str, profile: RapidProfile) -> str:
    """Build a `MoveL` / `MoveJ` / `PrintL` line referencing one robtarget."""
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


# ---------------------------------------------------------------------------
# Writer singleton + DSL classes.
# ---------------------------------------------------------------------------


class RapidWriter:
    """Shared output buffer + indent counter for the RAPID DSL.

    Singleton-via-`__new__`: the first `RapidWriter()` call creates the
    instance; subsequent calls return the same one. Every `RapidCommand`
    subclass holds a class-level reference to that instance, so nested
    `with` blocks all manipulate the same indent level and accumulate into
    the same buffer. Use `.clear()` to wipe the buffer + indent between
    independent emit sessions.
    """

    _tab: ClassVar[str] = "    "
    _instance: ClassVar[RapidWriter | None] = None

    buffer: StringIO
    _indent: int

    def __new__(cls) -> RapidWriter:
        if cls._instance is None:
            instance = super().__new__(cls)
            instance.buffer = StringIO()
            instance._indent = 0
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

    def getvalue(self) -> str:
        return self.buffer.getvalue()

    def clear(self) -> None:
        """Reset buffer + indent on the same singleton instance.

        `RapidCommand.rapid` captures this instance at import time, so
        `.clear()` (mutate in place) is correct; never replace the singleton
        via assignment, which would orphan the captured reference.
        """
        self.buffer = StringIO()
        self._indent = 0


class RapidCommand:
    """Base class for every DSL emitter; all subclasses share the same writer."""

    # TODO: validate variable name length (RAPID limit is 16 chars) and
    # regex-check that the name is a legal RAPID identifier.
    rapid: RapidWriter = RapidWriter()


# `InlineRapidCode` removed — for already-formatted lines that don't fit any
# DSL class, call `RapidWriter().write("…")` directly. Same effect (the
# singleton writer is shared with every `RapidCommand` subclass), no class
# ceremony, no false implication of validation.


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
        self.proc = f"PROC {name}()"
        self.rapid.write(self.proc)

    def __call__(self) -> str:
        return self.proc

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
    """

    def __new__(
        cls,
        varname: str,
        target: RapidTarget,
        rapid_type: RapidType = RapidType.PERS,
    ) -> str:
        qx, qy, qz, qw = target.pose.quaternion
        X = target.pose.x * 1000.0
        Y = target.pose.y * 1000.0
        Z = target.pose.z * 1000.0
        decl = (
            f"{rapid_type} {varname} := "
            f"[ [{X:.3f}, {Y:.3f}, {Z:.3f}],"
            f"[{qw:.9f}, {qx:.9f}, {qy:.9f}, {qz:.9f}],"
            f"{target.config},"
            f"{target.external_axis} ];"
        )
        cls.rapid.write(decl)
        return varname


def _robtarget_str(t: RapidTarget | str) -> str:
    """Accept either a `RapidTarget` (→ inline robtarget literal) or a `str`
    (→ already-declared robtarget variable name).

    Lets idiomatic RAPID workflows split between `MoveL [[...],[...],...], v200, ...`
    (one-shot, all data inline) and `CONST robtarget pN := [...]; MoveL pN, v200, ...`
    (large declared block, references-by-name in the procedure). The latter is
    preferred for long toolpaths and operator-tunable programs.
    """
    return t if isinstance(t, str) else t.robtarget


class Move(RapidCommand):
    """Dispatch to `MoveL` / `MoveJ` / `PrintL` based on `move_type` string.

    Circular motion has its own `MoveC` class because RAPID's `MoveC` takes
    two robtargets (via + end), not one.
    """

    def __init__(
        self,
        move_type: str,
        target: RapidTarget | str,
        profile: RapidProfile = _DEFAULT_PROFILE,
    ) -> None:
        kinds = {"joint": "MoveJ", "linear": "MoveL", "print": "PrintL"}
        if move_type not in kinds:
            raise TypeError(f"expected one of {list(kinds.keys())}, got {move_type!r}")
        self.rapid.write(_move_line(kinds[move_type], _robtarget_str(target), profile))


class MoveJ(RapidCommand):
    """`MoveJ` — joint-space path to a Cartesian target.

    Inline arguments make the generated code easier to adapt in RobotStudio and
    conserve controller memory (matters on older S4 / M94 controllers).
    `target` accepts a `RapidTarget` (inline literal) or `str` (varname).
    """

    def __init__(
        self,
        target: RapidTarget | str,
        profile: RapidProfile = _DEFAULT_PROFILE,
    ) -> None:
        Move("joint", target, profile)


class MoveL(RapidCommand):
    """`MoveL` — straight-line Cartesian path to target.

    `target` accepts a `RapidTarget` (inline literal) or `str` (varname).
    """

    def __init__(
        self,
        target: RapidTarget | str,
        profile: RapidProfile = _DEFAULT_PROFILE,
    ) -> None:
        Move("linear", target, profile)


class PrintL(RapidCommand):
    """`PrintL` — extrusion-style move (FWR-shop custom; controller must define the function)."""

    def __init__(
        self,
        target: RapidTarget | str,
        profile: RapidProfile = _DEFAULT_PROFILE,
    ) -> None:
        Move("print", target, profile)


class MoveC(RapidCommand):
    """`MoveC` — circular interpolation through a via-point to an endpoint.

    Each of `via` / `end` can independently be a `RapidTarget` (inline literal)
    or a `str` (already-declared robtarget varname).
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

    Joint values are radians (tesseract convention) and converted to degrees
    inside (RAPID convention). MoveAbsJ has no `\\Wobj` argument because joint
    space is independent of any work object — `profile.wobj` is unused.

    `external_axis` is a direct kwarg rather than coming from a target bundle
    because joint moves don't share the cartesian-target shape (no pose, no
    config field).
    """

    def __init__(
        self,
        joints_rad: ArrayLike,
        profile: RapidProfile = _DEFAULT_PROFILE,
        external_axis: ExternalAxis | None = None,
    ) -> None:
        joints = np.asarray(joints_rad, dtype=np.float64).ravel()
        if joints.size != 6:
            msg = f"MoveAbsJ requires exactly 6 joint values; got {joints.size} ({joints!r})"
            logger.error(msg)
            raise ValueError(msg)
        deg = [math.degrees(float(j)) for j in joints]
        joints_str = (
            f"[{deg[0]:.2f}, {deg[1]:.2f}, {deg[2]:.2f}, {deg[3]:.2f}, {deg[4]:.2f}, {deg[5]:.2f}]"
        )
        ext = external_axis if external_axis is not None else ExternalAxis()
        jointtarget = f"[{joints_str}, {ext}]"
        self.rapid.write(
            f"MoveAbsJ {jointtarget}, {profile.speed}, {profile.zone}, {profile.tool};"
        )


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
    """`WaitTime <seconds>;` — block program for a fixed duration."""

    def __init__(self, seconds: float) -> None:
        self.rapid.write(f"WaitTime {seconds};")


class WaitDI(RapidCommand):
    """`WaitDI <signal>, <0|1>;` — block until digital input matches `value`."""

    def __init__(self, signal: str, value: bool = True) -> None:
        self.rapid.write(f"WaitDI {signal}, {1 if value else 0};")


class WaitDO(RapidCommand):
    """`WaitDO <signal>, <0|1>;` — block until digital output matches `value`."""

    def __init__(self, signal: str, value: bool = True) -> None:
        self.rapid.write(f"WaitDO {signal}, {1 if value else 0};")


class SetDO(RapidCommand):
    """`SetDO <signal>, <0|1>;` — drive digital output."""

    def __init__(self, signal: str, value: bool) -> None:
        self.rapid.write(f"SetDO {signal}, {1 if value else 0};")


class SetAO(RapidCommand):
    """`SetAO <signal>, <value>;` — drive analog output to `value`."""

    def __init__(self, signal: str, value: float) -> None:
        self.rapid.write(f"SetAO {signal}, {value};")


class Conf(RapidCommand):
    """`ConfL/ConfJ \\On|\\Off;` — enable or disable configuration monitoring.

    `joint_or_linear` selects ConfL (`MOTION_TYPES.linear`) vs ConfJ
    (`MOTION_TYPES.joint`); `on_or_off` picks `\\On` or `\\Off`.
    """

    def __init__(self, joint_or_linear: int, on_or_off: bool) -> None:
        if joint_or_linear not in (MOTION_TYPES.linear, MOTION_TYPES.joint):
            raise ValueError(
                f"joint_or_linear must be MOTION_TYPES.linear or .joint, got {joint_or_linear}"
            )
        keyword = "L" if joint_or_linear == MOTION_TYPES.linear else "J"
        switch = r"\On" if on_or_off else r"\Off"
        self.rapid.write(f"Conf{keyword} {switch};")


class Tooldata(RapidCommand):
    """Emit a `PERS tooldata` declaration; returns a typed `ToolName`.

    Example output::

        PERS tooldata tl_:=[TRUE,[[367.71,-0.25,435.92],[0,0.5,0,0.8660254]],
                            [20,[0,0,220],[1,0,0,0],0,0,0]];
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
        body_template = f"[ {_XYZ}, {_QUAT} ], [ {_COG}, [ 1, 0, 0, 0 ], 0, 0, 0 ]"
        body = body_template.format(x=x, y=y, z=z, q=q_seq, cog=cog, kg=kg)
        cls.rapid.write(f"PERS tooldata {varname} := [ {stationary}, {body} ];")
        return ToolName(varname)


class Workobject(RapidCommand):
    """Emit a `PERS wobjdata` declaration; returns a typed `WobjName`.

    Example output::

        PERS wobjdata wob0:=[FALSE,TRUE,"",
                             [[0.00,0.00,0.00],[1,0,0,0]],
                             [[0,0,0],[1,0,0,0]]];
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
        user_coord_template = f"[ {_XYZ}, {_QUAT} ]"
        user_coord = user_coord_template.format(x=x, y=y, z=z, q=q)
        object_coord = "[ [0, 0, 0], [1, 0, 0 ,0] ]"
        cls.rapid.write(
            f"PERS wobjdata {varname} := "
            f'[ {hold_str}, {fixed_str}, "", '
            f"{user_coord}, {object_coord} ];"
        )
        return WobjName(varname)


class Zone(RapidCommand):
    """Emit a `PERS zonedata` declaration; returns a typed `ZoneName`."""

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
        """Args:
        name: variable name
        finepoint: True → stop point (no blending), False → fly-by zone
        pzone_tcp: TCP zone radius (mm)
        pzone_ori: tool reorientation zone (mm)
        pzone_eax: external axes zone (mm)
        zone_ori: tool reorientation zone (degrees)
        zone_eax: linear external axes zone (mm)
        zone_reax: rotating external axes zone (degrees)
        """
        finepoint_str = get_rapid_bool(finepoint)
        cls.rapid.write(
            f"PERS zonedata {name} := "
            f"[{finepoint_str}, {pzone_tcp}, {pzone_ori}, {pzone_eax}, "
            f"{zone_ori}, {zone_eax}, {zone_reax}];"
        )
        return ZoneName(name)


class Speed(RapidCommand):
    """Emit a `PERS speeddata` declaration; returns a typed `SpeedName`."""

    def __new__(
        cls,
        name: str,
        v_tcp: float = 200.0,
        v_ori: float = 200.0,
        v_leax: float = 1000.0,
        v_reax: float = 1000.0,
    ) -> SpeedName:
        cls.rapid.write(f"PERS speeddata {name} := [ {v_tcp}, {v_ori}, {v_leax}, {v_reax} ];")
        return SpeedName(name)


class Pos(RapidCommand):
    """Emit a `PERS pos` declaration (x/y/z position triple, no orientation)."""

    def __new__(cls, name: str, x: float, y: float, z: float) -> str:
        cls.rapid.write(f"PERS pos {name} := [ {x}, {y}, {z} ];")
        return name


class RapidBool(RapidCommand):
    """Emit a `PERS bool` declaration."""

    def __new__(cls, name: str, value: bool) -> str:
        value_str = str(bool(value)).upper()
        # TODO: track declared names so AssignVariable can validate references.
        cls.rapid.write(f"PERS bool {name} := {value_str};")
        return name


class AssignVariable(RapidCommand):
    """`<name> := <value>;` — assign to an already-declared RAPID variable."""

    # TODO: track declared names so this can raise on undeclared assignment.
    def __new__(cls, name: str, value: object) -> str:
        cls.rapid.write(f"{name} := {value};")
        return name


# `ExternalAxis` is defined near `RapidTarget` above (it's a `RapidTarget` field).
