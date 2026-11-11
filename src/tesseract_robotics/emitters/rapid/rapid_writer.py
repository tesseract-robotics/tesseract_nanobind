"""ABB RAPID code-generation DSL: class-per-statement with `with`-block scopes.

Every `RapidCommand` writes into the active `RapidWriter` on instantiation;
`with Module(): with Proc(): ...` blocks indent the buffer.

```python
with RapidWriter() as rapid:
    with Module("WELD"):
        p1 = Robtarget("p1", RapidTarget(pose, config))
        with Proc("main"):
            MoveL(p1, profile, tool0, wobj0)
text = rapid.getvalue()
```

The writer enforces the RAPID program structure (3HAC050946, RAPID kernel,
sections on modules, routines and data declarations) so the DSL cannot produce a
module the controller rejects:

- `MODULE` only at top level, never nested;
- `PROC` only directly inside a `MODULE` (routines do not nest);
- statements (moves, IO, `WHILE`, `FOR`, assignments, `Stop`) only inside a routine;
- `PERS` declarations only at module level;
- `VAR` / `CONST` declarations at module level, or in a routine before its first
  statement;
- every declaration names its data type (`robtarget`, `tooldata`, ...).

Violations raise `ProgramStructureError`. Declarations are objects carrying
their typed name (`Robtarget(...).name`, `Speed(...).name`), never bare strings.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from enum import Enum
from typing import ClassVar

from tesseract_robotics.planning.transforms import Pose

from ..core.dsl import Block, Command, Writer
from ..core.errors import (
    InvalidDeclarationError,
    ProgramStructureError,
    UndeclaredVariableError,
)
from ..core.precision import DEG_DECIMALS, MM_DECIMALS, fixed
from ..core.units import (
    Metres,
    MPerSec,
    Radians,
    RadPerSec,
    to_deg_per_s,
    to_degrees,
    to_millimetres,
    to_mm_per_s,
)
from .profile import (
    _DEFAULT_PROFILE,
    RapidProfile,
    SpeedName,
    ToolName,
    WobjName,
    ZoneName,
    tool0,
    wobj0,
)
from .syntax import (
    BRAND,
    RAPID_IDENTIFIERS,
    MotionType,
    RapidType,
    comment_text,
    rapid_bool,
    string_literal,
)
from .targets import JointTarget, RapidTarget, move_line, movec_line, pose_literal


class _Scope(Enum):
    MODULE = "MODULE"
    PROC = "PROC"
    WHILE = "WHILE"
    FOR = "FOR"


@dataclass
class _Frame:
    """One open scope plus whether a statement has been written directly in it."""

    kind: _Scope
    statements_started: bool = False


@dataclass
class _Declared:
    """A declared variable: its storage class and RAPID data type."""

    storage: RapidType
    data_type: str


class RapidWriter(Writer):
    """RAPID code buffer: 4-space indent, leading newline, structural checks.

    Tracks the open scopes and every declared variable so the DSL classes can
    reject statements the RAPID grammar does not allow where they are written.
    """

    _tab: ClassVar[str] = "    "
    _leading_newline: ClassVar[bool] = True

    def __init__(self) -> None:
        super().__init__()
        self._scopes: list[_Frame] = []
        self._declared: dict[str, _Declared] = {}

    # Scope tracking --------------------------------------------------------
    def _scope_names(self) -> list[str]:
        return [f.kind.value for f in self._scopes]

    def _fail(self, what: str) -> ProgramStructureError:
        return ProgramStructureError(BRAND, what, self._scope_names())

    def _in_routine(self) -> bool:
        return any(f.kind is _Scope.PROC for f in self._scopes)

    def open_scope(self, kind: _Scope) -> None:
        """Check `kind` may open here, then push it."""
        top = self._scopes[-1].kind if self._scopes else None
        if kind is _Scope.MODULE and top is not None:
            raise self._fail("MODULE")
        if kind is _Scope.PROC and top is not _Scope.MODULE:
            raise self._fail("PROC")
        if kind in (_Scope.WHILE, _Scope.FOR):
            self.check_statement(kind.value)
        self._scopes.append(_Frame(kind))

    def close_scope(self) -> None:
        self._scopes.pop()

    def check_statement(self, what: str) -> None:
        """Raise unless a statement may be written here; mark the routine started."""
        if not self._in_routine():
            raise self._fail(what)
        for frame in self._scopes:
            if frame.kind is _Scope.PROC:
                frame.statements_started = True

    def check_declaration(self, storage: RapidType, data_type: str, name: str) -> None:
        """Raise unless a `storage data_type name` declaration may be written here."""
        what = f"{storage} {data_type} {name}"
        if not self._scopes:
            raise self._fail(what)
        top = self._scopes[-1]
        if top.kind is _Scope.MODULE:
            pass
        elif top.kind is _Scope.PROC and storage is not RapidType.PERS:
            if top.statements_started:
                raise ProgramStructureError(
                    BRAND, f"{what} after the routine's first statement", self._scope_names()
                )
        else:
            raise self._fail(what)
        key = name.upper()
        if key in self._declared:
            raise ProgramStructureError(
                BRAND, f"second declaration of {name!r}", self._scope_names()
            )
        self._declared[key] = _Declared(storage, data_type)

    def declared(self, name: str) -> _Declared | None:
        return self._declared.get(name.upper())


class RapidCommand(Command):
    """Base class for every RAPID DSL statement; writes into the active `RapidWriter`."""

    writer_type: ClassVar[type[Writer]] = RapidWriter

    @property
    def rapid(self) -> RapidWriter:
        return RapidWriter.current()


class RapidStatement(RapidCommand):
    """A statement: only valid inside a routine."""

    def _emit(self, what: str, line: str) -> None:
        self.rapid.check_statement(what)
        self.rapid.write(line)


# ---------------------------------------------------------------------------
# Scopes.
# ---------------------------------------------------------------------------


class _RapidBlock(RapidCommand, Block):
    _kind: ClassVar[_Scope]

    def __init__(self, opening: str, closing: str) -> None:
        self.rapid.open_scope(self._kind)
        Block.__init__(self, opening, closing)

    def __exit__(self, *exc: object) -> None:
        Block.__exit__(self, *exc)
        self.rapid.close_scope()


class Module(_RapidBlock):
    """`MODULE <name> ... ENDMODULE` (top level only)."""

    _kind = _Scope.MODULE

    def __init__(self, name: str) -> None:
        super().__init__(f"MODULE {RAPID_IDENTIFIERS.validate(name)}", "ENDMODULE\n")


class Proc(_RapidBlock):
    """`PROC <name>() ... ENDPROC` (directly inside a `MODULE`)."""

    _kind = _Scope.PROC

    def __init__(self, name: str) -> None:
        super().__init__(f"PROC {RAPID_IDENTIFIERS.validate(name)}()", "ENDPROC")


class While(_RapidBlock):
    """`WHILE <condition> DO ... ENDWHILE` (inside a routine)."""

    _kind = _Scope.WHILE

    def __init__(self, condition: str) -> None:
        super().__init__(f"WHILE {condition} DO", "ENDWHILE")


class For(_RapidBlock):
    """`FOR <i> FROM <start> TO <end> DO ... ENDFOR` (inside a routine)."""

    _kind = _Scope.FOR

    def __init__(self, loop_counter: str, start: int, end: int) -> None:
        counter = RAPID_IDENTIFIERS.validate(loop_counter)
        super().__init__(f"FOR {counter} FROM {start} TO {end} DO", "ENDFOR")


# ---------------------------------------------------------------------------
# Declarations: objects carrying their typed name.
# ---------------------------------------------------------------------------


class Declaration(RapidCommand):
    """Base of every data declaration; `name` is the declared variable."""

    data_type: ClassVar[str]
    name: str
    storage: RapidType

    def _declare(self, name: str, storage: RapidType, value: str) -> None:
        RAPID_IDENTIFIERS.validate(name)
        self.rapid.check_declaration(storage, self.data_type, name)
        self.storage = storage
        self.rapid.write(f"{storage} {self.data_type} {name} := {value};")


class Robtarget(Declaration):
    """`<storage> robtarget <name> := [...];` Default storage `CONST`.

    ```rapid
    CONST robtarget p1 := [[600.000, -100.000, 800.000],[...],[0, 0, 0, 1],[9E+09,...]];
    ```
    """

    data_type = "robtarget"

    def __init__(
        self, name: str, target: RapidTarget, storage: RapidType = RapidType.CONST
    ) -> None:
        self._declare(name, storage, target.literal())
        self.name = name


class Jointtarget(Declaration):
    """`<storage> jointtarget <name> := [...];` Default storage `CONST`."""

    data_type = "jointtarget"

    def __init__(
        self, name: str, target: JointTarget, storage: RapidType = RapidType.CONST
    ) -> None:
        self._declare(name, storage, target.literal())
        self.name = name


class Tooldata(Declaration):
    """`PERS tooldata <name> := [robhold, tframe, tload];` (always `PERS`).

    Args:
        name: Variable name.
        tframe: TCP pose in the flange frame (metres, scalar-last quaternion).
        mass_kg: Tool mass (kg), > 0.
        cog_m: Centre of gravity in the flange frame (metres).
        robhold: Whether the robot holds the tool (`TRUE`) or it is stationary.

    ```rapid
    PERS tooldata gun := [TRUE, [[0.000, 0.000, 200.000],[1.000000000, ...]], [2.5, [0.000, 0.000, 100.000], [1, 0, 0, 0], 0, 0, 0]];
    ```
    """

    data_type = "tooldata"

    def __init__(
        self,
        name: str,
        tframe: Pose,
        mass_kg: float,
        cog_m: Sequence[float],
        *,
        robhold: bool = True,
    ) -> None:
        if not mass_kg > 0.0:
            raise InvalidDeclarationError(f"tooldata {name}: mass_kg must be > 0; got {mass_kg}")
        if len(cog_m) != 3:
            raise InvalidDeclarationError(f"tooldata {name}: cog_m must be 3 values; got {cog_m}")
        cog = ", ".join(fixed(to_millimetres(Metres(float(c))), MM_DECIMALS) for c in cog_m)
        tload = f"[{mass_kg}, [{cog}], [1, 0, 0, 0], 0, 0, 0]"
        self._declare(
            name, RapidType.PERS, f"[{rapid_bool(robhold)}, {pose_literal(tframe)}, {tload}]"
        )
        self.name = ToolName(name)


class Workobject(Declaration):
    """`PERS wobjdata <name> := [robhold, ufprog, ufmec, uframe, oframe];` (always `PERS`).

    Args:
        name: Variable name.
        uframe: User frame in the world frame (metres, scalar-last quaternion).
        oframe: Object frame in the user frame (identity by default).
        robhold: Whether the robot holds the work object.
        ufprog: Whether the user frame is fixed (`TRUE`) or moved by a mechanical unit.
    """

    data_type = "wobjdata"

    def __init__(
        self,
        name: str,
        uframe: Pose,
        oframe: Pose | None = None,
        *,
        robhold: bool = False,
        ufprog: bool = True,
    ) -> None:
        obj = oframe if oframe is not None else Pose()
        value = (
            f'[{rapid_bool(robhold)}, {rapid_bool(ufprog)}, "", '
            f"{pose_literal(uframe)}, {pose_literal(obj)}]"
        )
        self._declare(name, RapidType.PERS, value)
        self.name = WobjName(name)


class Zone(Declaration):
    """`<storage> zonedata <name> := [finep, pzone_tcp, pzone_ori, pzone_eax, zone_ori, zone_leax, zone_reax];`

    SI arguments, converted at the writer: lengths in metres (printed mm),
    angles in radians (printed deg). Default storage `CONST`.
    """

    data_type = "zonedata"

    def __init__(
        self,
        name: str,
        *,
        finepoint: bool,
        pzone_tcp_m: float,
        pzone_ori_m: float,
        pzone_eax_m: float,
        zone_ori_rad: float,
        zone_leax_m: float,
        zone_reax_rad: float,
        storage: RapidType = RapidType.CONST,
    ) -> None:
        values = (pzone_tcp_m, pzone_ori_m, pzone_eax_m, zone_ori_rad, zone_leax_m, zone_reax_rad)
        if any(v < 0.0 for v in values):
            raise InvalidDeclarationError(f"zonedata {name}: zone sizes must be >= 0; got {values}")
        mm = [
            fixed(to_millimetres(Metres(v)), MM_DECIMALS)
            for v in (pzone_tcp_m, pzone_ori_m, pzone_eax_m)
        ]
        ori = fixed(to_degrees(Radians(zone_ori_rad)), DEG_DECIMALS)
        leax = fixed(to_millimetres(Metres(zone_leax_m)), MM_DECIMALS)
        reax = fixed(to_degrees(Radians(zone_reax_rad)), DEG_DECIMALS)
        value = f"[{rapid_bool(finepoint)}, {', '.join(mm)}, {ori}, {leax}, {reax}]"
        self._declare(name, storage, value)
        self.name = ZoneName(name)


class Speed(Declaration):
    """`<storage> speeddata <name> := [v_tcp, v_ori, v_leax, v_reax];`

    SI arguments, converted at the writer: m/s (printed mm/s) and rad/s
    (printed deg/s). Default storage `CONST`.
    """

    data_type = "speeddata"

    def __init__(
        self,
        name: str,
        *,
        tcp_m_per_s: float,
        ori_rad_per_s: float,
        leax_m_per_s: float,
        reax_rad_per_s: float,
        storage: RapidType = RapidType.CONST,
    ) -> None:
        values = (tcp_m_per_s, ori_rad_per_s, leax_m_per_s, reax_rad_per_s)
        if any(not v > 0.0 for v in values):
            raise InvalidDeclarationError(f"speeddata {name}: speeds must be > 0; got {values}")
        tcp = fixed(to_mm_per_s(MPerSec(tcp_m_per_s)), MM_DECIMALS)
        ori = fixed(to_deg_per_s(RadPerSec(ori_rad_per_s)), DEG_DECIMALS)
        leax = fixed(to_mm_per_s(MPerSec(leax_m_per_s)), MM_DECIMALS)
        reax = fixed(to_deg_per_s(RadPerSec(reax_rad_per_s)), DEG_DECIMALS)
        self._declare(name, storage, f"[{tcp}, {ori}, {leax}, {reax}]")
        self.name = SpeedName(name)


class Pos(Declaration):
    """`<storage> pos <name> := [x, y, z];` from metres (printed mm). Default `CONST`."""

    data_type = "pos"

    def __init__(
        self, name: str, xyz_m: Sequence[float], storage: RapidType = RapidType.CONST
    ) -> None:
        if len(xyz_m) != 3:
            raise InvalidDeclarationError(f"pos {name}: xyz_m must be 3 values; got {xyz_m}")
        mm = ", ".join(fixed(to_millimetres(Metres(float(v))), MM_DECIMALS) for v in xyz_m)
        self._declare(name, storage, f"[{mm}]")
        self.name = name


class RapidBool(Declaration):
    """`<storage> bool <name> := TRUE|FALSE;` Default storage `VAR`."""

    data_type = "bool"

    def __init__(self, name: str, value: bool, storage: RapidType = RapidType.VAR) -> None:
        self._declare(name, storage, rapid_bool(value))
        self.name = name


# ---------------------------------------------------------------------------
# Statements.
# ---------------------------------------------------------------------------


def _target(t: RapidTarget | Robtarget) -> str:
    return t.name if isinstance(t, Robtarget) else t.literal()


class MoveJ(RapidStatement):
    """`MoveJ` (joint-interpolated) to a robtarget literal or a declared `Robtarget`."""

    def __init__(
        self,
        target: RapidTarget | Robtarget,
        profile: RapidProfile = _DEFAULT_PROFILE,
        tool: ToolName = tool0,
        wobj: WobjName = wobj0,
    ) -> None:
        self._emit("MoveJ", move_line("MoveJ", _target(target), profile, tool, wobj))


class MoveL(RapidStatement):
    """`MoveL` (straight line) to a robtarget literal or a declared `Robtarget`."""

    def __init__(
        self,
        target: RapidTarget | Robtarget,
        profile: RapidProfile = _DEFAULT_PROFILE,
        tool: ToolName = tool0,
        wobj: WobjName = wobj0,
    ) -> None:
        self._emit("MoveL", move_line("MoveL", _target(target), profile, tool, wobj))


class MoveC(RapidStatement):
    """`MoveC` (circular) through `via` to `end`; each a literal or a declared `Robtarget`."""

    def __init__(
        self,
        via: RapidTarget | Robtarget,
        end: RapidTarget | Robtarget,
        profile: RapidProfile = _DEFAULT_PROFILE,
        tool: ToolName = tool0,
        wobj: WobjName = wobj0,
    ) -> None:
        self._emit("MoveC", movec_line(_target(via), _target(end), profile, tool, wobj))


class MoveAbsJ(RapidStatement):
    """`MoveAbsJ <jointtarget>, speed, zone, tool;` (3HAC050917: `Tool` is mandatory).

    ```rapid
    MoveAbsJ [[0.0000, 90.0000, 0.0000, 0.0000, 0.0000, 0.0000], [9E+09,...]], v200, z10, tool0;
    ```
    """

    def __init__(
        self,
        target: JointTarget | Jointtarget,
        profile: RapidProfile = _DEFAULT_PROFILE,
        tool: ToolName = tool0,
    ) -> None:
        text = target.name if isinstance(target, Jointtarget) else target.literal()
        self._emit("MoveAbsJ", f"MoveAbsJ {text}, {profile.speed}, {profile.zone}, {tool};")


class Comment(RapidCommand):
    """`! <text>`: a line comment (allowed anywhere; never contains a newline)."""

    def __init__(self, text: str) -> None:
        self.rapid.write(f"! {comment_text(text)}")


class TPWrite(RapidStatement):
    """`TPWrite "<text>";`: write an escaped string (at most 80 characters) to the pendant."""

    def __init__(self, text: str) -> None:
        self._emit("TPWrite", f"TPWrite {string_literal(text)};")


class Stop(RapidStatement):
    """`Stop;`: halt program execution; resumable from the FlexPendant."""

    def __init__(self) -> None:
        self._emit("Stop", "Stop;")


class WaitTime(RapidStatement):
    """`WaitTime <seconds>;`"""

    def __init__(self, seconds: float) -> None:
        self._emit("WaitTime", f"WaitTime {seconds};")


class WaitDI(RapidStatement):
    """`WaitDI <signal>, 0|1;`: block until a digital input matches."""

    def __init__(self, signal: str, value: bool = True) -> None:
        self._emit("WaitDI", f"WaitDI {signal}, {1 if value else 0};")


class WaitDO(RapidStatement):
    """`WaitDO <signal>, 0|1;`: block until a digital output matches."""

    def __init__(self, signal: str, value: bool = True) -> None:
        self._emit("WaitDO", f"WaitDO {signal}, {1 if value else 0};")


class SetDO(RapidStatement):
    """`SetDO <signal>, 0|1;`: drive a digital output."""

    def __init__(self, signal: str, value: bool) -> None:
        self._emit("SetDO", f"SetDO {signal}, {1 if value else 0};")


class SetAO(RapidStatement):
    """`SetAO <signal>, <value>;`: drive an analog output."""

    def __init__(self, signal: str, value: float) -> None:
        self._emit("SetAO", f"SetAO {signal}, {value};")


class Conf(RapidStatement):
    """`ConfL \\On|\\Off;` or `ConfJ \\On|\\Off;`: configuration monitoring."""

    def __init__(self, motion: MotionType, on: bool) -> None:
        keyword = "L" if motion is MotionType.LINEAR else "J"
        switch = r"\On" if on else r"\Off"
        self._emit(f"Conf{keyword}", f"Conf{keyword} {switch};")


class AssignVariable(RapidStatement):
    """`<name> := <value>;` to a declared, non-`CONST` variable.

    Raises:
        UndeclaredVariableError: `target` is not declared in this writer, or is `CONST`.
    """

    def __init__(self, target: Declaration, value: str) -> None:
        declared = self.rapid.declared(target.name)
        if declared is None:
            raise UndeclaredVariableError(target.name, BRAND, "not declared in this program")
        if declared.storage is RapidType.CONST:
            raise UndeclaredVariableError(target.name, BRAND, "declared CONST")
        self._emit("assignment", f"{target.name} := {value};")
