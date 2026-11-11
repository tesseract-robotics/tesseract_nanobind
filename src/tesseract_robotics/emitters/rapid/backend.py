"""`RapidBackend`: consume the core IR, emit RAPID through the `rapid_writer` DSL.

It owns three RAPID decisions:

- signal naming: a `SetDigital` with `key is None` renders `do<index>` (the
  Timer convention); a named key renders `key` (index 0) or `key{index}`
  (array subscript); every name is validated as a RAPID identifier;
- frames: each move's tool and work object come from the frame binding, by the
  move's `tcp_frame` / `working_frame`;
- configuration: every Cartesian target carries exact confdata, from the
  waypoint's seed joints (`cf1`, `cf4`, `cf6`) and the posture solver (`cfx`).
  Without a seed or a solver the move raises `MissingConfigurationError`.
"""

from __future__ import annotations

from collections.abc import Mapping
from contextlib import AbstractContextManager

from ..core.backend import ProgramBackend
from ..core.errors import (
    ExternalAxisError,
    MissingConfigurationError,
    MissingProfileError,
    UnsupportedInstructionError,
)
from ..core.events import (
    CartesianMove,
    Dwell,
    JointMove,
    MoveKind,
    Note,
    SetAnalog,
    SetDigital,
    ToolChange,
    WaitDigital,
)
from ..core.frames import FrameBinding, MoveFrames
from ..core.identity import EmitIdentity
from ..core.posture import PostureSolver
from .profile import RapidProfile, ToolName, WobjName
from .rapid_writer import (
    Comment,
    Module,
    MoveAbsJ,
    MoveJ,
    MoveL,
    Proc,
    RapidWriter,
    SetAO,
    SetDO,
    WaitDI,
    WaitDO,
    WaitTime,
)
from .syntax import BRAND, RAPID_IDENTIFIERS
from .targets import Config, ExternalAxis, JointTarget, RapidTarget

#: Robot axes in a robtarget/jointtarget.
_ROBAX = 6


def _signal(prefix: str, key: str | None, index: int) -> str:
    """RAPID signal name: `<prefix><index>` unnamed, `key` or `key{index}` named."""
    if key is None:
        return RAPID_IDENTIFIERS.validate(f"{prefix}{index}")
    name = RAPID_IDENTIFIERS.validate(key)
    return name if index == 0 else f"{name}{{{index}}}"


class RapidBackend(ProgramBackend):
    """Stateful RAPID backend; writes one `MODULE` with one `PROC` into its own writer."""

    def __init__(
        self,
        *,
        profiles: Mapping[str, RapidProfile],
        frames: FrameBinding[WobjName, ToolName],
        posture: PostureSolver | None,
        module_name: str,
        proc_name: str,
        identity: EmitIdentity,
    ) -> None:
        self._profiles = profiles
        self._frames = frames
        self._posture = posture
        self._module_name = RAPID_IDENTIFIERS.validate(module_name)
        self._proc_name = RAPID_IDENTIFIERS.validate(proc_name)
        self._identity = identity
        self._rapid = RapidWriter()
        self._module: Module | None = None
        self._proc: Proc | None = None
        self._move_index = 0

    def _profile(self, name: str) -> RapidProfile:
        try:
            return self._profiles[name]
        except KeyError:
            raise MissingProfileError(name, list(self._profiles.keys())) from None

    def _tool(self, frames: MoveFrames) -> ToolName:
        frames.require_no_offset(BRAND)
        return self._frames.tool(frames, BRAND)

    # ProgramBackend protocol -------------------------------------------
    def session(self) -> AbstractContextManager[object]:
        return self._rapid

    def prog_start(self) -> None:
        self._module = Module(self._module_name)
        self._module.__enter__()
        for line in self._identity.header_lines():
            Comment(line)
        self._proc = Proc(self._proc_name)
        self._proc.__enter__()

    def move_joint(self, m: JointMove) -> None:
        self._move_index += 1
        if len(m.joints) != _ROBAX:
            raise ExternalAxisError(len(m.joints), _ROBAX, BRAND)
        target = JointTarget.build(m.joints, ExternalAxis.build(m.external_axes))
        MoveAbsJ(target, self._profile(m.profile), self._tool(m.frames))

    def move_cartesian(self, m: CartesianMove) -> None:
        self._move_index += 1
        profile = self._profile(m.profile)
        frames = m.target.frames
        tool = self._tool(frames)
        wobj = self._frames.work(frames, BRAND)
        target = RapidTarget(
            pose=m.target.pose,
            config=self._config(m),
            external_axis=ExternalAxis.build(m.external_axes),
        )
        if m.kind is MoveKind.LINEAR:
            MoveL(target, profile, tool, wobj)
        else:
            MoveJ(target, profile, tool, wobj)

    def _config(self, m: CartesianMove) -> Config:
        if m.seed_joints is None:
            raise MissingConfigurationError(
                self._move_index, BRAND, "the waypoint has no seed joints to derive confdata from"
            )
        if len(m.seed_joints) != _ROBAX:
            raise ExternalAxisError(len(m.seed_joints), _ROBAX, BRAND)
        if self._posture is None:
            raise MissingConfigurationError(
                self._move_index,
                BRAND,
                "cfx needs the arm posture; pass emit_rapid(posture=...) "
                "(e.g. OpwPostureSolver for the robot's OPW parameters)",
            )
        return Config.from_joints(m.seed_joints, self._posture.posture(m.seed_joints))

    def dwell(self, e: Dwell) -> None:
        WaitTime(e.seconds)

    def wait_digital(self, e: WaitDigital) -> None:
        signal = _signal("di" if e.is_input else "do", None, e.index)
        (WaitDI if e.is_input else WaitDO)(signal, value=e.value)

    def set_digital(self, e: SetDigital) -> None:
        SetDO(_signal("do", e.key, e.index), e.value)

    def set_analog(self, e: SetAnalog) -> None:
        SetAO(_signal("ao", e.key, e.index), e.value)

    def tool_change(self, e: ToolChange) -> None:
        raise UnsupportedInstructionError(
            f"RAPID: SetTool({e.tool_id}) has no RAPID equivalent; the tool is an argument of "
            "every move, taken from the move's tcp_frame through the frame binding"
        )

    def note(self, e: Note) -> None:
        Comment(e.text)

    def prog_finish(self) -> dict[str, str]:
        assert self._proc is not None and self._module is not None
        self._proc.__exit__(None, None, None)
        self._module.__exit__(None, None, None)
        return {f"{self._module_name}.mod": self._rapid.getvalue()}
