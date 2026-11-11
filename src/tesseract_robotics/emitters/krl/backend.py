"""`KrlBackend`: consume the core IR, emit KRL through the `krl_writer` DSL.

State it owns, each emitted only on change: the active `$BASE` / `$TOOL` (from
each move's frames through the frame binding), `$VEL.CP`, the PTP velocity and
`$APO.CDIS`.

Motion mapping:

- joint moves become `PTP {A1 .. A6}`;
- LINEAR Cartesian moves become `LIN {X .. C}`. KSS 8.3 section 9.10: "The robot
  controller only takes the programmed Status and Turn values into consideration
  for PTP motions. They are ignored for CP motions", so a frame without S/T is
  exact for `LIN`;
- FREESPACE Cartesian moves would be `PTP` to a frame, where Status and Turn
  select the configuration. KSS 8.3 defines Status bit 2 by the direction A5 is
  tilted relative to the A4 zero position, which the plan's joint values do not
  determine without the robot's wrist kinematics; the move raises
  `MissingConfigurationError` rather than let the controller pick a branch.
"""

from __future__ import annotations

from collections.abc import Mapping
from contextlib import AbstractContextManager

from ..core.backend import ProgramBackend
from ..core.errors import (
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
from ..core.units import MmPerSec, to_m_per_s
from .krl_writer import (
    ApoCdis,
    BasVelPtp,
    Comment,
    Def,
    KrlWriter,
    Lin,
    Ptp,
    SetAnOut,
    SetBase,
    SetOut,
    SetTool,
    VelCp,
    WaitFor,
    WaitSec,
)
from .profile import KrlProfile
from .syntax import BRAND, KRL_IDENTIFIERS, KrlFrameRef, frame_expr
from .targets import krl_axis_literal, krl_frame_literal


def _numbered_only(key: str | None, what: str) -> None:
    if key is not None:
        raise UnsupportedInstructionError(
            f"KRL: {what} {key!r} is a named signal; KRL IO is addressed by number "
            "($OUT[i], $ANOUT[i]), so a key cannot be mapped without guessing"
        )


class KrlBackend(ProgramBackend):
    """Stateful KRL backend; writes one `DEF` into its own writer."""

    def __init__(
        self,
        *,
        profiles: Mapping[str, KrlProfile],
        frames: FrameBinding[KrlFrameRef, KrlFrameRef],
        program_name: str,
        identity: EmitIdentity,
    ) -> None:
        self._profiles = profiles
        self._frames = frames
        self._name = KRL_IDENTIFIERS.validate(program_name)
        self._identity = identity
        self._krl = KrlWriter()
        self._def: Def | None = None
        self._move_index = 0
        self._base: KrlFrameRef | None = None
        self._tool: KrlFrameRef | None = None
        self._vel_cp: float | None = None
        self._vel_ptp: float | None = None
        self._cdis: float | None = None

    def _profile(self, name: str) -> KrlProfile:
        try:
            return self._profiles[name]
        except KeyError:
            raise MissingProfileError(name, list(self._profiles.keys())) from None

    def _select_tool(self, frames: MoveFrames) -> None:
        frames.require_no_offset(BRAND)
        tool = self._frames.tool(frames, BRAND)
        if tool != self._tool:
            SetTool(frame_expr(tool, "TOOL_DATA"))
            self._tool = tool

    def _select_base(self, frames: MoveFrames) -> None:
        base = self._frames.work(frames, BRAND)
        if base != self._base:
            SetBase(frame_expr(base, "BASE_DATA"))
            self._base = base

    def _approximation(self, profile: KrlProfile) -> bool:
        if profile.approximation_mm is None:
            return False
        if profile.approximation_mm != self._cdis:
            ApoCdis(profile.approximation_mm)
            self._cdis = profile.approximation_mm
        return True

    # ProgramBackend protocol -------------------------------------------
    def session(self) -> AbstractContextManager[object]:
        return self._krl

    def prog_start(self) -> None:
        self._def = Def(self._name)
        self._def.__enter__()
        for line in self._identity.header_lines():
            Comment(line)

    def move_joint(self, m: JointMove) -> None:
        self._move_index += 1
        profile = self._profile(m.profile)
        target = krl_axis_literal(m.joints)
        self._select_tool(m.frames)
        if profile.ptp_speed_percent != self._vel_ptp:
            BasVelPtp(profile.ptp_speed_percent)
            self._vel_ptp = profile.ptp_speed_percent
        Ptp(target, c_dis=self._approximation(profile))

    def move_cartesian(self, m: CartesianMove) -> None:
        self._move_index += 1
        profile = self._profile(m.profile)
        if m.kind is not MoveKind.LINEAR:
            raise MissingConfigurationError(
                self._move_index,
                BRAND,
                "a FREESPACE Cartesian move is a PTP to a frame, whose Status bit 2 depends "
                "on the A5 tilt relative to the A4 zero position; plan it as a joint waypoint",
            )
        frames = m.target.frames
        self._select_tool(frames)
        self._select_base(frames)
        m_per_s = to_m_per_s(MmPerSec(profile.cp_speed_mms))
        if m_per_s != self._vel_cp:
            VelCp(m_per_s)
            self._vel_cp = m_per_s
        Lin(krl_frame_literal(m.target.pose), c_dis=self._approximation(profile))

    def dwell(self, e: Dwell) -> None:
        WaitSec(e.seconds)

    def wait_digital(self, e: WaitDigital) -> None:
        WaitFor(e.index, e.value, is_input=e.is_input)

    def set_digital(self, e: SetDigital) -> None:
        _numbered_only(e.key, "digital output")
        SetOut(e.index, e.value)

    def set_analog(self, e: SetAnalog) -> None:
        _numbered_only(e.key, "analog output")
        SetAnOut(e.index, e.value)

    def tool_change(self, e: ToolChange) -> None:
        raise UnsupportedInstructionError(
            f"KRL: SetTool({e.tool_id}) conflicts with frame-bound tools; $TOOL follows each "
            "move's tcp_frame through the frame binding"
        )

    def note(self, e: Note) -> None:
        Comment(e.text)

    def prog_finish(self) -> dict[str, str]:
        assert self._def is not None
        self._def.__exit__(None, None, None)
        return {f"{self._name}.src": self._krl.getvalue() + "\n"}
