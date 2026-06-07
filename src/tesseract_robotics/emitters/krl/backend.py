"""``KrlBackend`` — consume the core IR, emit KRL via the ``krl_writer`` DSL.

The twin of ``emitters.rapid.backend.RapidBackend``: it does not build strings
itself — it drives the brand DSL (``Def`` / ``Ptp`` / ``Lin`` / ``SetOut`` …),
exactly as ``RapidBackend`` drives the RAPID DSL. State it owns: the current
``$VEL.CP`` (so the assignment is emitted only on change) and the open ``Def``
scope. Joint moves and FREESPACE Cartesian → ``PTP``; LINEAR → ``LIN``. KRL
status/turn (S/T) are optional and omitted in v1, so Cartesian moves need no
robot configuration.
"""

from __future__ import annotations

from collections.abc import Mapping

from ..core.backend import ProgramBackend
from ..core.errors import MissingProfileError
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
from ..core.identity import EmitIdentity
from ..core.naming import safe_identifier
from ..core.units import MmPerSec, to_m_per_s
from .krl_writer import (
    BasVelPtp,
    Comment,
    Def,
    KrlWriter,
    Lin,
    Ptp,
    SetAnOut,
    SetOut,
    VelCp,
    WaitFor,
    WaitSec,
)
from .profile import KrlProfile
from .targets import krl_axis_literal, krl_frame_literal

#: KRL identifier length cap (KSS).
_KRL_NAME_MAX = 24


class KrlBackend(ProgramBackend):
    """Stateful KRL backend; drives the ``krl_writer`` DSL (clears it on start)."""

    def __init__(
        self,
        *,
        profiles: Mapping[str, KrlProfile],
        program_name: str,
        identity: EmitIdentity | None = None,
    ) -> None:
        self._profiles = profiles
        self._name = safe_identifier(program_name, brand="KRL", max_len=_KRL_NAME_MAX)
        self._identity = identity
        self._krl = KrlWriter()
        self._def: Def | None = None
        self._current_vel_cp: float | None = None

    def _profile(self, name: str) -> KrlProfile:
        try:
            return self._profiles[name]
        except KeyError:
            raise MissingProfileError(name, list(self._profiles.keys())) from None

    def _set_vel_cp(self, cp_speed_mms: float) -> None:
        m_per_s = to_m_per_s(MmPerSec(cp_speed_mms))
        if self._current_vel_cp != m_per_s:
            VelCp(m_per_s)
            self._current_vel_cp = m_per_s

    # ProgramBackend protocol -------------------------------------------
    def prog_start(self) -> None:
        self._krl.clear()
        self._def = Def(self._name)
        self._def.__enter__()
        if self._identity is not None:
            for line in self._identity.header_lines():
                Comment(line)

    def move_joint(self, m: JointMove) -> None:
        BasVelPtp(self._profile(m.profile).ptp_speed_percent)
        Ptp(krl_axis_literal(m.joints))

    def move_cartesian(self, m: CartesianMove) -> None:
        profile = self._profile(m.profile)
        if m.kind is MoveKind.LINEAR:
            self._set_vel_cp(profile.cp_speed_mms)
            Lin(krl_frame_literal(m.pose))
        else:
            Ptp(krl_frame_literal(m.pose))

    def dwell(self, e: Dwell) -> None:
        WaitSec(e.seconds)

    def wait_digital(self, e: WaitDigital) -> None:
        WaitFor(e.index, e.value, is_input=e.is_input)

    def set_digital(self, e: SetDigital) -> None:
        SetOut(e.index, e.value)

    def set_analog(self, e: SetAnalog) -> None:
        SetAnOut(e.index, e.value)

    def tool_change(self, e: ToolChange) -> None:
        Comment(f"tool change to tool id {e.tool_id}")

    def note(self, e: Note) -> None:
        Comment(e.text)

    def prog_finish(self) -> dict[str, str]:
        assert self._def is not None
        self._def.__exit__(None, None, None)
        return {f"{self._name}.src": self._krl.getvalue() + "\n"}
