"""``KrlBackend`` — IR event stream → self-contained KUKA KRL ``.src``.

One instance per emit (no shared state). Tracks ``$VEL.CP`` so it is emitted
only on change. Joint moves and FREESPACE Cartesian → ``PTP``; LINEAR → ``LIN``.
KRL status/turn (S/T) are optional and omitted in v1, so Cartesian moves need
no robot configuration.
"""

from __future__ import annotations

from collections.abc import Mapping

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
from ..core.naming import safe_identifier
from ..core.units import MmPerSec, to_m_per_s
from .profile import KrlProfile
from .targets import krl_axis_literal, krl_frame_literal

_KRL_NAME_MAX = 24
_VEL_CP_DECIMALS = 5


class KrlBackend:
    """Stateful KRL emitter consumed by ``core.drive``."""

    def __init__(self, *, profiles: Mapping[str, KrlProfile], program_name: str) -> None:
        self._profiles = profiles
        self._name = safe_identifier(program_name, brand="KRL", max_len=_KRL_NAME_MAX)
        self._lines: list[str] = []
        self._current_vel_cp: float | None = None

    def _line(self, text: str) -> None:
        self._lines.append(f"  {text}")

    def _profile(self, name: str) -> KrlProfile:
        try:
            return self._profiles[name]
        except KeyError:
            raise MissingProfileError(name, list(self._profiles.keys())) from None

    def _set_vel_cp(self, cp_speed_mms: float) -> None:
        m_per_s = to_m_per_s(MmPerSec(cp_speed_mms))
        if self._current_vel_cp != m_per_s:
            self._line(f"$VEL.CP = {m_per_s:.{_VEL_CP_DECIMALS}f}")
            self._current_vel_cp = m_per_s

    # ProgramBackend protocol -------------------------------------------
    def prog_start(self, name: str) -> None:
        self._lines.append(f"DEF {self._name}()")

    def move_joint(self, m: JointMove) -> None:
        profile = self._profile(m.profile)
        self._line(f"BAS(#VEL_PTP, {profile.ptp_speed_percent:g})")
        self._line(f"PTP {krl_axis_literal(m.joints)}")

    def move_cartesian(self, m: CartesianMove) -> None:
        profile = self._profile(m.profile)
        if m.kind is MoveKind.LINEAR:
            self._set_vel_cp(profile.cp_speed_mms)
            self._line(f"LIN {krl_frame_literal(m.pose)}")
        else:
            self._line(f"PTP {krl_frame_literal(m.pose)}")

    def dwell(self, e: Dwell) -> None:
        self._line(f"WAIT SEC {e.seconds:g}")

    def wait_digital(self, e: WaitDigital) -> None:
        signal = "$IN" if e.is_input else "$OUT"
        self._line(f"WAIT FOR ({signal}[{e.index}] == {'TRUE' if e.value else 'FALSE'})")

    def set_digital(self, e: SetDigital) -> None:
        self._line(f"$OUT[{e.index}] = {'TRUE' if e.value else 'FALSE'}")

    def set_analog(self, e: SetAnalog) -> None:
        self._line(f"$ANOUT[{e.index}] = {e.value:g}")

    def tool_change(self, e: ToolChange) -> None:
        self._line(f"; tool change to tool id {e.tool_id}")

    def note(self, e: Note) -> None:
        self._line(f"; {e.text}")

    def prog_finish(self) -> dict[str, str]:
        self._lines.append("END")
        return {f"{self._name}.src": "\n".join(self._lines) + "\n"}
