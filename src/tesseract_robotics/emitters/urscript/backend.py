"""``UrScriptBackend`` — consume the core IR, emit URScript via the ``urscript_writer`` DSL.

The twin of ``emitters.krl.backend.KrlBackend``: it does not build strings itself
— it drives the brand DSL (``Def`` / ``MoveJ`` / ``MoveL`` / ``SetDigitalOut`` …),
exactly as ``KrlBackend`` drives the KRL DSL. State it owns: the open ``Def``
scope. Joint moves → ``movej``; LINEAR Cartesian → ``movel``; FREESPACE Cartesian
→ ``movej(get_inverse_kin(p[…]))`` (joint-interpolated to a Cartesian target).
UR poses carry full orientation (rotation vector), so Cartesian moves need no
separate robot configuration.
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
from .profile import UrScriptProfile
from .targets import joints_literal, pose_literal
from .urscript_writer import (
    Comment,
    Def,
    MoveJ,
    MoveJPose,
    MoveL,
    SetDigitalOut,
    Sleep,
    UrScriptWriter,
    WaitDigitalIn,
    WaitDigitalOut,
)

#: URScript identifier length cap. UR identifiers are generous; 32 is comfortable.
_URSCRIPT_NAME_MAX = 32


class UrScriptBackend(ProgramBackend):
    """Stateful URScript backend; drives the ``urscript_writer`` DSL (clears on start)."""

    def __init__(
        self,
        *,
        profiles: Mapping[str, UrScriptProfile],
        program_name: str,
        identity: EmitIdentity | None = None,
    ) -> None:
        self._profiles = profiles
        self._name = safe_identifier(program_name, brand="URScript", max_len=_URSCRIPT_NAME_MAX)
        self._identity = identity
        self._writer = UrScriptWriter()
        self._def: Def | None = None

    def _profile(self, name: str) -> UrScriptProfile:
        try:
            return self._profiles[name]
        except KeyError:
            raise MissingProfileError(name, list(self._profiles.keys())) from None

    # ProgramBackend protocol -------------------------------------------
    def prog_start(self) -> None:
        self._writer.clear()
        self._def = Def(self._name)
        self._def.__enter__()
        if self._identity is not None:
            for line in self._identity.header_lines():
                Comment(line)

    def move_joint(self, m: JointMove) -> None:
        profile = self._profile(m.profile)
        MoveJ(
            joints_literal(m.joints),
            accel=profile.joint_accel_radss,
            speed=profile.joint_speed_rads,
        )

    def move_cartesian(self, m: CartesianMove) -> None:
        profile = self._profile(m.profile)
        if m.kind is MoveKind.LINEAR:
            MoveL(
                pose_literal(m.pose),
                profile.linear_accel_mss,
                profile.linear_speed_ms,
                profile.blend_radius_m,
            )
        else:  # FREESPACE: joint-interpolate to the Cartesian target via on-controller IK
            MoveJPose(
                pose_literal(m.pose),
                accel=profile.joint_accel_radss,
                speed=profile.joint_speed_rads,
            )

    def dwell(self, e: Dwell) -> None:
        Sleep(e.seconds)

    def wait_digital(self, e: WaitDigital) -> None:
        if e.is_input:
            WaitDigitalIn(e.index, e.value)
        else:
            WaitDigitalOut(e.index, e.value)

    def set_digital(self, e: SetDigital) -> None:
        SetDigitalOut(e.index, e.value)

    def set_analog(self, e: SetAnalog) -> None:
        Comment(f"set analog output {e.index} to {e.value:g}")

    def tool_change(self, e: ToolChange) -> None:
        Comment(f"tool change to tool id {e.tool_id}")

    def note(self, e: Note) -> None:
        Comment(e.text)

    def prog_finish(self) -> dict[str, str]:
        assert self._def is not None
        self._def.__exit__(None, None, None)
        return {f"{self._name}.script": self._writer.getvalue() + "\n"}
