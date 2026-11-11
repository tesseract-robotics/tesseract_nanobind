"""`UrScriptBackend`: consume the core IR, emit URScript through the `urscript_writer` DSL.

State it owns: the open `def` scope and the active TCP. Each move's TCP comes
from the frame binding (`tcps[tcp_frame]` composed with the move's
`tcp_offset`) and is emitted with `set_tcp` whenever it changes. Cartesian
targets are transformed exactly into the robot base frame with the bound feature
pose of their working frame.

Motion mapping: joint moves become `movej([...])`; LINEAR Cartesian moves
become `movel(p[...])` (the linear path fixes the configuration continuously);
FREESPACE Cartesian moves become `movej(get_inverse_kin(p[...], qnear=[seed]))`,
so the controller picks the IK solution closest to the plan's configuration. A
FREESPACE Cartesian move without seed joints raises `MissingConfigurationError`.
"""

from __future__ import annotations

from collections.abc import Mapping
from contextlib import AbstractContextManager

import numpy as np

from tesseract_robotics.planning.transforms import Pose

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
from .profile import UrScriptProfile
from .syntax import ANALOG_MAX, ANALOG_MIN, ANALOG_PORTS, BRAND, UR_IDENTIFIERS
from .targets import joints_literal, pose_literal
from .urscript_writer import (
    Comment,
    Def,
    MoveJ,
    MoveJPose,
    MoveL,
    SetAnalogOut,
    SetDigitalOut,
    SetTcp,
    Sleep,
    UrScriptWriter,
    WaitDigitalIn,
    WaitDigitalOut,
)


def _numbered_only(key: str | None, what: str) -> None:
    if key is not None:
        raise UnsupportedInstructionError(
            f"URScript: {what} {key!r} is a named signal; standard IO is addressed by "
            "number, so a key cannot be mapped without guessing"
        )


class UrScriptBackend(ProgramBackend):
    """Stateful URScript backend; writes one `def` into its own writer."""

    def __init__(
        self,
        *,
        profiles: Mapping[str, UrScriptProfile],
        frames: FrameBinding[Pose, Pose],
        program_name: str,
        identity: EmitIdentity,
    ) -> None:
        self._profiles = profiles
        self._frames = frames
        self._name = UR_IDENTIFIERS.validate(program_name)
        self._identity = identity
        self._writer = UrScriptWriter()
        self._def: Def | None = None
        self._move_index = 0
        self._tcp: str | None = None

    def _profile(self, name: str) -> UrScriptProfile:
        try:
            return self._profiles[name]
        except KeyError:
            raise MissingProfileError(name, list(self._profiles.keys())) from None

    def _select_tcp(self, frames: MoveFrames) -> None:
        tcp = Pose(
            np.asarray(self._frames.tool(frames, BRAND).matrix)
            @ np.asarray(frames.tcp_offset.matrix)
        )
        literal = pose_literal(tcp)
        if literal != self._tcp:
            SetTcp(literal)
            self._tcp = literal

    # ProgramBackend protocol -------------------------------------------
    def session(self) -> AbstractContextManager[object]:
        return self._writer

    def prog_start(self) -> None:
        self._def = Def(self._name)
        self._def.__enter__()
        for line in self._identity.header_lines():
            Comment(line)

    def move_joint(self, m: JointMove) -> None:
        self._move_index += 1
        profile = self._profile(m.profile)
        joints = joints_literal(m.joints)
        self._select_tcp(m.frames)
        MoveJ(joints, accel=profile.joint_accel_radss, speed=profile.joint_speed_rads)

    def move_cartesian(self, m: CartesianMove) -> None:
        self._move_index += 1
        profile = self._profile(m.profile)
        frames = m.target.frames
        feature = np.asarray(self._frames.work(frames, BRAND).matrix)
        target = pose_literal(Pose(feature @ np.asarray(m.target.pose.matrix)))
        if m.kind is MoveKind.LINEAR:
            self._select_tcp(frames)
            MoveL(target, profile.linear_accel_mss, profile.linear_speed_ms, profile.blend_radius_m)
            return
        if m.seed_joints is None:
            raise MissingConfigurationError(
                self._move_index,
                BRAND,
                "a FREESPACE Cartesian move needs seed joints as get_inverse_kin qnear",
            )
        qnear = joints_literal(m.seed_joints)
        self._select_tcp(frames)
        MoveJPose(target, qnear, accel=profile.joint_accel_radss, speed=profile.joint_speed_rads)

    def dwell(self, e: Dwell) -> None:
        Sleep(e.seconds)

    def wait_digital(self, e: WaitDigital) -> None:
        if e.is_input:
            WaitDigitalIn(e.index, e.value)
        else:
            WaitDigitalOut(e.index, e.value)

    def set_digital(self, e: SetDigital) -> None:
        _numbered_only(e.key, "digital output")
        SetDigitalOut(e.index, e.value)

    def set_analog(self, e: SetAnalog) -> None:
        _numbered_only(e.key, "analog output")
        if e.index not in ANALOG_PORTS or not ANALOG_MIN <= e.value <= ANALOG_MAX:
            raise UnsupportedInstructionError(
                f"URScript: set_standard_analog_out({e.index}, {e.value}) needs n in "
                f"{list(ANALOG_PORTS)} and a relative level in [{ANALOG_MIN}, {ANALOG_MAX}]"
            )
        SetAnalogOut(e.index, e.value)

    def tool_change(self, e: ToolChange) -> None:
        raise UnsupportedInstructionError(
            f"URScript: SetTool({e.tool_id}) has no indexed tool; the TCP follows each "
            "move's tcp_frame through the frame binding (set_tcp)"
        )

    def note(self, e: Note) -> None:
        Comment(e.text)

    def prog_finish(self) -> dict[str, str]:
        assert self._def is not None
        self._def.__exit__(None, None, None)
        return {f"{self._name}.script": self._writer.getvalue() + "\n"}
