"""RAPID motion-parameter bundle, type-tagged variable names, and the frame binding.

`RapidProfile` holds typed references to `speeddata` / `zonedata` variables; the
type-tagged `str` subclasses make passing a `ToolName` where a `SpeedName` is
expected a type error, even though both are plain strings at runtime. Tool and
work object are not motion parameters: they follow each move's frames, through
the `FrameBinding` built by `rapid_frames`.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

from ..core.frames import FrameBinding
from .syntax import RAPID_IDENTIFIERS


class SpeedName(str):
    """RAPID `speeddata` variable name."""

    __slots__ = ()


class ZoneName(str):
    """RAPID `zonedata` variable name."""

    __slots__ = ()


class ToolName(str):
    """RAPID `tooldata` variable name."""

    __slots__ = ()


class WobjName(str):
    """RAPID `wobjdata` variable name."""

    __slots__ = ()


# ABB built-in data, present on every controller.
v200: SpeedName = SpeedName("v200")
z10: ZoneName = ZoneName("z10")
tool0: ToolName = ToolName("tool0")
wobj0: WobjName = WobjName("wobj0")


@dataclass(frozen=True)
class RapidProfile:
    """RAPID motion parameters: typed references to declared speed and zone data.

    Fields are either ABB built-ins (`v200`, `z10`, `fine`, …) already on every
    controller, or names declared with the `Speed` / `Zone` DSL classes. Build via
    `RapidProfile.build` for validation.
    """

    speed: SpeedName = v200
    zone: ZoneName = z10

    @classmethod
    def build(cls, *, speed: str = v200, zone: str = z10) -> RapidProfile:
        """Validate and construct.

        Raises:
            IdentifierError: a name is not a legal RAPID identifier.
        """
        return cls(
            speed=SpeedName(RAPID_IDENTIFIERS.validate(speed)),
            zone=ZoneName(RAPID_IDENTIFIERS.validate(zone)),
        )


#: Shared default profile (ABB built-ins) used by the move DSL classes.
_DEFAULT_PROFILE = RapidProfile()


def rapid_frames(
    *, work_objects: Mapping[str, str], tools: Mapping[str, str]
) -> FrameBinding[WobjName, ToolName]:
    """Bind tesseract frames to RAPID `wobjdata` / `tooldata` variables.

    Args:
        work_objects: Working-frame link name to `wobjdata` name. The work
            object's user and object frames must place the link where the
            controller expects it (e.g. `{"base_link": "wobj0"}` for a floor
            robot whose base is the world frame).
        tools: TCP link name to `tooldata` name whose TCP is that link (e.g.
            `{"tool0": "tool0"}` for the flange).

    Returns:
        The binding `emit_rapid(frames=...)` takes.

    Raises:
        IdentifierError: a variable name is not a legal RAPID identifier.
    """
    return FrameBinding.build(
        work_frames={k: WobjName(RAPID_IDENTIFIERS.validate(v)) for k, v in work_objects.items()},
        tools={k: ToolName(RAPID_IDENTIFIERS.validate(v)) for k, v in tools.items()},
    )
