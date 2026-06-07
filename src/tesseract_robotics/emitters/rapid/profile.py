"""RAPID motion-parameter bundle + type-tagged variable names.

The twin of the other brands' ``profile.py``. ``RapidProfile`` holds typed
references to declared ``PERS speeddata`` / ``zonedata`` / ``tooldata`` /
``wobjdata`` variables; the type-tagged ``str`` subclasses make passing a
``ToolName`` where a ``SpeedName`` is expected a type error, even though both are
plain strings at runtime. Build via ``RapidProfile.build`` (validates) or the
raw frozen constructor (minimal, bypass-safe).
"""

from __future__ import annotations

from dataclasses import dataclass


class SpeedName(str):
    """RAPID ``speeddata`` variable name. Returned by ``Speed(name, …)``."""

    __slots__ = ()


class ZoneName(str):
    """RAPID ``zonedata`` variable name. Returned by ``Zone(name, …)``."""

    __slots__ = ()


class ToolName(str):
    """RAPID ``tooldata`` variable name. Returned by ``Tooldata(name, …)``."""

    __slots__ = ()


class WobjName(str):
    """RAPID ``wobjdata`` variable name. Returned by ``Workobject(name, …)``."""

    __slots__ = ()


# ABB built-in references; safe defaults when no explicit declaration is needed.
v200: SpeedName = SpeedName("v200")
z10: ZoneName = ZoneName("z10")
tool0: ToolName = ToolName("tool0")
wobj0: WobjName = WobjName("wobj0")


@dataclass(frozen=True)
class RapidProfile:
    """RAPID motion parameter bundle — typed references to declared variables.

    Construct fields by either (a) the built-in constants (``v200`` / ``z10`` /
    ``tool0`` / ``wobj0``) already on every ABB controller, or (b) custom names
    declared via the ``Speed`` / ``Zone`` / ``Tooldata`` / ``Workobject`` DSL
    classes inside a ``Module`` block — each returns a typed name usable here.
    Frozen; build via ``RapidProfile.build`` for validation.
    """

    speed: SpeedName = v200
    zone: ZoneName = z10
    tool: ToolName = tool0
    wobj: WobjName = wobj0

    @classmethod
    def build(
        cls,
        *,
        speed: str = v200,
        zone: str = z10,
        tool: str = tool0,
        wobj: str = wobj0,
    ) -> RapidProfile:
        """Validate and construct. Raises ``ValueError`` on an empty variable name."""
        for label, value in (("speed", speed), ("zone", zone), ("tool", tool), ("wobj", wobj)):
            if not value:
                raise ValueError(f"RapidProfile.{label} must be a non-empty variable name")
        return cls(
            speed=SpeedName(speed), zone=ZoneName(zone), tool=ToolName(tool), wobj=WobjName(wobj)
        )


#: Shared default profile (ABB built-ins) used by the move DSL classes.
_DEFAULT_PROFILE = RapidProfile()
