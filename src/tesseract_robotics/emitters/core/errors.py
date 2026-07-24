"""Emitter exception taxonomy — one named class per failure mode.

Every exception carries structured fields; the message is derived. Brand
packages add subclasses but never raise bare ``EmitterError``. Messages are
chosen so the existing RAPID dispatcher tests (which match on substrings like
the quoted profile name, "CIRCULAR", "no leaf instructions") keep passing after
the reseat.
"""

from __future__ import annotations

from collections.abc import Sequence


class EmitterError(Exception):
    """Base for all emitter errors. Never raised directly."""


class EmptyProgramError(EmitterError):
    """The ``CompositeInstruction`` has zero leaf instructions after the walk."""

    def __init__(self, description: str) -> None:
        self.description = description
        super().__init__(f"CompositeInstruction {description!r} has no leaf instructions")


class UnsupportedInstructionError(EmitterError):
    """An instruction kind the IR cannot represent (notably CIRCULAR: tesseract's
    ``MoveInstructionPoly`` carries no via point)."""

    def __init__(self, what: str) -> None:
        self.what = what
        super().__init__(what)


class MissingProfileError(EmitterError):
    """A tesseract profile name on a move has no entry in the profiles mapping."""

    def __init__(self, profile: str, available: Sequence[str]) -> None:
        self.profile = profile
        self.available = sorted(available)
        super().__init__(
            f"profile {profile!r} referenced by move but not in profiles dict; "
            f"available: {self.available}"
        )


class MissingConfigurationError(EmitterError):
    """A Cartesian move needs a robot configuration but none can be derived.

    For formats that *require* config flags (Fanuc ``CONFIG : 'N U T, …'``) when
    no seed joints and no profile ``default_config`` exist. KRL does not hit this
    (status/turn optional, omitted in v1). Never guess silently.
    """

    def __init__(self, move_index: int, brand: str) -> None:
        self.move_index = move_index
        self.brand = brand
        super().__init__(
            f"{brand}: Cartesian move #{move_index} requires a robot configuration; "
            f"waypoint has no seed joints and the profile has no default_config"
        )


class IdentifierError(EmitterError):
    """A program/signal identifier cannot be made legal for the target language."""

    def __init__(self, name: str, brand: str, reason: str) -> None:
        self.name = name
        self.brand = brand
        self.reason = reason
        super().__init__(f"{brand}: identifier {name!r} is not emittable: {reason}")


class ExternalAxisError(EmitterError):
    """Joint state carries more axes than the backend is configured to emit."""

    def __init__(self, got: int, supported: int, brand: str) -> None:
        self.got = got
        self.supported = supported
        self.brand = brand
        super().__init__(
            f"{brand}: move carries {got} joint values but the profile supports "
            f"{supported} (6 robot + declared external axes); declare external axes"
        )


class UnknownKinematicTopologyError(EmitterError):
    """A group has no ROP/REP-classifiable external-axis structure.

    Raised by the classification layer (which owns the URDF/SRDF) when it cannot
    decide which joints are the arm vs. the external axes. Never guessed.
    """

    def __init__(self, group: str, reason: str) -> None:
        self.group = group
        self.reason = reason
        super().__init__(f"group {group!r} has no classifiable external-axis topology: {reason}")


class ExternalAxisUnitError(EmitterError):
    """An external axis's declared kind/unit contradicts its actual joint type.

    E.g. a joint declared ``ROTARY`` (degrees) whose URDF type is ``prismatic``
    (metres → millimetres). Emitting either unit would be silently wrong.
    """

    def __init__(self, joint_name: str, declared: str, expected: str) -> None:
        self.joint_name = joint_name
        self.declared = declared
        self.expected = expected
        super().__init__(
            f"external axis {joint_name!r} declared {declared} but its joint type implies {expected}"
        )


class UncoordinatedTargetError(EmitterError):
    """A coordinated move's waypoint joints do not match the external-axis layout.

    The split is by joint name, so a mismatched joint set cannot be resolved —
    the layout naming a joint the waypoint lacks (an omitted external DOF), or the
    waypoint carrying a joint the layout does not classify, both raise here rather
    than guess a slot assignment.
    """

    def __init__(
        self,
        missing: Sequence[str],
        extra: Sequence[str],
        waypoint_names: Sequence[str],
    ) -> None:
        self.missing = list(missing)
        self.extra = list(extra)
        self.waypoint_names = list(waypoint_names)
        super().__init__(
            f"coordinated waypoint joints {self.waypoint_names} do not match the "
            f"external-axis layout; missing {self.missing}, unexpected {self.extra}"
        )
