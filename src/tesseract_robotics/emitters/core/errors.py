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
