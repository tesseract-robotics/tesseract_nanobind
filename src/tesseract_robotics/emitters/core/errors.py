"""Emitter exception taxonomy: one named class per failure mode.

Every exception carries structured fields; the message is derived from them and
states what was found against what was expected. Brand packages raise these
classes, never a bare `EmitterError`. Validation errors also subclass the
matching builtin (`ValueError`, `NameError`, `RuntimeError`) so generic callers
can still catch them by kind.
"""

from __future__ import annotations

from collections.abc import Sequence


class EmitterError(Exception):
    """Base for all emitter errors. Never raised directly."""


class EmptyProgramError(EmitterError):
    """The `CompositeInstruction` has zero leaf instructions after the walk."""

    def __init__(self, description: str) -> None:
        self.description = description
        super().__init__(f"CompositeInstruction {description!r} has no leaf instructions")


class UnsupportedInstructionError(EmitterError):
    """An instruction the IR or the target dialect cannot represent exactly.

    Examples: CIRCULAR moves (tesseract's `MoveInstructionPoly` carries no via
    point), a tool change on a controller whose tool follows the frame binding,
    a named signal on a controller that only numbers its IO.
    """

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
    """A Cartesian move needs a robot configuration that cannot be derived exactly.

    Raised when the waypoint has no seed joints, when the caller supplied no
    posture/configuration resolver for a dialect that needs one, or when the
    controller's configuration rule for the move is not determinable from the
    plan. Never guessed.
    """

    def __init__(self, move_index: int, brand: str, reason: str) -> None:
        self.move_index = move_index
        self.brand = brand
        self.reason = reason
        super().__init__(
            f"{brand}: Cartesian move #{move_index} requires a robot configuration: {reason}"
        )


class SingularPostureError(EmitterError):
    """A seed configuration lies exactly on a posture boundary (shoulder, elbow or wrist).

    On the boundary the configuration bit the controller needs has no defined
    value, so the emitter refuses rather than pick a side.
    """

    def __init__(self, quantity: str, joints: Sequence[float]) -> None:
        self.quantity = quantity
        self.joints = tuple(joints)
        super().__init__(
            f"seed joints {self.joints} put {quantity} exactly on its boundary; "
            "the configuration bit is undefined there"
        )


class IdentifierError(EmitterError, ValueError):
    """A program/signal/variable identifier is not legal in the target language."""

    def __init__(self, name: str, brand: str, reason: str) -> None:
        self.name = name
        self.brand = brand
        self.reason = reason
        super().__init__(f"{brand}: identifier {name!r} is not emittable: {reason}")


class UnemittableTextError(EmitterError, ValueError):
    """Free text (comment, pendant message) that would break out of its syntax."""

    def __init__(self, text: str, brand: str, reason: str) -> None:
        self.text = text
        self.brand = brand
        self.reason = reason
        super().__init__(f"{brand}: text {text!r} is not emittable: {reason}")


class ExternalAxisError(EmitterError):
    """A move carries more joint values than the backend can emit."""

    def __init__(self, got: int, supported: int, brand: str) -> None:
        self.got = got
        self.supported = supported
        self.brand = brand
        super().__init__(
            f"{brand}: move carries {got} joint values but this emitter supports "
            f"{supported}; external axes are not emitted for {brand}"
        )


class UncoordinatedTargetError(EmitterError):
    """A coordinated move's waypoint joints do not match the external-axis layout.

    The split is by joint name, so a mismatched joint set cannot be resolved:
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


class InvalidLayoutError(EmitterError, ValueError):
    """An `ExternalAxisLayout` / `ExternalAxisSpec` violates its invariants."""


class InvalidProfileError(EmitterError, ValueError):
    """A brand motion profile was built with an out-of-range or empty field."""


class InvalidEventError(EmitterError, ValueError):
    """An IR event was built with a value outside its domain (e.g. a negative dwell)."""


class InvalidDeclarationError(EmitterError, ValueError):
    """A DSL data declaration (tooldata, zonedata, …) has an out-of-domain field."""


class UnboundFrameError(EmitterError):
    """A move's working or TCP frame has no controller frame in the binding."""

    def __init__(self, frame: str, role: str, brand: str, available: Sequence[str]) -> None:
        self.frame = frame
        self.role = role
        self.brand = brand
        self.available = sorted(available)
        super().__init__(
            f"{brand}: {role} frame {frame!r} has no controller binding; bound: {self.available}"
        )


class TcpOffsetError(EmitterError):
    """A move's `tcp_offset` cannot be represented by the target controller."""

    def __init__(self, brand: str, reason: str) -> None:
        self.brand = brand
        self.reason = reason
        super().__init__(f"{brand}: tcp_offset not representable: {reason}")


class InvalidFrameBindingError(EmitterError, ValueError):
    """A frame binding maps a frame to a controller handle outside its valid range."""


class ToolConflictError(EmitterError):
    """Moves in one program bind to different tools where the format allows only one."""

    def __init__(self, brand: str, tools: Sequence[int]) -> None:
        self.brand = brand
        self.tools = sorted(set(tools))
        super().__init__(
            f"{brand}: one position table can carry one tool, but the moves bind to {self.tools}"
        )


class MissingCalibrationError(EmitterError):
    """A unit conversion needs controller calibration data the caller did not supply."""

    def __init__(self, brand: str, what: str) -> None:
        self.brand = brand
        self.what = what
        super().__init__(f"{brand}: {what}")


class InvalidCalibrationError(EmitterError, ValueError):
    """Supplied calibration data is malformed (wrong axis count, zero or non-finite ratio)."""


class ProgramStructureError(EmitterError):
    """A DSL statement was written where the target grammar forbids it.

    E.g. a RAPID `PROC` inside a `WHILE`, a `PERS` declaration inside a routine,
    a data declaration after the first statement of a routine.
    """

    def __init__(self, brand: str, what: str, scope: Sequence[str]) -> None:
        self.brand = brand
        self.what = what
        self.scope = list(scope)
        where = " > ".join(self.scope) if self.scope else "top level"
        super().__init__(f"{brand}: {what} is not allowed at {where}")


class UndeclaredVariableError(EmitterError, NameError):
    """An assignment targets a variable that is not declared (or is `CONST`)."""

    def __init__(self, name: str, brand: str, reason: str) -> None:
        self.name = name
        self.brand = brand
        super().__init__(f"{brand}: cannot assign {name!r}: {reason}")


class NoActiveWriterError(EmitterError, RuntimeError):
    """A DSL statement ran with no writer of its brand active in the current context."""

    def __init__(self, writer: str) -> None:
        self.writer = writer
        super().__init__(
            f"no active {writer}: open one with `with {writer}() as w:` before writing statements"
        )


class ProgramFileCountError(EmitterError, ValueError):
    """`EmittedProgram.text` was read on a program that does not hold exactly one file."""

    def __init__(self, filenames: Sequence[str]) -> None:
        self.filenames = list(filenames)
        super().__init__(
            f".text requires exactly one file; this has {len(self.filenames)}: "
            f"{self.filenames}. Use .files[<name>]."
        )


class MissingManipulatorInfoError(EmitterError):
    """A move has no working or TCP frame after combining its `ManipulatorInfo` chain."""

    def __init__(self, field: str, move_index: int) -> None:
        self.field = field
        self.move_index = move_index
        super().__init__(
            f"move #{move_index} has an empty ManipulatorInfo.{field} after combining the "
            "composite and move ManipulatorInfo; set it on the program or the move"
        )
