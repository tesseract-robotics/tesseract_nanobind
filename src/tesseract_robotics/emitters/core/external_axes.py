"""External-axis classification supplied to the emitter, plus its SI IR value.

The emitter is a pure `CompositeInstruction` to text transform with no
environment handle, so it cannot itself tell which group joints are the six arm
axes and which are a track or positioner, nor a joint's unit. The caller (the
robot-cell layer, which owns the URDF/SRDF) classifies each group joint and
passes an `ExternalAxisLayout` to `emit_rapid(external_axes=...)`.

`lower()` splits every coordinated joint vector by joint name against that
layout, never by index, so an ordering difference between the layout and the
trajectory (e.g. the ROP reference cell lists the positioner joint first)
cannot misroute an arm value into an external-axis slot.

Units follow the joint kind, not the topology: a LINEAR axis is SI metres; a
ROTARY axis is SI radians. The conversion to display units lives in the brand
backend via `core.units`; the IR stays strictly SI.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from enum import Enum

from .errors import InvalidLayoutError

#: Robot axes a RAPID robtarget/jointtarget carries (robax a1..a6).
_ROBAX = 6
#: External-axis slots in a RAPID robtarget/jointtarget (eax_a..eax_f).
_MAX_EXTERNAL = 6


class ExternalAxisKind(Enum):
    """Kind of an external axis; fixes its SI unit.

    `LINEAR`: a track/rail (prismatic joint), SI metres.
    `ROTARY`: a positioner/turntable (revolute joint), SI radians.
    """

    LINEAR = "LINEAR"
    ROTARY = "ROTARY"


@dataclass(frozen=True)
class ExternalAxisValue:
    """One external-axis value in SI (metres if `LINEAR`, radians if `ROTARY`)."""

    value_si: float
    kind: ExternalAxisKind


@dataclass(frozen=True)
class ExternalAxisSpec:
    """One external joint: its exact group joint name plus its kind (unit)."""

    joint_name: str
    kind: ExternalAxisKind

    @classmethod
    def build(cls, joint_name: str, kind: ExternalAxisKind) -> ExternalAxisSpec:
        """Validate and construct.

        Raises:
            InvalidLayoutError: empty name or `kind` not an `ExternalAxisKind`.
        """
        if not joint_name:
            raise InvalidLayoutError("ExternalAxisSpec.joint_name must be a non-empty joint name")
        if not isinstance(kind, ExternalAxisKind):
            raise InvalidLayoutError(
                f"ExternalAxisSpec.kind must be ExternalAxisKind, got {type(kind).__name__}"
            )
        return cls(joint_name, kind)


@dataclass(frozen=True)
class ExternalAxisLayout:
    """Which group joints are the six arm axes vs. external axes, in RAPID order.

    `arm_joint_names` are the six robot-axis joint names in robax order
    (a1..a6). `external_axes` are the external joints in eax-slot order:
    `external_axes[0]` is `eax_a`, `[1]` is `eax_b`, and so on. The caller owns
    the controller's slot assignment by ordering this tuple.
    """

    arm_joint_names: tuple[str, ...]
    external_axes: tuple[ExternalAxisSpec, ...]

    @classmethod
    def build(
        cls,
        arm_joint_names: Sequence[str],
        external_axes: Sequence[ExternalAxisSpec],
    ) -> ExternalAxisLayout:
        """Validate one arm/external classification.

        Raises:
            InvalidLayoutError: not exactly six non-empty arm names, more than six
                external axes, a non-`ExternalAxisSpec` entry, or a joint name used twice.
        """
        arm = tuple(str(name) for name in arm_joint_names)
        ext = tuple(external_axes)
        if len(arm) != _ROBAX:
            raise InvalidLayoutError(
                f"ExternalAxisLayout needs exactly {_ROBAX} arm joint names, got {len(arm)}: {arm}"
            )
        if any(not name for name in arm):
            raise InvalidLayoutError("ExternalAxisLayout arm joint names must all be non-empty")
        if len(ext) > _MAX_EXTERNAL:
            raise InvalidLayoutError(
                f"RAPID has {_MAX_EXTERNAL} external-axis slots; got {len(ext)} external axes"
            )
        for spec in ext:
            if not isinstance(spec, ExternalAxisSpec):
                raise InvalidLayoutError(
                    f"external_axes entries must be ExternalAxisSpec, got {type(spec).__name__}"
                )
        names = list(arm) + [spec.joint_name for spec in ext]
        if len(names) != len(set(names)):
            raise InvalidLayoutError(
                f"ExternalAxisLayout joint names must be unique across arm + external: {names}"
            )
        return cls(arm, ext)

    @property
    def joint_names(self) -> tuple[str, ...]:
        """Every group joint name the layout accounts for (arm followed by external)."""
        return self.arm_joint_names + tuple(spec.joint_name for spec in self.external_axes)
