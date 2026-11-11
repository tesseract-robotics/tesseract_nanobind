"""RAPID target bundles and literal formatters: pure, no writer state.

Holds the robtarget/jointtarget bundles (`RapidTarget` / `JointTarget`), the
`Config` (confdata) and `ExternalAxis` value types, and the one formatter per
literal. All unit conversions (metres to millimetres, radians to degrees,
scalar-last to scalar-first quaternion) go through `core.units` here.

`confdata` (3HAC050917, section `confdata`, six-axis serial robots):

- `cf1`, `cf4`, `cf6` are quadrant numbers of axes 1, 4 and 6: "quadrant 0 is
  the first quarter revolution, 0 to 90 deg, in a positive direction from the
  zero position; quadrant 1 is the next quarter revolution, 90 to 180 deg, and
  so on. Quadrant -1 is the quarter revolution 0 to (-90) deg". The manual does
  not assign the exact multiples of 90 deg; `quadrant` uses the half-open
  intervals `[k*90, (k+1)*90)` (floor), the one assignment under which quadrant
  0 owns 0 deg (the zero position) and every quadrant has the same shape.
- `cfx` selects one of eight configurations by three bits, in this order:
  wrist centre relative to axis 1 (in front of / behind), wrist centre relative
  to the lower arm (in front of / behind), axis 5 angle (positive / negative):
  0 = front, front, positive; 1 = front, front, negative; 2 = front, behind,
  positive; ...; 7 = behind, behind, negative.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, field

from tesseract_robotics.common import Quaterniond
from tesseract_robotics.planning.transforms import Pose

from ..core.errors import InvalidDeclarationError
from ..core.external_axes import ExternalAxisKind, ExternalAxisValue
from ..core.posture import ArmPosture
from ..core.precision import DEG_DECIMALS, MM_DECIMALS, QUAT_DECIMALS, fixed
from ..core.units import Metres, Radians, to_degrees, to_millimetres
from .profile import RapidProfile, ToolName, WobjName

#: Robot axes in a robtarget/jointtarget (robax a1..a6).
_ROBAX = 6
#: External-axis slots in a robtarget/jointtarget (eax a..f).
_EAX_SLOTS = 6
#: ABB's "no value" marker for an unused external-axis slot.
_EAX_UNUSED = "9E+09"
#: One quadrant is a quarter revolution (rad).
_QUARTER_TURN = math.pi / 2.0


def quadrant(angle: Radians) -> int:
    """ABB confdata quadrant number of a joint angle: `floor(angle / 90 deg)`."""
    return math.floor(angle / _QUARTER_TURN)


def cfx(posture: ArmPosture) -> int:
    """ABB six-axis `cfx` (0..7) from the three posture bits (manual table order)."""
    return (
        4 * (not posture.wrist_in_front_of_axis1)
        + 2 * (not posture.wrist_in_front_of_lower_arm)
        + (not posture.axis5_positive)
    )


@dataclass(frozen=True)
class Config:
    """RAPID robtarget confdata `[cf1, cf4, cf6, cfx]`."""

    cf1: int
    cf4: int
    cf6: int
    cfx: int

    @classmethod
    def from_joints(cls, joints: Sequence[float], posture: ArmPosture) -> Config:
        """Exact confdata of a six-axis joint configuration.

        Args:
            joints: Six arm-axis values (radians) of the configuration the target
                is meant to be reached in.
            posture: The posture of those joints (from a `PostureSolver`).

        Raises:
            InvalidDeclarationError: `joints` is not six values.
        """
        if len(joints) != _ROBAX:
            raise InvalidDeclarationError(
                f"confdata needs {_ROBAX} joint values; got {len(joints)}"
            )
        return cls(
            cf1=quadrant(Radians(joints[0])),
            cf4=quadrant(Radians(joints[3])),
            cf6=quadrant(Radians(joints[5])),
            cfx=cfx(posture),
        )

    def literal(self) -> str:
        return f"[{self.cf1}, {self.cf4}, {self.cf6}, {self.cfx}]"


@dataclass(frozen=True)
class ExternalAxis:
    """RAPID external-axis values in SI, one per used slot (at most six).

    Unused slots serialize to `9E+09`, ABB's marker for "no value". LINEAR values
    print in millimetres, ROTARY values in degrees.
    """

    values: tuple[ExternalAxisValue, ...] = ()

    @classmethod
    def build(cls, values: Sequence[ExternalAxisValue]) -> ExternalAxis:
        """Validate and construct.

        Raises:
            InvalidDeclarationError: more than six values, or a non-finite value.
        """
        vals = tuple(values)
        if len(vals) > _EAX_SLOTS:
            raise InvalidDeclarationError(
                f"RAPID has {_EAX_SLOTS} external-axis slots; got {len(vals)} values"
            )
        for v in vals:
            if not math.isfinite(v.value_si):
                raise InvalidDeclarationError(f"external-axis value must be finite; got {v}")
        return cls(vals)

    def literal(self) -> str:
        tokens = [_eax_token(v) for v in self.values]
        tokens.extend([_EAX_UNUSED] * (_EAX_SLOTS - len(tokens)))
        return "[" + ",".join(tokens) + "]"


def _eax_token(value: ExternalAxisValue) -> str:
    if value.kind is ExternalAxisKind.LINEAR:
        return fixed(to_millimetres(Metres(value.value_si)), MM_DECIMALS)
    if value.kind is ExternalAxisKind.ROTARY:
        return fixed(to_degrees(Radians(value.value_si)), DEG_DECIMALS)
    raise AssertionError(f"unhandled ExternalAxisKind: {value.kind!r}")  # pragma: no cover


@dataclass(frozen=True)
class RapidTarget:
    """RAPID robtarget bundle: pose, confdata and external axes.

    Attributes:
        pose: TCP pose in the work object (metres, scalar-last quaternion).
        config: confdata of the configuration the pose is meant to be reached in.
        external_axis: External-axis values (all unused by default).
    """

    pose: Pose
    config: Config
    external_axis: ExternalAxis = field(default_factory=ExternalAxis)

    def literal(self) -> str:
        """Serialize to `[[x, y, z], [q1, q2, q3, q4], [confdata], [eax]]` (mm, w-first)."""
        return (
            "["
            + ",".join(
                [
                    position_literal(self.pose),
                    orientation_literal(self.pose),
                    self.config.literal(),
                    self.external_axis.literal(),
                ]
            )
            + "]"
        )


@dataclass(frozen=True)
class JointTarget:
    """RAPID jointtarget bundle: six arm angles (radians) plus external axes.

    Build via `JointTarget.build` (validates the axis count and finiteness).
    """

    joints: tuple[Radians, Radians, Radians, Radians, Radians, Radians]
    external_axis: ExternalAxis = field(default_factory=ExternalAxis)

    @classmethod
    def build(
        cls, joints: Sequence[float], external_axis: ExternalAxis | None = None
    ) -> JointTarget:
        """Validate and construct.

        Raises:
            InvalidDeclarationError: not exactly six finite joint values.
        """
        if len(joints) != _ROBAX:
            raise InvalidDeclarationError(
                f"jointtarget needs exactly {_ROBAX} joint values; got {len(joints)}"
            )
        q = tuple(Radians(float(j)) for j in joints)
        if not all(math.isfinite(j) for j in q):
            raise InvalidDeclarationError(f"jointtarget joint values must be finite; got {q}")
        return cls((q[0], q[1], q[2], q[3], q[4], q[5]), external_axis or ExternalAxis())

    def literal(self) -> str:
        """Serialize to `[[a1, ..., a6], [eax]]` (degrees)."""
        deg = ", ".join(fixed(to_degrees(j), DEG_DECIMALS) for j in self.joints)
        return f"[[{deg}], {self.external_axis.literal()}]"


# ---------------------------------------------------------------------------
# Formatters, consumed by the DSL move and declaration classes.
# ---------------------------------------------------------------------------


def position_literal(pose: Pose) -> str:
    """`[x, y, z]` of `pose` in millimetres."""
    t = pose.translation
    return "[" + ", ".join(fixed(to_millimetres(Metres(float(v))), MM_DECIMALS) for v in t) + "]"


def orientation_literal(pose: Pose) -> str:
    """`[q1, q2, q3, q4]` = scalar-first `[w, x, y, z]` of `pose`'s rotation."""
    qx, qy, qz, qw = (float(v) for v in Quaterniond(pose.linear).coeffs())
    return "[" + ", ".join(fixed(v, QUAT_DECIMALS) for v in (qw, qx, qy, qz)) + "]"


def pose_literal(pose: Pose) -> str:
    """`[[x, y, z], [q1, q2, q3, q4]]` (the `pose` data type) of `pose`."""
    return f"[{position_literal(pose)},{orientation_literal(pose)}]"


def move_line(
    keyword: str, target: str, profile: RapidProfile, tool: ToolName, wobj: WobjName
) -> str:
    """A `MoveL` / `MoveJ` statement to one robtarget expression."""
    return f"{keyword} {target}, {profile.speed}, {profile.zone}, {tool} \\WObj:={wobj};"


def movec_line(via: str, end: str, profile: RapidProfile, tool: ToolName, wobj: WobjName) -> str:
    """A `MoveC` statement through `via` to `end`."""
    return f"MoveC {via}, {end}, {profile.speed}, {profile.zone}, {tool} \\WObj:={wobj};"
