"""Arm posture: the geometric configuration bits controllers store with a Cartesian target.

A 6R arm with a spherical wrist reaches most Cartesian poses in up to eight
joint configurations. Controllers that store Cartesian targets also store which
one was meant, as three binary choices plus per-axis turn information:

- is the wrist centre in front of or behind axis 1 (shoulder);
- is the wrist centre in front of or behind the lower arm (elbow);
- is axis 5 positive or negative (wrist).

These are geometric facts about a joint vector, so the emitter computes them
from the waypoint's seed joints through a `PostureSolver`. `OpwPostureSolver`
is the exact solver for the ortho-parallel spherical-wrist family (ABB, KUKA,
Fanuc, Yaskawa six-axis arms), parametrised exactly like tesseract's OPW
inverse-kinematics plugin (Brandstötter, Angerer, Hofbaur, "An analytical
solution of the inverse kinematics problem of industrial serial manipulators
with an ortho-parallel basis and a spherical wrist", 2014). A seed lying exactly
on a boundary has no defined bit and raises `SingularPostureError`.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Protocol

from .errors import InvalidProfileError, SingularPostureError

#: Six arm axes in an OPW arm.
_OPW_AXES = 6


@dataclass(frozen=True)
class ArmPosture:
    """The three geometric configuration bits of a 6R spherical-wrist arm.

    Attributes:
        wrist_in_front_of_axis1: Wrist centre lies on the positive side of axis 1
            in the frame that rotates with axis 1.
        wrist_in_front_of_lower_arm: Wrist centre lies in front of the lower arm
            (the axis-2 to axis-3 link), the elbow-up side of the ABB home posture.
        axis5_positive: Axis 5 is strictly positive.
    """

    wrist_in_front_of_axis1: bool
    wrist_in_front_of_lower_arm: bool
    axis5_positive: bool


class PostureSolver(Protocol):
    """Computes the `ArmPosture` of a six-axis joint vector (radians)."""

    def posture(self, joints: Sequence[float]) -> ArmPosture:
        """The posture of `joints` (radians, controller axis order).

        Raises:
            SingularPostureError: `joints` lie exactly on a posture boundary.
        """
        ...


@dataclass(frozen=True)
class OpwParameters:
    """OPW kinematic parameters, as in tesseract's `OPWInvKin` plugin config.

    Lengths in metres, offsets in radians, sign corrections in {-1, +1}. The
    OPW joint angle is `theta = sign_correction * q - offset`, matching the
    opw_kinematics forward kinematics.
    """

    a1: float
    a2: float
    b: float
    c1: float
    c2: float
    c3: float
    c4: float
    offsets: tuple[float, float, float, float, float, float]
    sign_corrections: tuple[int, int, int, int, int, int]

    @classmethod
    def build(
        cls,
        *,
        a1: float,
        a2: float,
        b: float,
        c1: float,
        c2: float,
        c3: float,
        c4: float,
        offsets: Sequence[float],
        sign_corrections: Sequence[int],
    ) -> OpwParameters:
        """Validate and construct.

        Raises:
            InvalidProfileError: a parameter is non-finite, the lower-arm or
                forearm length is not positive, or offsets/sign corrections are
                not six entries (sign corrections each ±1).
        """
        values = {"a1": a1, "a2": a2, "b": b, "c1": c1, "c2": c2, "c3": c3, "c4": c4}
        for label, value in values.items():
            if not math.isfinite(value):
                raise InvalidProfileError(f"OpwParameters.{label} must be finite; got {value}")
        if c2 <= 0.0:
            raise InvalidProfileError(f"OpwParameters.c2 (lower arm) must be > 0; got {c2}")
        if math.hypot(a2, c3) <= 0.0:
            raise InvalidProfileError("OpwParameters forearm (a2, c3) must have nonzero length")
        if len(offsets) != _OPW_AXES or not all(math.isfinite(o) for o in offsets):
            raise InvalidProfileError(
                f"OpwParameters.offsets must be 6 finite values; got {offsets}"
            )
        if len(sign_corrections) != _OPW_AXES or any(s not in (-1, 1) for s in sign_corrections):
            raise InvalidProfileError(
                f"OpwParameters.sign_corrections must be 6 values in {{-1, +1}}; got {sign_corrections}"
            )
        o = tuple(float(v) for v in offsets)
        s = tuple(int(v) for v in sign_corrections)
        return cls(a1, a2, b, c1, c2, c3, c4, (o[0], o[1], o[2], o[3], o[4], o[5]),
                   (s[0], s[1], s[2], s[3], s[4], s[5]))  # fmt: skip


@dataclass(frozen=True)
class OpwPostureSolver:
    """Exact `PostureSolver` for an OPW arm.

    In the arm plane (the frame rotated by axis 1), OPW forward kinematics puts
    the wrist centre at radial coordinate
    `cx1 = a1 + c2 sin(t2) + k sin(t2 + t3 + psi3)` with `k = hypot(a2, c3)` and
    `psi3 = atan2(a2, c3)`. The shoulder bit is the sign of `cx1`. The
    lower arm points along `(sin t2, cos t2)` and the forearm (elbow to wrist
    centre) along `(sin(t2+t3+psi3), cos(t2+t3+psi3))`; their cross product is
    `-k sin(t3 + psi3)`, so the wrist centre is in front of the lower arm exactly
    when `sin(t3 + psi3) > 0`. The wrist bit is the sign of the controller's
    axis-5 value itself (the joint value, not the OPW model angle).
    """

    params: OpwParameters

    def posture(self, joints: Sequence[float]) -> ArmPosture:
        """The posture of `joints` (radians, controller axis order).

        Raises:
            SingularPostureError: the wrist centre is exactly on axis 1 or on the
                lower-arm line, or axis 5 is exactly zero.
        """
        q = tuple(float(v) for v in joints)
        if len(q) != _OPW_AXES:
            raise InvalidProfileError(f"OPW posture needs 6 joint values; got {len(q)}")
        p = self.params
        t2 = p.sign_corrections[1] * q[1] - p.offsets[1]
        t3 = p.sign_corrections[2] * q[2] - p.offsets[2]
        psi3 = math.atan2(p.a2, p.c3)
        k = math.hypot(p.a2, p.c3)
        cx1 = p.a1 + p.c2 * math.sin(t2) + k * math.sin(t2 + t3 + psi3)
        elbow = math.sin(t3 + psi3)
        if cx1 == 0.0:
            raise SingularPostureError("the wrist centre (shoulder singularity)", q)
        if elbow == 0.0:
            raise SingularPostureError("the wrist centre (elbow singularity)", q)
        if q[4] == 0.0:
            raise SingularPostureError("axis 5 (wrist singularity)", q)
        return ArmPosture(
            wrist_in_front_of_axis1=cx1 > 0.0,
            wrist_in_front_of_lower_arm=elbow > 0.0,
            axis5_positive=q[4] > 0.0,
        )
