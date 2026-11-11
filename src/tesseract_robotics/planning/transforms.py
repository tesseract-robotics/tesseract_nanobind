"""
Pose helpers for creating poses and transformations.

`Pose` inherits from `tesseract_common.Isometry3d` — every Pose IS an
Isometry3d, accepted directly by every C++ binding that expects one and
composable with all `Isometry3d` operators. The factory classmethods on
this class are the Python ergonomic layer over Eigen's ctors; nothing
about `Pose` adds an indirection.

Design rule: **no parallel numpy state on Pose**. A numpy matrix kept
alongside the Isometry3d must be kept in sync with it, and the two drift.
Pose is an Isometry3d, full stop: construction goes through
`Translation3d(x, y, z)` + `Quaterniond(...)` directly (no intermediate
`np.array([x, y, z])` for the Eigen caster to convert back), and the
single source of truth is the inherited Isometry3d state.

On-demand getters returning numpy *are* allowed and desirable:
`pose.rotation_matrix` (3×3 ndarray copy) and the inherited
`pose.translation` / `pose.linear` / `pose.matrix` all return
numpy. They are reads from the canonical state, not a parallel store.

Example:
    ```python
    from tesseract_robotics.planning import Pose, translation, rotation_z

    pose = Pose.from_xyz_rpy([0.5, 0.0, 0.3], [0, 0, 1.57])
    pose = translation(0.5, 0.0, 0.3) @ rotation_z(1.57)
    iso = pose  # no conversion needed: Pose is an Isometry3d
    ```
"""

from __future__ import annotations

import math
from typing import Self, overload

import numpy as np
from numpy.typing import ArrayLike

from tesseract_robotics.common import (
    EIGEN_DEFAULT_PREC,
    X_AXIS,
    Y_AXIS,
    Z_AXIS,
    AngleAxisd,
    Isometry3d,
    Quaterniond,
    Translation3d,
)
from tesseract_robotics.planning.errors import DegenerateRotationError, InvalidArrayShapeError

# Denominator floor for `.normalized()`-style operations. Below this
# magnitude, float64 division by `.norm()` (Eigen's `.normalized()` or the
# manual `axis / norm`) is ill-conditioned — it produces NaN for zero input
# and unstably-amplified roundoff for sub-epsilon input. We refuse rather
# than silently corrupt the result.
#
# This is a NUMERICAL FLOOR, not a tolerance: it does not assert anything
# about input accuracy, only that the input can be safely divided into. Sourced from Eigen's `NumTraits<double>::dummy_precision()`
# (re-exported as `EIGEN_DEFAULT_PREC` from the binding) so the whole
# stack — this constant, the C++ `kDegenerateGeometryEps`, the test
# constants — share one anchor.
_NORMALISE_DENOM_FLOOR = EIGEN_DEFAULT_PREC


class Pose(Isometry3d):
    """A 3D pose: `Isometry3d` plus convenient factory methods.

    `Pose` is a Python subclass of `Isometry3d`, not a wrapper — every
    `Pose` is also an `Isometry3d` (and `isinstance(p, Isometry3d)` is True).
    C++ bindings that accept `Isometry3d` accept `Pose` directly. The
    inherited surface: `matrix` / `translation` / `rotation` / `linear` are
    **properties** (stored-data getters); `inverse()`, `isApprox()`, the
    in-place mutators (`translate`, `rotate`, `pretranslate`, `prerotate`),
    and the `*` operator are **methods**.

    Factory convention: one name per logical operation. Every factory whose
    argument is one logical vector (a position, a quaternion, RPY angles, an
    axis) accepts both the array form and the scalar form via `@overload`:
    - `from_xyz(position)` / `from_xyz(x, y, z)`
    - `from_xyz_quat(position, quaternion)` / `from_xyz_quat(x, y, z, qx, qy, qz, qw)`
    - `from_xyz_rpy(position, rpy)` / `from_xyz_rpy(x, y, z, roll, pitch, yaw)`

    `from_matrix_position(R, pos)` takes a 3x3 matrix, which has no natural scalar
    form; its position is always an array.

    The inherited `Pose()` / `Pose(matrix)` / `Pose(iso)` /
    `Pose(Translation3d, Quaterniond)` ctors cover identity, raw-matrix,
    copy, and typed construction respectively.

    For reading: scalar accessors (`pose.x/y/z`, `pose.qx/qy/qz/qw`,
    `pose.roll/pitch/yaw`) and typed accessors (`pose.quaternion` →
    `Quaterniond`, `pose.rotation_matrix` → `np.ndarray` copy) cover the
    common cases. The inherited `pose.translation`, `pose.rotation`,
    `pose.linear`, `pose.matrix` and `pose.inverse()` are also available.

    Equality and hashability:

    - `==` is **identity-based** (Python's default; the binding deliberately
      does not override). Two structurally-identical poses constructed
      independently are `!=`. This is the right design for floating-point
      transforms — value-equality would silently collapse poses that differ
      by sub-epsilon drift and would *fail* for poses that *should* be equal
      but accumulated FP error through composition.
    - `Pose` is **unhashable** (`__hash__ = None`). `hash(pose)` raises
      `TypeError`; `pose` cannot be a `dict` key or `set` member. This is
      deliberate: identity-based hashing would let `pose in {p1, p2}` give
      counter-intuitive results — you could not look up a pose you just
      constructed by value. Floating-point types should not pretend to be
      hashable.
    - Use `pose.isApprox(other, prec=…)` for tolerance-based comparison
      (default `prec ≈ 1e-12`, tighten or loosen as context requires).

    Example:
        p = Pose.from_xyz(0.5, 0, 0.3)
        p = Pose.from_xyz_quat([0.5, 0, 0.3], [0, 0, 0.707, 0.707])
        p = Pose(np.eye(4))               # inherited matrix4d ctor
        p2 = p * Pose.from_xyz(1, 0, 0)   # Isometry3d composition
    """

    # ----- Factories ----------------------------------------------------

    @overload
    @classmethod
    def from_xyz(cls, position: ArrayLike, /) -> Self: ...
    @overload
    @classmethod
    def from_xyz(cls, x: float, y: float, z: float, /) -> Self: ...
    # Same structural pyright limitation as from_xyz_quat; see the comment there.
    @classmethod
    def from_xyz(cls, *args) -> Self:  # pyright: ignore[reportInconsistentOverload]
        """Pure translation pose.

        Two calling conventions, dispatched on `len(args)`:
        - **Array**: `from_xyz([x, y, z])`.
        - **Scalars**: `from_xyz(x, y, z)`.

        Raises:
            TypeError: neither one array nor three scalars were given.
            InvalidArrayShapeError: the array does not have 3 elements.
        """
        if len(args) == 1:
            (position,) = args
        elif len(args) == 3:
            position = args
        else:
            raise TypeError(
                f"from_xyz takes 1 arraylike arg (position) or 3 scalar args (x, y, z); "
                f"got {len(args)} args"
            )
        x, y, z = _vector3(position, "Position")
        return cls(Translation3d(x, y, z), Quaterniond.Identity())

    @overload
    @classmethod
    def from_xyz_quat(cls, position: ArrayLike, quaternion: ArrayLike, /) -> Self: ...
    @overload
    @classmethod
    def from_xyz_quat(
        cls,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
        /,
    ) -> Self: ...
    # pyright wants the impl signature to enumerate named params from every
    # overload (would be 7 params with 5 defaults, narrowed on len(args)
    # anyway). The dispatch in the body is correct; the wart is purely
    # structural. Two paths tried and rejected:
    #   1. `*args: Any` — pyright still complains; the check is structural,
    #      not type-based.
    #   2. Verbose impl signature `(position_or_x: ArrayLike | float, ...)`
    #      with 5 optional params, dispatching on `if z is None`, plus two
    #      private `_from_xyz_quat_pair` / `_from_xyz_quat_scalars` helpers.
    #      DOES drop the ignore cleanly (verified) but the impl signature
    #      becomes a misleading union obscuring both overload shapes — a
    #      reader scrolls past 30 lines of plumbing to find the dispatch.
    #      Readability cost > diagnostic-noise cost. Kept the one-line
    #      ignore as the principled trade-off.
    @classmethod
    def from_xyz_quat(cls, *args) -> Self:  # pyright: ignore[reportInconsistentOverload]
        """Pose from position `[x, y, z]` and scalar-last quaternion `[qx, qy, qz, qw]`.

        Two calling conventions, dispatched on `len(args)`:
        - **Pair**: `from_xyz_quat(position, quaternion)` — useful when
          position and quaternion already exist as arrays.
        - **Scalars**: `from_xyz_quat(x, y, z, qx, qy, qz, qw)` — useful
          for literal values.

        Quaternion is normalised.

        Raises:
            TypeError: wrong number of arguments.
            InvalidArrayShapeError: position or quaternion has the wrong number of elements.
            DegenerateRotationError: the quaternion is too short to normalise.
        """
        if len(args) == 2:
            position, quaternion = args
        elif len(args) == 7:
            position, quaternion = args[:3], args[3:]
        else:
            raise TypeError(
                "from_xyz_quat takes 2 arraylike args (pos, quat) "
                f"or 7 scalar args (x, y, z, qx, qy, qz, qw); got {len(args)} args"
            )
        x, y, z = _vector3(position, "Position")
        qx, qy, qz, qw = _vector(quaternion, 4, "Quaternion")
        return cls(Translation3d(x, y, z), _safe_quaternion(qx, qy, qz, qw))

    @overload
    @classmethod
    def from_xyz_rpy(cls, position: ArrayLike, rpy: ArrayLike, /) -> Self: ...
    @overload
    @classmethod
    def from_xyz_rpy(
        cls,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
        /,
    ) -> Self: ...
    # Same structural pyright limitation as from_xyz_quat — see comment there.
    @classmethod
    def from_xyz_rpy(cls, *args) -> Self:  # pyright: ignore[reportInconsistentOverload]
        """Pose from position `[x, y, z]` and roll-pitch-yaw angles `[roll, pitch, yaw]` (rad).

        Two calling conventions:
        - **Pair**: `from_xyz_rpy(position, rpy)`.
        - **Scalars**: `from_xyz_rpy(x, y, z, roll, pitch, yaw)`.

        Composition: `R = Rz(yaw) · Ry(pitch) · Rx(roll)`, the intrinsic Z-Y-X Tait-Bryan
        convention (the same rotation as fixed-axis rotations about X, then Y, then Z).
        `roll`, `pitch` and `yaw` read it back with the same convention.

        Raises:
            TypeError: wrong number of arguments.
            InvalidArrayShapeError: position or rpy has the wrong number of elements.
        """
        if len(args) == 2:
            position, rpy = args
        elif len(args) == 6:
            position, rpy = args[:3], args[3:]
        else:
            raise TypeError(
                "from_xyz_rpy takes 2 arraylike args (pos, rpy) "
                f"or 6 scalar args (x, y, z, roll, pitch, yaw); got {len(args)} args"
            )
        x, y, z = _vector3(position, "Position")
        angles = np.asarray(_vector3(rpy, "RPY"), dtype=np.float64)
        return cls(Translation3d(x, y, z), Quaterniond.from_rpy(angles))

    @classmethod
    def from_matrix_position(cls, rotation: ArrayLike, position: ArrayLike) -> Self:
        """Pose from a 3x3 rotation matrix and `[x, y, z]` position.

        The rotation must be orthonormal — `Quaterniond(Matrix3d)` validates
        this at the binding boundary and raises `ValueError` for non-rotation
        input (scaling, shear, or accumulated FP drift beyond ~1e-12).

        Raises:
            InvalidArrayShapeError: `rotation` is not 3x3 or `position` has no 3 elements.
        """
        R = np.asarray(rotation, dtype=np.float64)
        if R.shape != (3, 3):
            raise InvalidArrayShapeError(f"Rotation must be 3x3, got {R.shape}")
        x, y, z = _vector3(position, "Position")
        return cls(Translation3d(x, y, z), Quaterniond(R))

    # ----- Scalar accessors ---------------------------------------------
    #
    # Ergonomic float-returning shortcuts over the inherited Isometry3d API:
    # `pose.translation[0]` is clunky, `pose.x` reads. They are one-line scalar
    # extractors with no parallel state and no numpy round trip.
    #
    # Perf note: each `qx/qy/qz/qw` access allocates a fresh `Quaterniond`
    # from `self.linear`; each `roll/pitch/yaw` access additionally runs
    # an `eulerAngles("ZYX")` decomposition. Negligible in typical use
    # (logging, formatting, one-off reads). For multi-component reads in
    # a hot loop, do it once:
    #     q = pose.quaternion                       # one allocation
    #     qx, qy, qz, qw = q.x, q.y, q.z, q.w
    #     yaw, pitch, roll = q.eulerAngles("ZYX")   # one decomposition
    # Caching would require parallel mutable state — explicitly out of scope
    # given Pose IS-A Isometry3d (which is itself mutable via in-place ops).
    #
    # Quantified by `tests/benchmarks/test_pose_accessors.py`: 3× RPY
    # property reads ~6 µs (M-series, debug build); single-eulerAngles call
    # ~2.5 µs. Property paths are 2.5–3.5× slower than the bind-once pattern,
    # but all are sub-microsecond per access — recommendation matters in
    # tight loops, not in typical logging / formatting usage.

    @property
    def x(self) -> float:
        """X translation component."""
        return float(self.translation[0])

    @property
    def y(self) -> float:
        """Y translation component."""
        return float(self.translation[1])

    @property
    def z(self) -> float:
        """Z translation component."""
        return float(self.translation[2])

    @property
    def qx(self) -> float:
        """Quaternion x component (scalar-last [qx, qy, qz, qw] convention)."""
        return float(Quaterniond(self.linear).x)

    @property
    def qy(self) -> float:
        """Quaternion y component."""
        return float(Quaterniond(self.linear).y)

    @property
    def qz(self) -> float:
        """Quaternion z component."""
        return float(Quaterniond(self.linear).z)

    @property
    def qw(self) -> float:
        """Quaternion w (scalar) component — last in the canonical order."""
        return float(Quaterniond(self.linear).w)

    # The roll-pitch-yaw decomposition (intrinsic Z-Y-X Tait-Bryan) has a singularity (gimbal lock) at
    # |pitch| = π/2 — at that pose only (yaw ± roll) is well-defined; the
    # individual angles become numerically unstable and depend on Eigen's
    # tie-breaking. The pose itself is unaffected; this is purely a property
    # of the (yaw, pitch, roll) decomposition. Avoid these properties on
    # poses with near-vertical tool axes; use `pose.quaternion` instead.

    @property
    def roll(self) -> float:
        """Roll angle (X rotation) from the intrinsic Z-Y-X Tait-Bryan decomposition. Unstable near |pitch|=π/2 (see section comment)."""
        return float(Quaterniond(self.linear).to_rpy()[0])

    @property
    def pitch(self) -> float:
        """Pitch angle (Y rotation) from the intrinsic Z-Y-X Tait-Bryan decomposition. Singular at ±π/2 (see section comment)."""
        return float(Quaterniond(self.linear).to_rpy()[1])

    @property
    def yaw(self) -> float:
        """Yaw angle (Z rotation) from the intrinsic Z-Y-X Tait-Bryan decomposition. Unstable near |pitch|=π/2 (see section comment)."""
        return float(Quaterniond(self.linear).to_rpy()[2])

    @property
    def rotation_matrix(self) -> np.ndarray:
        """3x3 rotation matrix as an independent numpy copy.

        Convenience over the inherited `linear()` for callers that want a
        guaranteed-owned numpy array (e.g. to mutate locally without
        aliasing the pose's state).
        """
        return np.asarray(self.linear).copy()

    @property
    def quaternion(self) -> Quaterniond:
        """Quaternion of the rotation component as a typed `Quaterniond`.

        Returns the Eigen-bound type, not a numpy array — so `.x`, `.y`,
        `.z`, `.w` (properties), `.coeffs()`, `.toRotationMatrix()`, slerp,
        etc. are all available without an extra wrapping step.
        """
        return Quaterniond(self.linear)

    # ----- Conversions / ops --------------------------------------------

    # A subclass (a frame-tagged pose) gets its own type back from every
    # operation that builds a pose: the factories (`cls(...)`), and
    # `inverse()` / `*` by an Isometry3d, which the binding builds as
    # `type(self)` in C++ (`isometry_like`), so `@` below needs no re-wrap.

    def __matmul__(self, other: object) -> Self:
        """Compose: `self @ other` applies `other` then `self`.

        Returns `NotImplemented` (not raises) for non-Isometry3d operands so
        Python's binary-op protocol can try `other.__rmatmul__(self)` before
        giving up with `TypeError`. Parameter is annotated `object` because
        the dunder must accept anything at the language level — narrowing to
        `Isometry3d` would mislead static analysis into treating the
        `isinstance` check as dead code.
        """
        if isinstance(other, Isometry3d):
            return self * other
        return NotImplemented

    def __repr__(self) -> str:
        # 6 sig figs (`:.6g`) matches the C++ binding's default ostream
        # precision used in Quaterniond / Isometry3d __repr__, so reprs are
        # consistent across the surface. Identity values render as `0` / `1`
        # rather than `0.000000` / `1.000000`, keeping common poses readable.
        t = self.translation
        q = Quaterniond(self.linear).coeffs()
        return (
            f"Pose(position=[{t[0]:.6g}, {t[1]:.6g}, {t[2]:.6g}], "
            f"quaternion=[{q[0]:.6g}, {q[1]:.6g}, {q[2]:.6g}, {q[3]:.6g}])"
        )

    # Floating-point transforms are not value-hashable (drift would break
    # the eq/hash contract); identity-hashing would mislead callers who
    # build sets/dicts expecting value semantics. See class docstring.
    __hash__ = None  # type: ignore[assignment]


# Convenience factory functions


@overload
def translation(position: ArrayLike, /) -> Pose: ...
@overload
def translation(x: float, y: float, z: float, /) -> Pose: ...
def translation(*args) -> Pose:  # pyright: ignore[reportInconsistentOverload]
    """Pure translation pose: `translation([x, y, z])` or `translation(x, y, z)`."""
    return Pose.from_xyz(*args)


def rotation_x(angle: float) -> Pose:
    """Pure rotation about the X axis (radians)."""
    return Pose(Isometry3d(AngleAxisd(angle, X_AXIS)))


def rotation_y(angle: float) -> Pose:
    """Pure rotation about the Y axis (radians)."""
    return Pose(Isometry3d(AngleAxisd(angle, Y_AXIS)))


def rotation_z(angle: float) -> Pose:
    """Pure rotation about the Z axis (radians)."""
    return Pose(Isometry3d(AngleAxisd(angle, Z_AXIS)))


@overload
def rotation_from_quaternion(quaternion: ArrayLike, /) -> Pose: ...
@overload
def rotation_from_quaternion(qx: float, qy: float, qz: float, qw: float, /) -> Pose: ...
def rotation_from_quaternion(*args) -> Pose:  # pyright: ignore[reportInconsistentOverload]
    """Pure rotation from a scalar-last quaternion.

    Two calling conventions: `rotation_from_quaternion([qx, qy, qz, qw])` or
    `rotation_from_quaternion(qx, qy, qz, qw)`. The quaternion is normalised.

    Raises:
        TypeError: neither one array nor four scalars were given.
        InvalidArrayShapeError: the array does not have 4 elements.
        DegenerateRotationError: the quaternion is too short to normalise.
    """
    if len(args) == 1:
        (quaternion,) = args
    elif len(args) == 4:
        quaternion = args
    else:
        raise TypeError(
            "rotation_from_quaternion takes 1 arraylike arg (quaternion) or 4 scalar args "
            f"(qx, qy, qz, qw); got {len(args)} args"
        )
    return Pose(Isometry3d(_safe_quaternion(*_vector(quaternion, 4, "Quaternion"))))


@overload
def rotation_from_axis_angle(axis: ArrayLike, angle: float, /) -> Pose: ...
@overload
def rotation_from_axis_angle(x: float, y: float, z: float, angle: float, /) -> Pose: ...
def rotation_from_axis_angle(*args) -> Pose:  # pyright: ignore[reportInconsistentOverload]
    """Pure rotation from axis-angle (radians). The axis need not be unit length.

    Two calling conventions: `rotation_from_axis_angle([x, y, z], angle)` or
    `rotation_from_axis_angle(x, y, z, angle)`.

    Raises:
        TypeError: neither (axis, angle) nor (x, y, z, angle) was given.
        InvalidArrayShapeError: the axis does not have 3 elements.
        DegenerateRotationError: the axis is too short to normalise.
    """
    if len(args) == 2:
        axis, angle = args
    elif len(args) == 4:
        axis, angle = args[:3], args[3]
    else:
        raise TypeError(
            "rotation_from_axis_angle takes (axis, angle) or (x, y, z, angle); "
            f"got {len(args)} args"
        )
    axis_arr = np.asarray(_vector3(axis, "Axis"), dtype=np.float64)
    norm = float(np.linalg.norm(axis_arr))
    if norm < _NORMALISE_DENOM_FLOOR:
        raise DegenerateRotationError(f"Axis magnitude too small for normalisation: {norm}")
    return Pose(Isometry3d(AngleAxisd(angle, axis_arr / norm)))


def _vector(value: ArrayLike, size: int, what: str) -> tuple[float, ...]:
    """`value` as a flat tuple of `size` floats."""
    arr = np.asarray(value, dtype=np.float64).ravel()
    if arr.shape != (size,):
        raise InvalidArrayShapeError(f"{what} must have {size} elements, got {arr.shape}")
    return tuple(float(v) for v in arr)


def _vector3(value: ArrayLike, what: str) -> tuple[float, float, float]:
    """`value` as three floats."""
    x, y, z = _vector(value, 3, what)
    return x, y, z


def _safe_quaternion(qx: float, qy: float, qz: float, qw: float) -> Quaterniond:
    """Build a normalised quaternion from scalar-last (qx, qy, qz, qw).

    Eigen's `.normalized()` divides by `.norm()` unconditionally; for a
    zero quaternion that yields NaN, which then propagates silently into
    every subsequent computation. Guard at the source.

    The project convention is scalar-last [qx, qy, qz, qw] at every Python surface.
    Internally bridges to Eigen's scalar-first ctor via `from_xyzw`.

    Raises:
        DegenerateRotationError: the quaternion is too short to normalise.
    """
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < _NORMALISE_DENOM_FLOOR:
        raise DegenerateRotationError(
            f"Quaternion magnitude too small for normalisation: "
            f"|(qx={qx}, qy={qy}, qz={qz}, qw={qw})| = {norm}"
        )
    return Quaterniond.from_xyzw(qx, qy, qz, qw).normalized()
