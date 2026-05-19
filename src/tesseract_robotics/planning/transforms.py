"""
Pose helpers for creating poses and transformations.

`Pose` inherits from `tesseract_common.Isometry3d` — every Pose IS an
Isometry3d, accepted directly by every C++ binding that expects one and
composable with all `Isometry3d` operators. The factory classmethods on
this class are the Python ergonomic layer over Eigen's ctors; nothing
about `Pose` adds an indirection.

Design rule (per PR #76 reviews): **no parallel numpy state on Pose**.
The old design kept a `_matrix: np.ndarray` alongside an Isometry3d
that was kept in sync — that was the anti-pattern. The current design
is "Pose IS-A Isometry3d, full stop": construction goes through
`Translation3d(x, y, z)` + `Quaterniond(...)` directly (no intermediate
`np.array([x, y, z])` for the Eigen caster to convert back), and the
single source of truth is the inherited Isometry3d state.

On-demand getters returning numpy *are* allowed and desirable:
`pose.rotation_matrix` (3×3 ndarray copy) and the inherited
`pose.translation()` / `pose.linear()` / `pose.matrix()` all return
numpy. They are reads from the canonical state, not a parallel store.

Example:
    from tesseract_robotics.planning import Pose, translation, rotation_z

    pose = Pose.from_xyz_rpy([0.5, 0.0, 0.3], [0, 0, 1.57])
    pose = translation(0.5, 0.0, 0.3) @ rotation_z(1.57)
    iso = pose  # no conversion needed — Pose is an Isometry3d
"""

from __future__ import annotations

import math
from typing import overload

import numpy as np
from numpy.typing import ArrayLike

from tesseract_robotics.tesseract_common import (
    X_AXIS,
    Y_AXIS,
    Z_AXIS,
    AngleAxisd,
    Isometry3d,
    Quaterniond,
    Translation3d,
)

# Denominator floor for `.normalized()`-style operations. Below this
# magnitude, float64 division by `.norm()` (Eigen's `.normalized()` or the
# manual `axis / norm`) is ill-conditioned — it produces NaN for zero input
# and unstably-amplified roundoff for sub-epsilon input. We refuse rather
# than silently corrupt the result.
#
# This is a NUMERICAL FLOOR, not a tolerance (per CLAUDE.md): it does not
# assert anything about input accuracy, only that the input can be safely
# divided into. The value is a property of float64 (~4 orders of magnitude
# above eps = 2.2e-16), independent of vector dimension — applies equally
# to unit quaternions (4 components) and rotation axes (3 components).
_NORMALISE_DENOM_FLOOR = 1e-12


class Pose(Isometry3d):
    """A 3D pose: `Isometry3d` plus convenient factory methods.

    `Pose` is a Python subclass of `Isometry3d`, not a wrapper — every
    `Pose` is also an `Isometry3d` (and `isinstance(p, Isometry3d)` is True).
    C++ bindings that accept `Isometry3d` accept `Pose` directly, and every
    `Isometry3d` method (`matrix()`, `translation()`, `rotation()`,
    `linear()`, `inverse()`, `isApprox()`, the in-place mutators, the `*`
    operator) is inherited.

    Factory convention: **one name per logical operation; both arraylike-
    pair and by-component signatures accepted via `@overload` *only when
    both styles read naturally* at the call site**.

    Where `@overload` is provided:
    - `from_xyz_quat(pos, quat)` / `from_xyz_quat(x, y, z, qx, qy, qz, qw)`
    - `from_xyz_rpy(pos, rpy)` / `from_xyz_rpy(x, y, z, roll, pitch, yaw)`

    Where it is NOT (asymmetry by design):
    - `from_xyz(x, y, z)` — 3 scalars only. `from_xyz([x, y, z])` saves no
      keystrokes and a literal `[0.5, 0, 0.3]` rarely already exists as a
      variable; the @overload would be ceremony.
    - `from_matrix_position(R, pos)` — `R` is always arraylike (9 scalars
      is absurd). `pos` could grow a scalar overload but the asymmetric
      `(R, x, y, z)` shape reads worse than the current uniform-arraylike.

    Rule of thumb: add `@overload` when both call shapes show up unprompted
    at real call sites. Don't add overloads for hypothetical symmetry.

    The inherited `Pose()` / `Pose(matrix)` / `Pose(iso)` /
    `Pose(Translation3d, Quaterniond)` ctors cover identity, raw-matrix,
    copy, and typed construction respectively.

    For reading: scalar accessors (`pose.x/y/z`, `pose.qx/qy/qz/qw`,
    `pose.roll/pitch/yaw`) and typed accessors (`pose.quaternion` →
    `Quaterniond`, `pose.rotation_matrix` → `np.ndarray` copy) cover the
    common cases. The inherited `pose.translation()`, `pose.rotation()`,
    `pose.linear()`, `pose.matrix()` and `pose.inverse()` are also available.

    Example:
        p = Pose.from_xyz(0.5, 0, 0.3)
        p = Pose.from_xyz_quat([0.5, 0, 0.3], [0, 0, 0.707, 0.707])
        p = Pose(np.eye(4))               # inherited matrix4d ctor
        p2 = p * Pose.from_xyz(1, 0, 0)   # Isometry3d composition
    """

    # ----- Factories ----------------------------------------------------

    @classmethod
    def from_xyz(cls, x: float, y: float, z: float) -> Pose:
        """Pure translation pose from scalar components."""
        return cls(Translation3d(x, y, z), Quaterniond.Identity())

    @overload
    @classmethod
    def from_xyz_quat(cls, position: ArrayLike, quaternion: ArrayLike) -> Pose: ...
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
    ) -> Pose: ...
    # pyright wants the impl signature to enumerate named params from every
    # overload (would be 7 params with 5 defaults, narrowed on len(args)
    # anyway). The dispatch in the body is correct; the wart is purely
    # structural. Tried `*args: Any` — pyright still complains; the check is
    # structural not type-based.
    @classmethod
    def from_xyz_quat(cls, *args) -> Pose:  # pyright: ignore[reportInconsistentOverload]
        """Pose from position `[x, y, z]` and scalar-last quaternion `[qx, qy, qz, qw]`.

        Two calling conventions, dispatched on `len(args)`:
        - **Pair**: `from_xyz_quat(position, quaternion)` — useful when
          position and quaternion already exist as arrays.
        - **Scalars**: `from_xyz_quat(x, y, z, qx, qy, qz, qw)` — useful
          for literal values.

        Quaternion is normalised; zero-magnitude quaternions are rejected
        with `ValueError`.
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
        pos = np.asarray(position, dtype=np.float64).ravel()
        quat = np.asarray(quaternion, dtype=np.float64).ravel()
        if pos.shape != (3,):
            raise ValueError(f"Position must have 3 elements, got {pos.shape}")
        if quat.shape != (4,):
            raise ValueError(f"Quaternion must have 4 elements, got {quat.shape}")
        return cls(
            Translation3d(float(pos[0]), float(pos[1]), float(pos[2])),
            _safe_quaternion(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])),
        )

    @overload
    @classmethod
    def from_xyz_rpy(cls, position: ArrayLike, rpy: ArrayLike) -> Pose: ...
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
    ) -> Pose: ...
    # Same structural pyright limitation as from_xyz_quat — see comment there.
    @classmethod
    def from_xyz_rpy(cls, *args) -> Pose:  # pyright: ignore[reportInconsistentOverload]
        """Pose from position `[x, y, z]` and XYZ Tait-Bryan angles `[roll, pitch, yaw]`.

        Two calling conventions:
        - **Pair**: `from_xyz_rpy(position, rpy)`.
        - **Scalars**: `from_xyz_rpy(x, y, z, roll, pitch, yaw)`.

        Composition: `R = Rz(yaw) · Ry(pitch) · Rx(roll)`.
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
        pos = np.asarray(position, dtype=np.float64).ravel()
        angles = np.asarray(rpy, dtype=np.float64).ravel()
        if pos.shape != (3,):
            raise ValueError(f"Position must have 3 elements, got {pos.shape}")
        if angles.shape != (3,):
            raise ValueError(f"RPY must have 3 elements, got {angles.shape}")
        roll, pitch, yaw = float(angles[0]), float(angles[1]), float(angles[2])
        q = (
            Quaterniond(AngleAxisd(yaw, Z_AXIS))
            * Quaterniond(AngleAxisd(pitch, Y_AXIS))
            * Quaterniond(AngleAxisd(roll, X_AXIS))
        )
        return cls(Translation3d(float(pos[0]), float(pos[1]), float(pos[2])), q)

    @classmethod
    def from_matrix_position(cls, rotation: ArrayLike, position: ArrayLike) -> Pose:
        """Pose from a 3x3 rotation matrix and `[x, y, z]` position.

        The rotation must be orthonormal — `Quaterniond(Matrix3d)` validates
        this at the binding boundary and raises `ValueError` for non-rotation
        input (scaling, shear, or accumulated FP drift beyond ~1e-12).
        """
        R = np.asarray(rotation, dtype=np.float64)
        pos = np.asarray(position, dtype=np.float64).ravel()
        if R.shape != (3, 3):
            raise ValueError(f"Rotation must be 3x3, got {R.shape}")
        if pos.shape != (3,):
            raise ValueError(f"Position must have 3 elements, got {pos.shape}")
        return cls(
            Translation3d(float(pos[0]), float(pos[1]), float(pos[2])),
            Quaterniond(R),
        )

    # ----- Scalar accessors ---------------------------------------------
    #
    # Ergonomic float-returning shortcuts over the inherited Isometry3d API.
    # Per Joelkang's PR #76 review: `pose.translation()[0]` is clunky; `pose.x`
    # is the ergonomic spelling. These do NOT reintroduce the numpy facade
    # @johnwason objected to — they're 1-line scalar extractors, no parallel
    # state, no numpy roundtrip.
    #
    # Perf note: each `qx/qy/qz/qw` access allocates a fresh `Quaterniond`
    # from `self.linear()`; each `roll/pitch/yaw` access additionally runs
    # an `eulerAngles("ZYX")` decomposition. Negligible in typical use
    # (logging, formatting, one-off reads). For multi-component reads in
    # a hot loop, do it once:
    #     q = pose.quaternion                       # one allocation
    #     qx, qy, qz, qw = q.x, q.y, q.z, q.w
    #     yaw, pitch, roll = q.eulerAngles("ZYX")   # one decomposition
    # Caching would require parallel mutable state — explicitly out of scope
    # given Pose IS-A Isometry3d (which is itself mutable via in-place ops).

    @property
    def x(self) -> float:
        """X translation component."""
        return float(self.translation()[0])

    @property
    def y(self) -> float:
        """Y translation component."""
        return float(self.translation()[1])

    @property
    def z(self) -> float:
        """Z translation component."""
        return float(self.translation()[2])

    @property
    def qx(self) -> float:
        """Quaternion x component (scalar-last [qx, qy, qz, qw] convention)."""
        return float(Quaterniond(self.linear()).x)

    @property
    def qy(self) -> float:
        """Quaternion y component."""
        return float(Quaterniond(self.linear()).y)

    @property
    def qz(self) -> float:
        """Quaternion z component."""
        return float(Quaterniond(self.linear()).z)

    @property
    def qw(self) -> float:
        """Quaternion w (scalar) component — last in the canonical order."""
        return float(Quaterniond(self.linear()).w)

    # ZYX Tait-Bryan decomposition has a singularity (gimbal lock) at
    # |pitch| = π/2 — at that pose only (yaw ± roll) is well-defined; the
    # individual angles become numerically unstable and depend on Eigen's
    # tie-breaking. The pose itself is unaffected; this is purely a property
    # of the (yaw, pitch, roll) decomposition. Avoid these properties on
    # poses with near-vertical tool axes; use `pose.quaternion` instead.

    @property
    def roll(self) -> float:
        """Roll angle (X rotation) from ZYX Tait-Bryan decomposition. Unstable near |pitch|=π/2 (see section comment)."""
        # eulerAngles("ZYX") returns (yaw, pitch, roll); roll is index 2.
        return float(Quaterniond(self.linear()).eulerAngles("ZYX")[2])

    @property
    def pitch(self) -> float:
        """Pitch angle (Y rotation) from ZYX Tait-Bryan decomposition. Singular at ±π/2 (see section comment)."""
        return float(Quaterniond(self.linear()).eulerAngles("ZYX")[1])

    @property
    def yaw(self) -> float:
        """Yaw angle (Z rotation) from ZYX Tait-Bryan decomposition. Unstable near |pitch|=π/2 (see section comment)."""
        return float(Quaterniond(self.linear()).eulerAngles("ZYX")[0])

    @property
    def rotation_matrix(self) -> np.ndarray:
        """3x3 rotation matrix as an independent numpy copy.

        Convenience over the inherited `linear()` for callers that want a
        guaranteed-owned numpy array (e.g. to mutate locally without
        aliasing the pose's state).
        """
        return np.asarray(self.linear()).copy()

    @property
    def quaternion(self) -> Quaterniond:
        """Quaternion of the rotation component as a typed `Quaterniond`.

        Returns the Eigen-bound type, not a numpy array — so `.x()`, `.y()`,
        `.z()`, `.w()`, `.coeffs()`, `.toRotationMatrix()`, slerp, etc. are
        all available without an extra wrapping step.
        """
        return Quaterniond(self.linear())

    # ----- Conversions / ops --------------------------------------------

    def __matmul__(self, other: object) -> Pose:
        """Compose: `self @ other` applies `other` then `self`.

        Returns `NotImplemented` (not raises) for non-Isometry3d operands so
        Python's binary-op protocol can try `other.__rmatmul__(self)` before
        giving up with `TypeError`. Parameter is annotated `object` because
        the dunder must accept anything at the language level — narrowing to
        `Isometry3d` would mislead static analysis into treating the
        `isinstance` check as dead code.
        """
        if isinstance(other, Isometry3d):
            return Pose(self * other)
        return NotImplemented

    def __repr__(self) -> str:
        # 6 sig figs (`:.6g`) matches the C++ binding's default ostream
        # precision used in Quaterniond / Isometry3d __repr__, so reprs are
        # consistent across the surface. Identity values render as `0` / `1`
        # rather than `0.000000` / `1.000000`, keeping common poses readable.
        t = self.translation()
        q = Quaterniond(self.linear()).coeffs()
        return (
            f"Pose(position=[{t[0]:.6g}, {t[1]:.6g}, {t[2]:.6g}], "
            f"quaternion=[{q[0]:.6g}, {q[1]:.6g}, {q[2]:.6g}, {q[3]:.6g}])"
        )


# Convenience factory functions


def translation(x: float, y: float, z: float) -> Pose:
    """Pure translation pose."""
    return Pose.from_xyz(x, y, z)


def rotation_x(angle: float) -> Pose:
    """Pure rotation about the X axis (radians)."""
    return Pose(Isometry3d(AngleAxisd(angle, X_AXIS)))


def rotation_y(angle: float) -> Pose:
    """Pure rotation about the Y axis (radians)."""
    return Pose(Isometry3d(AngleAxisd(angle, Y_AXIS)))


def rotation_z(angle: float) -> Pose:
    """Pure rotation about the Z axis (radians)."""
    return Pose(Isometry3d(AngleAxisd(angle, Z_AXIS)))


def rotation_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> Pose:
    """Pure rotation from scalar-last quaternion (qx, qy, qz, qw)."""
    return Pose(Isometry3d(_safe_quaternion(qx, qy, qz, qw)))


def rotation_from_axis_angle(axis: ArrayLike, angle: float) -> Pose:
    """Pure rotation from axis-angle. Axis need not be unit length (normalised here)."""
    axis_arr = np.asarray(axis, dtype=np.float64).ravel()
    if axis_arr.shape != (3,):
        raise ValueError(f"Axis must have 3 elements, got {axis_arr.shape}")
    norm = np.linalg.norm(axis_arr)
    if norm < _NORMALISE_DENOM_FLOOR:
        raise ValueError(f"Axis magnitude too small for normalisation: {norm}")
    return Pose(Isometry3d(AngleAxisd(angle, axis_arr / norm)))


def _safe_quaternion(qx: float, qy: float, qz: float, qw: float) -> Quaterniond:
    """Build a normalised quaternion from scalar-last (qx, qy, qz, qw).

    Eigen's `.normalized()` divides by `.norm()` unconditionally — for a
    zero quaternion that yields NaN, which then propagates silently into
    every subsequent computation. Guard at the source.

    Project convention is scalar-last [qx, qy, qz, qw] (see CLAUDE.md).
    Internally bridges to Eigen's scalar-first ctor via `from_xyzw`.
    """
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < _NORMALISE_DENOM_FLOOR:
        raise ValueError(
            f"Quaternion magnitude too small for normalisation: "
            f"|(qx={qx}, qy={qy}, qz={qz}, qw={qw})| = {norm}"
        )
    return Quaterniond.from_xyzw(qx, qy, qz, qw).normalized()
