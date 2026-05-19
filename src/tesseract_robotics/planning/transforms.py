"""
Pose helpers for creating poses and transformations.

`Pose` inherits from `tesseract_common.Isometry3d` — every Pose IS an
Isometry3d, accepted directly by every C++ binding that expects one and
composable with all `Isometry3d` operators. The factory classmethods on
this class are the Python ergonomic layer over Eigen's ctors; nothing
about `Pose` adds an indirection.

Per @johnwason's PR #76 review: no numpy facade on `Pose`. Construction
goes through `Translation3d(x, y, z)` + `Quaterniond(...)` directly — no
intermediate `np.array([x, y, z])` that the Eigen caster has to convert
back. Read accessors are the inherited `Isometry3d` methods.

Example:
    from tesseract_robotics.planning import Pose, translation, rotation_z

    pose = Pose.from_xyz_rpy(0.5, 0.0, 0.3, 0, 0, 1.57)
    pose = translation(0.5, 0.0, 0.3) @ rotation_z(1.57)
    iso = pose  # no conversion needed — Pose is an Isometry3d
"""

from __future__ import annotations

import math

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

# Minimum quaternion magnitude before normalisation is considered ill-posed.
# Below this the implied rotation is undefined and `.normalized()` would
# produce NaN. We refuse rather than silently corrupt the resulting pose.
_QUAT_MIN_NORM = 1e-12


class Pose(Isometry3d):
    """A 3D pose: `Isometry3d` plus convenient factory methods.

    `Pose` is a Python subclass of `Isometry3d`, not a wrapper — every
    `Pose` is also an `Isometry3d` (and `isinstance(p, Isometry3d)` is True).
    C++ bindings that accept `Isometry3d` accept `Pose` directly, and every
    `Isometry3d` method (`matrix()`, `translation()`, `rotation()`,
    `linear()`, `inverse()`, `isApprox()`, the in-place mutators, the `*`
    operator) is inherited.

    The classmethods below (`from_xyz_rpy`, `from_xyz_quat`, …) wrap Eigen's
    ctors with human-readable parameter conventions. To read components,
    use the inherited Eigen API directly: `pose.translation()` for the
    translation 3-vector, `pose.rotation()` / `pose.linear()` for the 3×3,
    `pose.matrix()` for the homogeneous 4×4, and `Quaterniond(pose.linear())`
    if you need a quaternion (then `.coeffs()` / `.eulerAngles("ZYX")`).

    Example:
        p = Pose.from_xyz_quat(0.5, 0, 0.3, 0, 0, 0.707, 0.707)
        p = Pose(np.eye(4))               # via Isometry3d's matrix4d ctor
        p2 = p * Pose.from_xyz(1, 0, 0)   # Isometry3d composition (returns Isometry3d)
    """

    # ----- Factories ----------------------------------------------------

    @classmethod
    def identity(cls) -> Pose:
        """Identity pose (no rotation, no translation)."""
        return cls()

    @classmethod
    def from_xyz(cls, x: float, y: float, z: float) -> Pose:
        """Pure translation pose."""
        return cls(Translation3d(x, y, z), Quaterniond.Identity())

    @classmethod
    def from_position(cls, position: ArrayLike) -> Pose:
        """Pure translation pose from a 3-element position array."""
        pos = np.asarray(position, dtype=np.float64).ravel()
        if pos.shape != (3,):
            raise ValueError(f"Position must have 3 elements, got {pos.shape}")
        return cls(
            Translation3d(float(pos[0]), float(pos[1]), float(pos[2])), Quaterniond.Identity()
        )

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
    ) -> Pose:
        """Pose from position and scalar-last quaternion (qx, qy, qz, qw).

        Quaternion is normalised so callers can pass un-normalised values;
        zero-magnitude quaternions are rejected with `ValueError`.
        """
        return cls(Translation3d(x, y, z), _safe_quaternion(qw, qx, qy, qz))

    @classmethod
    def from_position_quaternion(cls, position: ArrayLike, quaternion: ArrayLike) -> Pose:
        """Pose from position [x,y,z] and quaternion [qx,qy,qz,qw] (scalar-last)."""
        pos = np.asarray(position, dtype=np.float64).ravel()
        quat = np.asarray(quaternion, dtype=np.float64).ravel()
        if pos.shape != (3,):
            raise ValueError(f"Position must have 3 elements, got {pos.shape}")
        if quat.shape != (4,):
            raise ValueError(f"Quaternion must have 4 elements, got {quat.shape}")
        return cls(
            Translation3d(float(pos[0]), float(pos[1]), float(pos[2])),
            _safe_quaternion(float(quat[3]), float(quat[0]), float(quat[1]), float(quat[2])),
        )

    @classmethod
    def from_xyz_rpy(
        cls,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
    ) -> Pose:
        """Pose from position and XYZ Tait-Bryan angles; R = Rz(yaw)·Ry(pitch)·Rx(roll)."""
        q = (
            Quaterniond(AngleAxisd(yaw, Z_AXIS))
            * Quaterniond(AngleAxisd(pitch, Y_AXIS))
            * Quaterniond(AngleAxisd(roll, X_AXIS))
        )
        return cls(Translation3d(x, y, z), q)

    @classmethod
    def from_matrix(cls, matrix: ArrayLike) -> Pose:
        """Pose from a 4x4 homogeneous matrix (validated by Isometry3d's ctor)."""
        return cls(matrix)

    @classmethod
    def from_matrix_position(cls, rotation: ArrayLike, position: ArrayLike) -> Pose:
        """Pose from a 3x3 rotation matrix and a 3-element position.

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

    @classmethod
    def from_isometry(cls, isometry: Isometry3d) -> Pose:
        """Pose from an existing Isometry3d (defensively copied via Isometry3d's copy ctor)."""
        return cls(isometry)

    # ----- Conversions / ops --------------------------------------------

    def to_isometry(self) -> Isometry3d:
        """Return a plain `Isometry3d` copy (strips the `Pose` subclass).

        Legacy — `Pose` IS-A `Isometry3d`, so C++ bindings no longer need this
        conversion. Use only when you specifically want an `Isometry3d` and
        not a `Pose` subclass.
        """
        return Isometry3d(self)

    def __matmul__(self, other: Isometry3d) -> Pose:
        """Compose: `self @ other` applies `other` then `self`."""
        if isinstance(other, Isometry3d):
            return Pose(self * other)
        raise TypeError(f"Cannot multiply Pose with {type(other)}")

    def __repr__(self) -> str:
        t = self.translation()
        q = Quaterniond(self.linear()).coeffs()
        return (
            f"Pose(position=[{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}], "
            f"quaternion=[{q[0]:.4f}, {q[1]:.4f}, {q[2]:.4f}, {q[3]:.4f}])"
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
    return Pose(Isometry3d(_safe_quaternion(qw, qx, qy, qz)))


def rotation_from_axis_angle(axis: ArrayLike, angle: float) -> Pose:
    """Pure rotation from axis-angle. Axis need not be unit length (normalised here)."""
    axis_arr = np.asarray(axis, dtype=np.float64).ravel()
    if axis_arr.shape != (3,):
        raise ValueError(f"Axis must have 3 elements, got {axis_arr.shape}")
    norm = np.linalg.norm(axis_arr)
    if norm < _QUAT_MIN_NORM:
        raise ValueError(f"Axis magnitude too small for normalisation: {norm}")
    return Pose(Isometry3d(AngleAxisd(angle, axis_arr / norm)))


# Backwards compatibility alias
Transform = Pose


def _safe_quaternion(qw: float, qx: float, qy: float, qz: float) -> Quaterniond:
    """Build a normalised quaternion, rejecting zero-magnitude input.

    Eigen's `.normalized()` divides by `.norm()` unconditionally — for a
    zero quaternion that yields NaN, which then propagates silently into
    every subsequent computation. Guard at the source.
    """
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm < _QUAT_MIN_NORM:
        raise ValueError(
            f"Quaternion magnitude too small for normalisation: "
            f"|(w={qw}, x={qx}, y={qy}, z={qz})| = {norm}"
        )
    return Quaterniond(qw, qx, qy, qz).normalized()
