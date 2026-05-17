"""
Pose helpers for creating poses and transformations.

Thin Pythonic wrapper around `tesseract_common.Isometry3d`. All transform
math delegates to Eigen — this module owns the construction API, not the
linear algebra.

Example:
    from tesseract_robotics.planning import Pose, translation, rotation_z

    pose = Pose.from_xyz_rpy(0.5, 0.0, 0.3, 0, 0, 1.57)
    pose = translation(0.5, 0.0, 0.3) @ rotation_z(1.57)
    isometry = pose.to_isometry()
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
)

# Minimum quaternion magnitude before normalisation is considered ill-posed.
# Below this the implied rotation is undefined and `.normalized()` would
# produce NaN. We refuse rather than silently corrupt the resulting pose.
_QUAT_MIN_NORM = 1e-12


class Pose:
    """A 3D pose backed by `Isometry3d`.

    Value-type semantics: constructing from an existing `Isometry3d`
    defensively copies, and every accessor (`matrix`, `position`,
    `rotation_matrix`, `quaternion`) returns a fresh numpy array.
    A `Pose` cannot be mutated from outside through any of its members.

    Example:
        p = Pose.from_xyz_quat(0.5, 0, 0.3, 0, 0, 0.707, 0.707)
        p = Pose(np.eye(4))               # from 4x4 matrix
        p = Pose(Isometry3d.Identity())   # from Eigen directly (copied)
        result = p1 @ p2                  # compose
    """

    __slots__ = ("_iso",)

    def __init__(self, matrix: ArrayLike | Isometry3d):
        """Build from a 4x4 homogeneous matrix or an Isometry3d.

        Passing an `Isometry3d` makes a defensive copy — subsequent
        mutation of the caller's instance does not affect this `Pose`.
        """
        if isinstance(matrix, Isometry3d):
            self._iso = Isometry3d(matrix)  # copy ctor — no aliasing
            return
        mat = np.asarray(matrix, dtype=np.float64)
        if mat.shape != (4, 4):
            raise ValueError(f"Pose matrix must be 4x4, got {mat.shape}")
        self._iso = Isometry3d(mat)

    # ----- Factories ----------------------------------------------------

    @classmethod
    def identity(cls) -> Pose:
        """Identity pose (no rotation, no translation)."""
        return cls(Isometry3d.Identity())

    @classmethod
    def from_xyz(cls, x: float, y: float, z: float) -> Pose:
        """Pure translation pose."""
        return cls(Isometry3d(np.array([x, y, z], dtype=np.float64), Quaterniond.Identity()))

    @classmethod
    def from_position(cls, position: ArrayLike) -> Pose:
        """Pure translation pose from a 3-element position array."""
        pos = np.asarray(position, dtype=np.float64).ravel()
        if pos.shape != (3,):
            raise ValueError(f"Position must have 3 elements, got {pos.shape}")
        return cls(Isometry3d(pos, Quaterniond.Identity()))

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

        The quaternion is normalised so callers can pass un-normalised
        values. Zero-magnitude quaternions are rejected (would otherwise
        yield NaN under normalisation).
        """
        return cls(
            Isometry3d(
                np.array([x, y, z], dtype=np.float64),
                _safe_quaternion(qw, qx, qy, qz),
            )
        )

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
            Isometry3d(
                pos,
                _safe_quaternion(quat[3], quat[0], quat[1], quat[2]),
            )
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
        return cls(Isometry3d(np.array([x, y, z], dtype=np.float64), q))

    @classmethod
    def from_matrix(cls, matrix: ArrayLike) -> Pose:
        """Pose from a 4x4 homogeneous matrix (validated by `__init__`)."""
        return cls(matrix)

    @classmethod
    def from_matrix_position(cls, rotation: ArrayLike, position: ArrayLike) -> Pose:
        """Pose from a 3x3 rotation matrix and a 3-element position.

        The rotation must be orthonormal. `Quaterniond(Matrix3d)` validates
        this at the binding boundary and raises `ValueError` for non-rotation
        input (scaling, shear, or accumulated FP drift beyond ~1e-12).
        """
        R = np.asarray(rotation, dtype=np.float64)
        pos = np.asarray(position, dtype=np.float64).ravel()
        if R.shape != (3, 3):
            raise ValueError(f"Rotation must be 3x3, got {R.shape}")
        if pos.shape != (3,):
            raise ValueError(f"Position must have 3 elements, got {pos.shape}")
        return cls(Isometry3d(pos, Quaterniond(R)))

    @classmethod
    def from_isometry(cls, isometry: Isometry3d) -> Pose:
        """Pose from an existing Isometry3d (defensively copied)."""
        return cls(isometry)

    # ----- Accessors ----------------------------------------------------

    @property
    def matrix(self) -> np.ndarray:
        """4x4 homogeneous matrix as a fresh numpy array. Safe to mutate."""
        return self._iso.matrix()

    @property
    def position(self) -> np.ndarray:
        """Translation as a fresh `[x, y, z]` numpy array. Safe to mutate."""
        return self._iso.translation()

    @property
    def x(self) -> float:
        """X translation component."""
        return float(self._iso.translation()[0])

    @property
    def y(self) -> float:
        """Y translation component."""
        return float(self._iso.translation()[1])

    @property
    def z(self) -> float:
        """Z translation component."""
        return float(self._iso.translation()[2])

    @property
    def rotation_matrix(self) -> np.ndarray:
        """3x3 rotation matrix as a fresh numpy array. Safe to mutate."""
        return self._iso.rotation()

    @property
    def quaternion(self) -> np.ndarray:
        """Quaternion as `[qx, qy, qz, qw]` (scalar-last) — Eigen's coeffs() layout."""
        return Quaterniond(self._iso.linear()).coeffs()

    @property
    def rpy(self) -> tuple[float, float, float]:
        """Roll-pitch-yaw angles in radians, extracted such that R = Rz(yaw)·Ry(pitch)·Rx(roll)."""
        yaw, pitch, roll = Quaterniond(self._iso.linear()).eulerAngles("ZYX")
        return (float(roll), float(pitch), float(yaw))

    # ----- Conversions / ops --------------------------------------------

    def to_isometry(self) -> Isometry3d:
        """Return an `Isometry3d` copy — safe to mutate without affecting this Pose."""
        return Isometry3d(self._iso)

    def inverse(self) -> Pose:
        """Inverse pose. Uses Eigen's isometry-aware inverse (Rᵀ, −Rᵀ·t), not general 4x4 inversion."""
        return Pose(self._iso.inverse())

    def isApprox(self, other: Pose, prec: float = 1e-12) -> bool:
        """Component-wise approximate equality. Delegates to `Isometry3d.isApprox`."""
        return self._iso.isApprox(other._iso, prec)

    def __matmul__(self, other: Pose) -> Pose:
        """Compose poses: `self @ other` is the transform that applies `other` then `self`."""
        if isinstance(other, Pose):
            return Pose(self._iso * other._iso)
        raise TypeError(f"Cannot multiply Pose with {type(other)}")

    def __repr__(self) -> str:
        pos = self.position
        quat = self.quaternion
        return (
            f"Pose(position=[{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}], "
            f"quaternion=[{quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f}])"
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
