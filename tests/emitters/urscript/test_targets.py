"""URScript target formatters: pure pose/joints → URScript literal strings."""

from __future__ import annotations

import math

import numpy as np

from tesseract_robotics.emitters.urscript.targets import (
    joints_literal,
    pose_literal,
    rotation_vector,
)
from tesseract_robotics.planning import Pose


def test_joints_literal_radians_six_decimals() -> None:
    joints = tuple(np.deg2rad([0.0, -20.0, 30.0, 0.0, 50.0, 0.0]))
    assert joints_literal(joints) == "[0.000000, -0.349066, 0.523599, 0.000000, 0.872665, 0.000000]"


def test_pose_literal_tool_down() -> None:
    pose = Pose.from_xyz_rpy([0.6, -0.1, 0.8], [math.pi, 0.0, 0.0])
    # x/y/z in metres (NOT scaled); rotation vector ≈ (pi, 0, 0).
    assert pose_literal(pose) == "p[0.600000, -0.100000, 0.800000, 3.141593, 0.000000, 0.000000]"


def test_rotation_vector_tool_down() -> None:
    pose = Pose.from_xyz_rpy([0.6, -0.1, 0.8], [math.pi, 0.0, 0.0])
    rx, ry, rz = rotation_vector(pose)
    assert np.allclose([rx, ry, rz], [math.pi, 0.0, 0.0], atol=1e-9)


def test_rotation_vector_roundtrips_for_random_poses() -> None:
    rng = np.random.default_rng(0)
    for _ in range(200):
        rpy = rng.uniform(-math.pi, math.pi, size=3)
        pose = Pose.from_xyz_rpy([0.0, 0.0, 0.0], rpy.tolist())
        rx, ry, rz = rotation_vector(pose)
        v = np.array([rx, ry, rz])
        theta = float(np.linalg.norm(v))
        if theta < 1e-12:
            r_back = np.eye(3)
        else:
            k = v / theta
            kmat = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
            # Rodrigues / matrix exponential of the rotation vector.
            r_back = np.eye(3) + math.sin(theta) * kmat + (1.0 - math.cos(theta)) * (kmat @ kmat)
        assert np.allclose(r_back, np.asarray(pose.rotation_matrix), atol=1e-9)
