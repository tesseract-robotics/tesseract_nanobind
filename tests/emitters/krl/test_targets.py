"""KRL target formatters: pure pose/joints → KRL literal strings."""

from __future__ import annotations

import math

import numpy as np

from tesseract_robotics.emitters.krl.targets import (
    abc_from_pose,
    krl_axis_literal,
    krl_frame_literal,
)
from tesseract_robotics.planning import Pose


def test_axis_literal_degrees_five_decimals() -> None:
    joints = tuple(np.deg2rad([0.0, -20.0, 30.0, 0.0, 50.0, 0.0]))
    assert (
        krl_axis_literal(joints)
        == "{A1 0.00000,A2 -20.00000,A3 30.00000,A4 0.00000,A5 50.00000,A6 0.00000}"
    )


def test_frame_literal_tool_down() -> None:
    pose = Pose.from_xyz_rpy([0.6, -0.1, 0.8], [math.pi, 0.0, 0.0])
    assert krl_frame_literal(pose) == "{X 600.000,Y -100.000,Z 800.000,A 0.000,B 0.000,C 180.000}"


def test_abc_roundtrips_for_random_poses() -> None:
    rng = np.random.default_rng(0)
    for _ in range(200):
        rpy = rng.uniform(-math.pi, math.pi, size=3)
        pose = Pose.from_xyz_rpy([0.0, 0.0, 0.0], rpy.tolist())
        a, b, c = abc_from_pose(pose)
        ra, rb, rc = np.radians([a, b, c])
        rz = np.array(
            [[math.cos(ra), -math.sin(ra), 0], [math.sin(ra), math.cos(ra), 0], [0, 0, 1]]
        )
        ry = np.array(
            [[math.cos(rb), 0, math.sin(rb)], [0, 1, 0], [-math.sin(rb), 0, math.cos(rb)]]
        )
        rx = np.array(
            [[1, 0, 0], [0, math.cos(rc), -math.sin(rc)], [0, math.sin(rc), math.cos(rc)]]
        )
        assert np.allclose(rz @ ry @ rx, np.asarray(pose.rotation_matrix), atol=1e-9)
