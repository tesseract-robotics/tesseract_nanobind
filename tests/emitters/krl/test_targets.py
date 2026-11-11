"""KRL target formatters: pure pose/joints to KRL literal strings."""

from __future__ import annotations

import math

import numpy as np
import pytest

from tesseract_robotics.emitters.core.errors import ExternalAxisError
from tesseract_robotics.emitters.krl.targets import (
    abc_from_pose,
    krl_axis_literal,
    krl_frame_literal,
)
from tesseract_robotics.planning import Pose


def _zyx(a: float, b: float, c: float) -> np.ndarray:
    ra, rb, rc = np.radians([a, b, c])
    rz = np.array([[math.cos(ra), -math.sin(ra), 0], [math.sin(ra), math.cos(ra), 0], [0, 0, 1]])
    ry = np.array([[math.cos(rb), 0, math.sin(rb)], [0, 1, 0], [-math.sin(rb), 0, math.cos(rb)]])
    rx = np.array([[1, 0, 0], [0, math.cos(rc), -math.sin(rc)], [0, math.sin(rc), math.cos(rc)]])
    return rz @ ry @ rx


def test_axis_literal_degrees() -> None:
    joints = tuple(np.deg2rad([0.0, -20.0, 30.0, 0.0, 50.0, 0.0]))
    assert krl_axis_literal(joints) == (
        "{A1 0.0000,A2 -20.0000,A3 30.0000,A4 0.0000,A5 50.0000,A6 0.0000}"
    )


def test_axis_literal_rejects_external_axes() -> None:
    with pytest.raises(ExternalAxisError):
        krl_axis_literal((0.0,) * 7)


def test_frame_literal_tool_down() -> None:
    pose = Pose.from_xyz_rpy([0.6, -0.1, 0.8], [math.pi, 0.0, 0.0])
    assert (
        krl_frame_literal(pose) == "{X 600.000,Y -100.000,Z 800.000,A 0.0000,B 0.0000,C 180.0000}"
    )


def test_abc_roundtrips_for_random_poses() -> None:
    rng = np.random.default_rng(0)
    for _ in range(200):
        pose = Pose.from_xyz_rpy([0.0, 0.0, 0.0], rng.uniform(-math.pi, math.pi, size=3).tolist())
        assert np.allclose(_zyx(*abc_from_pose(pose)), pose.rotation_matrix, atol=1e-9)


@pytest.mark.parametrize("b_deg", [90.0, -90.0])
@pytest.mark.parametrize(("a_deg", "c_deg"), [(30.0, 0.0), (-50.0, 20.0), (170.0, -40.0)])
def test_abc_gimbal_lock_both_branches(b_deg: float, a_deg: float, c_deg: float) -> None:
    """At B = ±90 only A ∓ C is determined; the extraction must rebuild R exactly."""
    rot = _zyx(a_deg, b_deg, c_deg)
    pose = Pose.from_matrix_position(rot, [0.0, 0.0, 0.0])
    a, b, c = abc_from_pose(pose)
    assert math.isclose(b, b_deg, abs_tol=1e-9) and c == 0.0
    assert np.allclose(_zyx(a, b, c), rot, atol=1e-9)
