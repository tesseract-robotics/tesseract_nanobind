"""Fanuc LS target formatters: pure pose/joints → LS literal strings."""

from __future__ import annotations

import math

import numpy as np

from tesseract_robotics.emitters.ls.targets import (
    cartesian_pos_block,
    joint_move_line,
    joint_pos_block,
    linear_move_line,
    wpr_from_pose,
)
from tesseract_robotics.planning import Pose


def test_joint_pos_block_degrees_three_decimals() -> None:
    joints = tuple(np.deg2rad([0.0, -20.0, 30.0, 0.0, 50.0, 0.0]))
    assert joint_pos_block(1, joints) == (
        "P[1]{\n"
        "   GP1:\n"
        "\tUF : 0, UT : 1,\n"
        "\tJ1 = 0.000 deg, J2 = -20.000 deg, J3 = 30.000 deg,\n"
        "\tJ4 = 0.000 deg, J5 = 50.000 deg, J6 = 0.000 deg\n"
        "};"
    )


def test_cartesian_pos_block_tool_down() -> None:
    pose = Pose.from_xyz_rpy([0.6, -0.1, 0.8], [math.pi, 0.0, 0.0])
    assert cartesian_pos_block(3, pose) == (
        "P[3]{\n"
        "   GP1:\n"
        "\tUF : 0, UT : 1,\t\tCONFIG : 'N U T, 0, 0, 0',\n"
        "\tX = 600.000 mm, Y = -100.000 mm, Z = 800.000 mm,\n"
        "\tW = 180.000 deg, P = 0.000 deg, R = 0.000 deg\n"
        "};"
    )


def test_move_line_formatters() -> None:
    assert joint_move_line(1, 1, 100.0, "FINE") == "   1:J P[1] 100% FINE ;"
    assert linear_move_line(3, 3, 100.0, "FINE") == "   3:L P[3] 100mm/sec FINE ;"


def test_wpr_roundtrips_for_random_poses() -> None:
    rng = np.random.default_rng(0)
    for _ in range(200):
        rpy = rng.uniform(-math.pi, math.pi, size=3)
        pose = Pose.from_xyz_rpy([0.0, 0.0, 0.0], rpy.tolist())
        w, p, r = wpr_from_pose(pose)
        rw, rp, rr = np.radians([w, p, r])
        rx = np.array(
            [[1, 0, 0], [0, math.cos(rw), -math.sin(rw)], [0, math.sin(rw), math.cos(rw)]]
        )
        ry = np.array(
            [[math.cos(rp), 0, math.sin(rp)], [0, 1, 0], [-math.sin(rp), 0, math.cos(rp)]]
        )
        rz = np.array(
            [[math.cos(rr), -math.sin(rr), 0], [math.sin(rr), math.cos(rr), 0], [0, 0, 1]]
        )
        # Fanuc W/P/R is fixed-angle XYZ: R = Rz(R)·Ry(P)·Rx(W).
        assert np.allclose(rz @ ry @ rx, np.asarray(pose.rotation_matrix), atol=1e-9)
