"""Fanuc LS target formatters: pure pose/joints to LS literal strings."""

from __future__ import annotations

import math

import numpy as np
import pytest

from tesseract_robotics.emitters.core.errors import ExternalAxisError
from tesseract_robotics.emitters.core.units import Radians
from tesseract_robotics.emitters.ls import FanucArmConfig
from tesseract_robotics.emitters.ls.syntax import turn_number
from tesseract_robotics.emitters.ls.targets import (
    cartesian_pos_block,
    config_string,
    joint_move_line,
    joint_pos_block,
    linear_move_line,
    wpr_from_pose,
)
from tesseract_robotics.planning import Pose


def _xyz_fixed(w: float, p: float, r: float) -> np.ndarray:
    rw, rp, rr = np.radians([w, p, r])
    rx = np.array([[1, 0, 0], [0, math.cos(rw), -math.sin(rw)], [0, math.sin(rw), math.cos(rw)]])
    ry = np.array([[math.cos(rp), 0, math.sin(rp)], [0, 1, 0], [-math.sin(rp), 0, math.cos(rp)]])
    rz = np.array([[math.cos(rr), -math.sin(rr), 0], [math.sin(rr), math.cos(rr), 0], [0, 0, 1]])
    return rz @ ry @ rx


def test_joint_pos_block() -> None:
    joints = tuple(np.deg2rad([0.0, -20.0, 30.0, 0.0, 50.0, 0.0]))
    assert joint_pos_block(1, joints, uf=0, ut=2) == (
        "P[1]{\n"
        "   GP1:\n"
        "\tUF : 0, UT : 2,\n"
        "\tJ1 = 0.0000 deg, J2 = -20.0000 deg, J3 = 30.0000 deg,\n"
        "\tJ4 = 0.0000 deg, J5 = 50.0000 deg, J6 = 0.0000 deg\n"
        "};"
    )


def test_joint_pos_block_rejects_external_axes() -> None:
    with pytest.raises(ExternalAxisError):
        joint_pos_block(1, (0.0,) * 7, uf=0, ut=1)


def test_cartesian_pos_block_tool_down() -> None:
    pose = Pose.from_xyz_rpy([0.6, -0.1, 0.8], [math.pi, 0.0, 0.0])
    assert cartesian_pos_block(3, pose, uf=1, ut=2, config="N U T, 0, 0, 0") == (
        "P[3]{\n"
        "   GP1:\n"
        "\tUF : 1, UT : 2,\t\tCONFIG : 'N U T, 0, 0, 0',\n"
        "\tX = 600.000 mm, Y = -100.000 mm, Z = 800.000 mm,\n"
        "\tW = 180.0000 deg, P = 0.0000 deg, R = 0.0000 deg\n"
        "};"
    )


@pytest.mark.parametrize(
    ("deg", "turn"),
    [(0, 0), (179, 0), (179.999, 0), (180, 1), (539, 1), (540, 2), (-179.999, 0), (-180, -1),
     (-539, -1), (-540, -2)],
)  # fmt: skip
def test_turn_number_table(deg: float, turn: int) -> None:
    assert turn_number(Radians(math.radians(deg))) == turn


def test_config_string() -> None:
    joints = tuple(np.deg2rad([200, 0, 0, -190, 30, 10]))
    assert (
        config_string(FanucArmConfig(flip=True, up=False, front=False), joints) == "F D B, 1, -1, 0"
    )


def test_move_line_formatters() -> None:
    assert joint_move_line(1, 1, 100.0, "FINE") == "   1:J P[1] 100% FINE ;"
    assert linear_move_line(3, 3, 100.0, "CNT50") == "   3:L P[3] 100mm/sec CNT50 ;"


def test_wpr_roundtrips_for_random_poses() -> None:
    rng = np.random.default_rng(0)
    for _ in range(200):
        pose = Pose.from_xyz_rpy([0.0, 0.0, 0.0], rng.uniform(-math.pi, math.pi, size=3).tolist())
        assert np.allclose(_xyz_fixed(*wpr_from_pose(pose)), pose.rotation_matrix, atol=1e-9)


@pytest.mark.parametrize("p_deg", [90.0, -90.0])
@pytest.mark.parametrize(("w_deg", "r_deg"), [(30.0, 0.0), (-50.0, 20.0)])
def test_wpr_gimbal_lock_both_branches(p_deg: float, w_deg: float, r_deg: float) -> None:
    rot = _xyz_fixed(w_deg, p_deg, r_deg)
    w, p, r = wpr_from_pose(Pose.from_matrix_position(rot, [0.0, 0.0, 0.0]))
    assert math.isclose(p, p_deg, abs_tol=1e-9) and r == 0.0
    assert np.allclose(_xyz_fixed(w, p, r), rot, atol=1e-9)
