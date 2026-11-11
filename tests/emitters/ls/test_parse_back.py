"""Parse-back contract: emitted LS reads back to the intended targets.

Parses the `/POS` table (joint degrees, X/Y/Z mm, W/P/R deg rebuilt as
`Rz(R) Ry(P) Rx(W)`, CONFIG turn numbers) and the frame lines of `/MN`, and
compares them with the program inputs.
"""

from __future__ import annotations

import math
import re

import numpy as np

from tesseract_robotics.emitters.ls import emit_ls
from tests.emitters.ls.spike import (
    FRAMES,
    J1_DEG,
    J2_DEG,
    P1,
    P2,
    posture_config,
    spike_composite,
    spike_profiles,
)

JOINT_TOL_DEG = 5e-5
POS_TOL_MM = 5e-4
ROT_TOL = 3 * math.radians(5e-5)
_NUM = r"[-+]?\d+\.\d+"


def _xyz_fixed(w: float, p: float, r: float) -> np.ndarray:
    rw, rp, rr = np.radians([w, p, r])
    rx = np.array([[1, 0, 0], [0, math.cos(rw), -math.sin(rw)], [0, math.sin(rw), math.cos(rw)]])
    ry = np.array([[math.cos(rp), 0, math.sin(rp)], [0, 1, 0], [-math.sin(rp), 0, math.cos(rp)]])
    rz = np.array([[math.cos(rr), -math.sin(rr), 0], [math.sin(rr), math.cos(rr), 0], [0, 0, 1]])
    return rz @ ry @ rx


def test_emitted_ls_parses_back_to_inputs() -> None:
    text = emit_ls(spike_composite(), spike_profiles(), frames=FRAMES, config=posture_config).text
    joints = [
        [float(x) for x in re.findall(rf"J\d = ({_NUM}) deg", m.group(0))]
        for m in re.finditer(r"J1 = .*?J6 = [-+]?\d+\.\d+ deg", text, re.DOTALL)
    ]
    assert np.allclose(joints[0], J1_DEG, atol=JOINT_TOL_DEG)
    assert np.allclose(joints[1], J2_DEG, atol=JOINT_TOL_DEG)
    carts = re.findall(
        rf"CONFIG : '([NF] [UD] [TB]), (-?\d+), (-?\d+), (-?\d+)',\n"
        rf"\tX = ({_NUM}) mm, Y = ({_NUM}) mm, Z = ({_NUM}) mm,\n"
        rf"\tW = ({_NUM}) deg, P = ({_NUM}) deg, R = ({_NUM}) deg",
        text,
    )
    assert len(carts) == 2
    for row, pose in zip(carts, (P1, P2), strict=True):
        assert row[0] == "N U T"
        assert row[1:4] == ("0", "0", "1")  # J6 at 200 / 190 deg is turn 1
        vals = [float(v) for v in row[4:]]
        assert np.allclose(vals[:3], np.asarray(pose.translation) * 1000.0, atol=POS_TOL_MM)
        assert np.allclose(_xyz_fixed(*vals[3:]), pose.rotation_matrix, atol=ROT_TOL)
    assert text.count("UF : 0, UT : 1") == 4
    assert re.search(r"^\s+\d+:UTOOL_NUM=1 ;$", text, re.MULTILINE)
    assert re.search(r"^\s+\d+:UFRAME_NUM=0 ;$", text, re.MULTILINE)
