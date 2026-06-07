"""Parse-back contract: emitted LS reads back to the intended numeric targets.

Asserts against the program *inputs* (not a saved file), so a wrong-but-stable
formatter regression a golden could hide is caught here. We parse the ``/POS``
table — the canonical source of joint angles (deg) and Cartesian X/Y/Z (mm) —
which is where Fanuc actually stores the targets the ``/MN`` lines reference.
"""

from __future__ import annotations

import re

import numpy as np

from tesseract_robotics.emitters.ls import LsProfile, emit_ls
from tesseract_robotics.planning import CartesianTarget, JointTarget, MotionProgram, Pose

JOINT_TOL_DEG = 1e-3
POS_TOL_MM = 1e-3
_NUM = r"[-+]?\d+\.\d+"


def _pos_joints(text: str) -> list[list[float]]:
    """All J1..J6 degree tuples from joint-style ``/POS`` entries, in order."""
    out = []
    for m in re.finditer(r"J1 = .*?J6 = ([-+]?\d+\.\d+) deg", text, re.DOTALL):
        block = text[m.start() : m.end()]
        out.append([float(x) for x in re.findall(rf"J\d = ({_NUM}) deg", block)])
    return out


def _pos_xyz(text: str) -> list[tuple[float, float, float]]:
    """All (X, Y, Z) mm tuples from Cartesian ``/POS`` entries, in order."""
    pat = rf"X = ({_NUM}) mm, Y = ({_NUM}) mm, Z = ({_NUM}) mm"
    return [tuple(float(g) for g in m.groups()) for m in re.finditer(pat, text)]


def test_emitted_ls_parses_back_to_inputs() -> None:
    jn = [f"joint_{i}" for i in range(1, 7)]
    j1 = [0.0, -20.0, 30.0, 0.0, 50.0, 0.0]
    j2 = [30.0, 10.0, 15.0, 20.0, 40.0, -10.0]
    prog = MotionProgram("manipulator", tcp_frame="tool0", profile="SPIKE")
    prog.move_to(JointTarget(np.deg2rad(j1), profile="J"))
    prog.move_to(JointTarget(np.deg2rad(j2), profile="J"))
    prog.linear_to(CartesianTarget(Pose.from_xyz_rpy([0.6, -0.1, 0.8], [np.pi, 0, 0]), profile="L"))
    prog.set_joint_names(jn)
    composite = prog.to_composite_instruction(joint_names=jn, tcp_frame="tool0")

    text = emit_ls(
        composite, {"J": LsProfile.build(), "L": LsProfile.build()}, program_name="SPIKE"
    ).text

    joints = _pos_joints(text)
    assert np.allclose(joints[0], j1, atol=JOINT_TOL_DEG)
    assert np.allclose(joints[1], j2, atol=JOINT_TOL_DEG)
    assert np.allclose(_pos_xyz(text)[0], [600.0, -100.0, 800.0], atol=POS_TOL_MM)
