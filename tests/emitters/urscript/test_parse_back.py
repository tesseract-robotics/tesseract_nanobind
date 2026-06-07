"""Parse-back contract: emitted URScript reads back to the intended numeric targets.

Asserts against the program *inputs* (not a saved file), so a wrong-but-stable
formatter regression a golden could hide is caught here.
"""

from __future__ import annotations

import re

import numpy as np

from tesseract_robotics.emitters.urscript import UrScriptProfile, emit_urscript
from tesseract_robotics.planning import CartesianTarget, JointTarget, MotionProgram, Pose

JOINT_TOL_RAD = 1e-5
POS_TOL_M = 1e-5
_NUM = r"[-+]?\d+\.\d+"


def _movej_joints(src: str) -> list[list[float]]:
    return [
        [float(x) for x in re.findall(_NUM, m.group(1))]
        for m in re.finditer(r"movej\(\[([^\]]*)\]", src)
    ]


def _movel_xyz(src: str) -> list[tuple[float, ...]]:
    pat = rf"movel\(p\[({_NUM}), ({_NUM}), ({_NUM}),"
    return [tuple(float(g) for g in m.groups()) for m in re.finditer(pat, src)]


def test_emitted_urscript_parses_back_to_inputs() -> None:
    jn = [f"joint_{i}" for i in range(1, 7)]
    j1 = [0.0, -20.0, 30.0, 0.0, 50.0, 0.0]
    j2 = [30.0, 10.0, 15.0, 20.0, 40.0, -10.0]
    prog = MotionProgram("manipulator", tcp_frame="tool0", profile="SPIKE")
    prog.move_to(JointTarget(np.deg2rad(j1), profile="J"))
    prog.move_to(JointTarget(np.deg2rad(j2), profile="J"))
    prog.linear_to(CartesianTarget(Pose.from_xyz_rpy([0.6, -0.1, 0.8], [np.pi, 0, 0]), profile="L"))
    prog.set_joint_names(jn)
    composite = prog.to_composite_instruction(joint_names=jn, tcp_frame="tool0")

    src = emit_urscript(
        composite,
        {"J": UrScriptProfile.build(), "L": UrScriptProfile.build()},
        program_name="SPIKE",
    ).text

    joints = _movej_joints(src)
    assert np.allclose(joints[0], np.deg2rad(j1), atol=JOINT_TOL_RAD)
    assert np.allclose(joints[1], np.deg2rad(j2), atol=JOINT_TOL_RAD)
    assert np.allclose(_movel_xyz(src)[0], [0.6, -0.1, 0.8], atol=POS_TOL_M)
