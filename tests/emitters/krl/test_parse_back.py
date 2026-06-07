"""Parse-back contract: emitted KRL reads back to the intended numeric targets.

Asserts against the program *inputs* (not a saved file), so a wrong-but-stable
formatter regression a golden could hide is caught here.
"""

from __future__ import annotations

import re

import numpy as np

from tesseract_robotics.emitters.krl import KrlProfile, emit_krl
from tesseract_robotics.planning import CartesianTarget, JointTarget, MotionProgram, Pose

JOINT_TOL_DEG = 1e-4
POS_TOL_MM = 1e-3
_NUM = r"[-+]?\d+\.\d+"


def _ptp_axes(src: str) -> list[list[float]]:
    return [
        [float(x) for x in re.findall(_NUM, m.group(1))]
        for m in re.finditer(r"PTP \{([^}]*)\}", src)
    ]


def _lin_xyz(src: str) -> list[tuple[float, ...]]:
    pat = rf"LIN \{{X ({_NUM}),Y ({_NUM}),Z ({_NUM}),"
    return [tuple(float(g) for g in m.groups()) for m in re.finditer(pat, src)]


def test_emitted_krl_parses_back_to_inputs() -> None:
    jn = [f"joint_{i}" for i in range(1, 7)]
    j1 = [0.0, -20.0, 30.0, 0.0, 50.0, 0.0]
    j2 = [30.0, 10.0, 15.0, 20.0, 40.0, -10.0]
    prog = MotionProgram("manipulator", tcp_frame="tool0", profile="SPIKE")
    prog.move_to(JointTarget(np.deg2rad(j1), profile="J"))
    prog.move_to(JointTarget(np.deg2rad(j2), profile="J"))
    prog.linear_to(CartesianTarget(Pose.from_xyz_rpy([0.6, -0.1, 0.8], [np.pi, 0, 0]), profile="L"))
    prog.set_joint_names(jn)
    composite = prog.to_composite_instruction(joint_names=jn, tcp_frame="tool0")

    src = emit_krl(
        composite, {"J": KrlProfile.build(), "L": KrlProfile.build()}, program_name="SPIKE"
    ).text

    axes = _ptp_axes(src)
    assert np.allclose(axes[0], j1, atol=JOINT_TOL_DEG)
    assert np.allclose(axes[1], j2, atol=JOINT_TOL_DEG)
    assert np.allclose(_lin_xyz(src)[0], [600.0, -100.0, 800.0], atol=POS_TOL_MM)
