"""Parse-back contract: emitted JBI reads back to the intended numeric targets.

Asserts against the program *inputs* (not a saved file), so a wrong-but-stable
formatter regression a golden could hide is caught here. Each ``MOVJ C#####``
line is resolved to its ``C#####=`` position row, and the row's joint degrees are
compared to the input joint angles.
"""

from __future__ import annotations

import re

import numpy as np

from tesseract_robotics.emitters.jbi import JbiProfile, emit_jbi
from tesseract_robotics.planning import JointTarget, MotionProgram

JOINT_TOL_DEG = 1e-3
_NUM = r"[-+]?\d+\.\d+"


def _position_table(jbi: str) -> dict[str, list[float]]:
    """Map ``C#####`` → its joint-degree row from the ``//POS`` table."""
    return {
        m.group(1): [float(x) for x in m.group(2).split(",")]
        for m in re.finditer(r"^(C\d{5})=(.+)$", jbi, flags=re.MULTILINE)
    }


def _movj_indices(jbi: str) -> list[str]:
    """The ``C#####`` referenced by each ``MOVJ`` line, in program order."""
    return re.findall(r"MOVJ (C\d{5}) VJ=", jbi)


def test_emitted_jbi_parses_back_to_inputs() -> None:
    jn = [f"joint_{i}" for i in range(1, 7)]
    rows = [
        [0.0, -20.0, 30.0, 0.0, 50.0, 0.0],
        [30.0, 10.0, 15.0, 20.0, 40.0, -10.0],
        [10.0, 20.0, 30.0, 40.0, 50.0, 60.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]
    prog = MotionProgram("manipulator", tcp_frame="tool0", profile="SPIKE")
    for row in rows:
        prog.move_to(JointTarget(np.deg2rad(row), profile="J"))
    prog.set_joint_names(jn)
    composite = prog.to_composite_instruction(joint_names=jn, tcp_frame="tool0")

    jbi = emit_jbi(composite, {"J": JbiProfile.build()}, program_name="SPIKE").text

    table = _position_table(jbi)
    indices = _movj_indices(jbi)
    assert len(indices) == len(rows)
    for ref, expected in zip(indices, rows):
        assert np.allclose(table[ref], expected, atol=JOINT_TOL_DEG)
