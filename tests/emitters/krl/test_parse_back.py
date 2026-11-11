"""Parse-back contract: emitted KRL reads back to the intended targets.

Asserts against the program inputs (not a saved file), so a wrong-but-stable
formatter regression a golden could hide is caught here: joint axes, X/Y/Z and
the full A/B/C orientation (rebuilt as `Rz(A) Ry(B) Rx(C)` and compared to the
input rotation), plus the frame selection and approximation statements.
"""

from __future__ import annotations

import math
import re

import numpy as np

from tesseract_robotics.emitters.krl import emit_krl
from tests.emitters.krl.spike import FRAMES, J1_DEG, J2_DEG, P1, P2, spike_composite, spike_profiles

#: Half a display step of the 4-decimal degree fields.
JOINT_TOL_DEG = 5e-5
#: Half a display step of the 3-decimal mm fields.
POS_TOL_MM = 5e-4
#: Rotation-matrix entry tolerance for a 5e-5 deg angle error per axis.
ROT_TOL = 3 * math.radians(5e-5)
_NUM = r"[-+]?\d+\.\d+"


def _zyx(a: float, b: float, c: float) -> np.ndarray:
    ra, rb, rc = np.radians([a, b, c])
    rz = np.array([[math.cos(ra), -math.sin(ra), 0], [math.sin(ra), math.cos(ra), 0], [0, 0, 1]])
    ry = np.array([[math.cos(rb), 0, math.sin(rb)], [0, 1, 0], [-math.sin(rb), 0, math.cos(rb)]])
    rx = np.array([[1, 0, 0], [0, math.cos(rc), -math.sin(rc)], [0, math.sin(rc), math.cos(rc)]])
    return rz @ ry @ rx


def test_emitted_krl_parses_back_to_inputs() -> None:
    src = emit_krl(spike_composite(), spike_profiles(), frames=FRAMES).text
    axes = [
        [float(x) for x in re.findall(_NUM, m.group(1))]
        for m in re.finditer(r"PTP \{([^}]*)\}", src)
    ]
    assert np.allclose(axes[0], J1_DEG, atol=JOINT_TOL_DEG)
    assert np.allclose(axes[1], J2_DEG, atol=JOINT_TOL_DEG)
    frames = re.findall(
        rf"LIN \{{X ({_NUM}),Y ({_NUM}),Z ({_NUM}),A ({_NUM}),B ({_NUM}),C ({_NUM})\}}", src
    )
    assert len(frames) == 2
    for row, pose in zip(frames, (P1, P2), strict=True):
        vals = [float(v) for v in row]
        assert np.allclose(vals[:3], np.asarray(pose.translation) * 1000.0, atol=POS_TOL_MM)
        assert np.allclose(_zyx(*vals[3:]), pose.rotation_matrix, atol=ROT_TOL)
    assert "$TOOL = TOOL_DATA[1]" in src and "$BASE = $NULLFRAME" in src
    assert src.index("$TOOL = TOOL_DATA[1]") < src.index("PTP ")
    assert "$APO.CDIS = 5.000" in src and re.search(r"LIN \{[^}]*\} C_DIS", src)
