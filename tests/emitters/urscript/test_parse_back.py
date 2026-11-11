"""Parse-back contract: emitted URScript reads back to the intended targets.

Joint vectors, the full `p[x, y, z, rx, ry, rz]` pose (rotation vector turned
back into a matrix), the `set_tcp` pose and the `qnear` seed of the IK move.
"""

from __future__ import annotations

import math
import re

import numpy as np

from tesseract_robotics.emitters.urscript import emit_urscript
from tests.emitters.urscript.spike import (
    FRAMES,
    J1_DEG,
    J2_DEG,
    P1,
    P2,
    SEED2_DEG,
    TCP,
    spike_composite,
    spike_profiles,
)

#: Half a display step of the 6-decimal radian and metre fields.
TOL = 5e-7
_NUM = r"[-+]?\d+\.\d+"


def _rodrigues(v: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(v))
    if theta == 0.0:
        return np.eye(3)
    k = v / theta
    kmat = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(theta) * kmat + (1.0 - math.cos(theta)) * (kmat @ kmat)


def _vec(text: str) -> np.ndarray:
    return np.array([float(x) for x in re.findall(_NUM, text)])


def test_emitted_urscript_parses_back_to_inputs() -> None:
    src = emit_urscript(spike_composite(), spike_profiles(), frames=FRAMES).text
    joints = [_vec(m) for m in re.findall(r"movej\(\[([^\]]*)\]", src)]
    assert np.allclose(joints[0], np.deg2rad(J1_DEG), atol=TOL)
    assert np.allclose(joints[1], np.deg2rad(J2_DEG), atol=TOL)
    [tcp] = [_vec(m) for m in re.findall(r"set_tcp\(p\[([^\]]*)\]\)", src)]
    assert np.allclose(tcp[:3], TCP.translation, atol=TOL)
    poses = [_vec(m) for m in re.findall(r"(?:movel\(|get_inverse_kin\()p\[([^\]]*)\]", src)]
    for got, pose in zip(poses, (P1, P2), strict=True):
        assert np.allclose(got[:3], pose.translation, atol=TOL)
        assert np.allclose(_rodrigues(got[3:]), pose.rotation_matrix, atol=4 * TOL)
    [qnear] = [_vec(m) for m in re.findall(r"qnear=\[([^\]]*)\]", src)]
    assert np.allclose(qnear, np.deg2rad(SEED2_DEG), atol=TOL)
