"""CartLineConstraint.getJacobian() with fewer than 6 indices raises (#238, stopgap for #237).

trajopt 0.35.0 sizes a CartLineConstraint's Jacobian by ``indices.size()`` rows but fills six
whatever that size, in both the numeric (cartesian_line_constraint.cpp:179-188) and the analytic
(:238-247) branch: fewer than 6 indices write past the matrix and corrupt the heap (upstream
trajopt#618). The binding raises ``CartLineIndicesUnsupportedError`` before calling C++.
``getValues()`` is correct with any 1..6 indices and stays callable.

Each case runs in a child interpreter: unfixed, the call crashes most runs but can also return,
so the child reports what happened and the test asserts the exception.
"""

from __future__ import annotations

import subprocess
import sys

import pytest

from tesseract_robotics import trajopt_ifopt as ti

# s: a child loads the iiwa environment and builds one constraint; about 1 s on an M-series Mac
CHILD_TIMEOUT_S = 60.0
# m: the line's length along x; any nonzero length (equal endpoints are invalid)
LINE_LENGTH = 0.1
# rows of a twist error: 3 linear + 3 angular
TWIST_SIZE = 6
# a partial index set: the linear rows only
LINEAR_INDICES = (0, 1, 2)

_CHILD_SCRIPT = f"""\
import sys
from pathlib import Path

import numpy as np

from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics.tesseract_common import GeneralResourceLocator, Isometry3d
from tesseract_robotics.tesseract_environment import Environment

n_indices, numeric, link = int(sys.argv[1]), sys.argv[2] == "numeric", sys.argv[3] == "linked"

locator = GeneralResourceLocator()
urdf = locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf")
srdf = locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.srdf")
env = Environment()
assert env.init(Path(urdf.getFilePath()), Path(srdf.getFilePath()), locator)
manip = env.getKinematicGroup("manipulator")
joint_names = list(manip.getJointNames())
n = len(joint_names)
nodes = ti.createNodesVariables("trajectory", joint_names, [np.full(n, 0.1)], ti.toBounds(manip.getLimits().joint_limits))
identity = Isometry3d.Identity()

info = ti.CartLineInfo()
info.manip = manip
info.source_frame = "tool0"
info.target_frame = "base_link"
info.source_frame_offset = identity
info.target_frame_offset1 = identity
line_end = np.eye(4)
line_end[0, 3] = {LINE_LENGTH}
info.target_frame_offset2 = Isometry3d(line_end)
info.indices = np.arange(n_indices)
constraint = ti.CartLineConstraint(info, nodes.getNodes()[0].getVar("joints"), np.ones(n_indices))
constraint.use_numeric_differentiation = numeric
if link:
    constraint.linkWithVariables(nodes)
    print(f"VALUES {{len(constraint.getValues())}}", flush=True)
try:
    shape = constraint.getJacobian().shape
except Exception as exc:
    print(f"RAISED {{type(exc).__name__}}|{{exc}}", flush=True)
else:
    print(f"RETURNED {{shape[0]}} {{shape[1]}} {{nodes.getRows()}}", flush=True)
"""


def _run_child(n_indices: int, differentiation: str, link: str) -> list[str]:
    proc = subprocess.run(
        [sys.executable, "-c", _CHILD_SCRIPT, str(n_indices), differentiation, link],
        capture_output=True,
        text=True,
        check=False,
        timeout=CHILD_TIMEOUT_S,
    )
    traceback_at = proc.stderr.find("Traceback")
    assert proc.returncode == 0, (
        f"child died (rc={proc.returncode}, SIGSEGV is -11/139, SIGABRT -6/134): "
        f"{proc.stderr[traceback_at:] if traceback_at >= 0 else proc.stderr}"
    )
    return proc.stdout.splitlines()


@pytest.mark.parametrize("differentiation", ["analytic", "numeric"])
def test_partial_indices_jacobian_raises(differentiation):
    lines = _run_child(len(LINEAR_INDICES), differentiation, "linked")
    assert lines[0] == f"VALUES {len(LINEAR_INDICES)}", lines
    assert lines[1].startswith("RAISED CartLineIndicesUnsupportedError|"), lines
    message = lines[1].split("|", 1)[1]
    assert (
        "'CartLine'" in message
        and f"{len(LINEAR_INDICES)} indices" in message
        and "#237" in message
    ), message


@pytest.mark.parametrize("differentiation", ["analytic", "numeric"])
def test_six_indices_jacobian_returns(differentiation):
    lines = _run_child(TWIST_SIZE, differentiation, "linked")
    assert lines[0] == f"VALUES {TWIST_SIZE}", lines
    jac_rows, jac_cols, n_vars = map(int, lines[1].removeprefix("RETURNED ").split())
    assert (jac_rows, jac_cols) == (TWIST_SIZE, n_vars)


def test_unlinked_check_comes_first():
    lines = _run_child(len(LINEAR_INDICES), "numeric", "unlinked")
    assert lines[0].startswith("RAISED ConstraintNotLinkedError|"), lines


def test_cartline_indices_unsupported_error_is_a_value_error():
    assert issubclass(ti.CartLineIndicesUnsupportedError, ValueError)


def test_cartline_constraint_binds_its_own_get_jacobian():
    assert ti.CartLineConstraint.getJacobian is not ti.ConstraintSet.getJacobian
