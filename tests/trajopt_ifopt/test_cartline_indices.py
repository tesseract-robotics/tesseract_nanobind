"""CartLineConstraint with fewer than 6 indices: one Jacobian row per index (#237).

trajopt 0.35.0 sized the Jacobian by ``indices.size()`` rows but filled six, so fewer than six
indices wrote past the matrix and corrupted the heap (trajopt#618). trajopt#612 fills one row per
index; the pinned trajopt has it. The rows an index set names are then exactly those rows of the
six-index Jacobian, values included: both branches compute the full twist error and select rows.

Each differentiation runs in a child interpreter: on a trajopt without the fix the call corrupts
the heap, which crashes the child rather than the test worker.
"""

from __future__ import annotations

import json
import subprocess
import sys

import numpy as np
import pytest

from tesseract_robotics import trajopt_ifopt as ti

# s: a child loads the iiwa environment and builds nine constraints; about 1 s on an M-series Mac
CHILD_TIMEOUT_S = 60.0
# m: the line's length along x; any nonzero length (equal endpoints are invalid)
LINE_LENGTH = 0.1
# rows of a twist error: 3 linear + 3 angular
TWIST_SIZE = 6
# rad: every joint off zero, so no Jacobian column vanishes
JOINT_VALUE = 0.1
# 1 to 5 rows: linear only, angular only, mixed, non-contiguous and out of order
PARTIAL_INDICES = (
    (0,),
    (5,),
    (0, 1),
    (5, 2),
    (0, 1, 2),
    (3, 4, 5),
    (0, 2, 3, 5),
    (1, 2, 3, 4, 5),
)
# a partial index set: the linear rows only
LINEAR_INDICES = (0, 1, 2)

_CHILD_SCRIPT = f"""\
import json
import sys
from pathlib import Path

import numpy as np

from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics.tesseract_common import GeneralResourceLocator, Isometry3d
from tesseract_robotics.tesseract_environment import Environment

numeric, link = sys.argv[1] == "numeric", sys.argv[2] == "linked"
index_sets = json.loads(sys.argv[3])

locator = GeneralResourceLocator()
urdf = locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf")
srdf = locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.srdf")
env = Environment()
assert env.init(Path(urdf.getFilePath()), Path(srdf.getFilePath()), locator)
manip = env.getKinematicGroup("manipulator")
joint_names = [str(j) for j in manip.getJointIds()]
n = len(joint_names)
nodes = ti.createNodesVariables(
    "trajectory", joint_names, [np.full(n, {JOINT_VALUE})], ti.toBounds(manip.getLimits().joint_limits)
)
var = nodes.getNodes()[0].getVar("joints")
identity = Isometry3d.Identity()
line_end = np.eye(4)
line_end[0, 3] = {LINE_LENGTH}

results = []
for indices in index_sets:
    info = ti.CartLineInfo()
    info.manip = manip
    info.source_frame = "tool0"
    info.target_frame = "base_link"
    info.source_frame_offset = identity
    info.target_frame_offset1 = identity
    info.target_frame_offset2 = Isometry3d(line_end)
    info.indices = np.asarray(indices)
    constraint = ti.CartLineConstraint(info, var, np.ones(len(indices)))
    constraint.use_numeric_differentiation = numeric
    if not link:
        try:
            constraint.getJacobian()
        except Exception as exc:
            print(f"RAISED {{type(exc).__name__}}|{{exc}}", flush=True)
        else:
            print("RETURNED", flush=True)
        sys.exit(0)
    constraint.linkWithVariables(nodes)
    results.append({{
        "values": constraint.getValues().tolist(),
        "jacobian": constraint.getJacobian().toarray().tolist(),
        "block": constraint.calcJacobianBlock(var.value()).toarray().tolist(),
    }})
print(json.dumps(results), flush=True)
"""


def _run_child(differentiation: str, link: str, index_sets) -> list[str]:
    proc = subprocess.run(
        [sys.executable, "-c", _CHILD_SCRIPT, differentiation, link, json.dumps(index_sets)],
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
def test_partial_indices_are_the_rows_they_name(differentiation):
    full_indices = tuple(range(TWIST_SIZE))
    (line,) = _run_child(differentiation, "linked", [full_indices, *PARTIAL_INDICES])
    full, *partials = json.loads(line)
    full_jacobian = np.array(full["jacobian"])
    assert full_jacobian.shape[0] == TWIST_SIZE
    assert np.all(np.any(full_jacobian != 0.0, axis=1)), full_jacobian
    np.testing.assert_array_equal(np.array(full["block"]), full_jacobian)
    for indices, partial in zip(PARTIAL_INDICES, partials, strict=True):
        rows = list(indices)
        np.testing.assert_array_equal(
            np.array(partial["values"]), np.array(full["values"])[rows], err_msg=str(indices)
        )
        np.testing.assert_array_equal(
            np.array(partial["jacobian"]), full_jacobian[rows], err_msg=str(indices)
        )
        np.testing.assert_array_equal(
            np.array(partial["block"]), full_jacobian[rows], err_msg=str(indices)
        )


def test_unlinked_check_comes_first():
    lines = _run_child("numeric", "unlinked", [LINEAR_INDICES])
    assert lines[0].startswith("RAISED ConstraintNotLinkedError|"), lines


def test_cartline_constraint_binds_its_own_get_jacobian():
    assert ti.CartLineConstraint.getJacobian is not ti.ConstraintSet.getJacobian
