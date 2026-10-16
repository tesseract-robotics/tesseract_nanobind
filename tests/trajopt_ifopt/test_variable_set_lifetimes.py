"""Objects read out of a variable set or a constraint keep their owner alive (#297).

A Var points at its Node and a Node at its NodesVariables through raw C++ pointers, so every
getter that hands one out ties the result to the object it came from: a Var read out of a set
outlives the set, and its parent chain stays readable. A Node handed to NodesVariables is owned
by the set from then on; the caller's own reference to it is unusable, and the set's getters
return working Nodes even while the caller still holds it.

The evaluators' getJointGroup, CartLineConstraint.getInfo and
InverseKinematicsConstraint.getKinematicInfo return references into their owner, which the
result keeps alive too.

Each scenario runs in a child interpreter, so a use-after-free crashes the child, not the
session. On macOS MallocScribble fills freed memory, so a read through a dangling pointer
returns garbage instead of the old value.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

# s: a child loads at most one environment; about 1 s on an M-series Mac
CHILD_TIMEOUT_S = 60.0
REPO_ROOT = Path(__file__).resolve().parents[2]

_PRELUDE = """\
import gc
import numpy as np
from tesseract_robotics import trajopt_ifopt as ti

def node(name):
    n = ti.Node(name)
    n.addVar("joints", ["a", "b"], np.array([0.1, 0.2]), [ti.Bounds(-1.0, 1.0)] * 2)
    return n

def churn():
    # reuse the freed memory, so a dangling pointer reads someone else's data
    gc.collect()
    return [ti.NodesVariables("x" * 64, [node("z" * 64)]) for _ in range(200)]
"""

SCENARIOS = {
    "var_from_set_outlives_set": """
nv = ti.NodesVariables("set", [node("n0")])
var = nv.getNodes()[0].getVar("joints")
del nv
junk = churn()
parent = var.getParent()
print(parent.getName(), parent.getParent().getName())
""",
    "var_from_get_node_outlives_set": """
nv = ti.createNodesVariables("set", ["a", "b"], [np.zeros(2)], [ti.Bounds(-1.0, 1.0)] * 2)
var = nv.getNode(0).getVar("joints")
del nv
junk = churn()
parent = var.getParent()
print(parent.getName(), parent.getParent().getName())
""",
    "node_outlives_set": """
nv = ti.NodesVariables("set", [node("n0")])
n = nv.getNode(0)
del nv
junk = churn()
print(n.getName(), n.getParent().getName())
""",
    "var_added_before_handover_outlives_set": """
n0 = node("n0")
var = n0.getVar("joints")
nv = ti.NodesVariables("set", [n0])
del nv, n0
junk = churn()
parent = var.getParent()
print(parent.getName(), parent.getParent().getName())
""",
    "set_getters_while_caller_keeps_node": """
n0 = node("n0")
nv = ti.NodesVariables("set", [n0])
names = [n.getName() for n in nv.getNodes()]
var = nv.getNode(0).getVar("joints")
print(nv.getNode(0).getName(), names[0], var.getParent().getName())
""",
    "evaluator_joint_group_outlives_evaluator": """
from tests.trajopt_ifopt.scenes import OVERLAP_X, two_boxes_at
boxes = two_boxes_at(OVERLAP_X)
evaluator = boxes.discrete_evaluator()
group = evaluator.getJointGroup()
del evaluator
junk = churn()
print(*group.getJointNames())
""",
    "continuous_evaluator_joint_group_outlives_evaluator": """
from tests.trajopt_ifopt.scenes import OVERLAP_X, two_boxes_at
boxes = two_boxes_at(OVERLAP_X, OVERLAP_X)
evaluator = boxes.lvs_discrete_evaluator()
group = evaluator.getJointGroup()
del evaluator
junk = churn()
print(*group.getJointNames())
""",
    "cart_line_info_outlives_constraint": """
from tesseract_robotics.tesseract_common import Isometry3d
from tests.trajopt_ifopt.scenes import load_iiwa
iiwa = load_iiwa()
var = iiwa.nodes(np.zeros(len(iiwa.joint_names))).getNodes()[0].getVar("joints")
end = np.eye(4)
end[0, 3] = 0.1
info = ti.CartLineInfo(iiwa.manip, "tool0", "base_link", Isometry3d.Identity(), Isometry3d(end))
constraint = ti.CartLineConstraint(info, var, np.ones(6))
held = constraint.getInfo()
del constraint, info
junk = churn()
print(held.source_frame, held.target_frame, len(held.manip.getJointNames()))
""",
    "ik_info_outlives_constraint": """
from tesseract_robotics.tesseract_common import Isometry3d
from tests.trajopt_ifopt.scenes import load_iiwa
iiwa = load_iiwa()
n = len(iiwa.joint_names)
nodes = iiwa.nodes(np.zeros(n), np.zeros(n))
constraint_var, seed_var = (node.getVar("joints") for node in nodes.getNodes())
info = ti.InverseKinematicsInfo(iiwa.manip, "base_link", "tool0")
constraint = ti.InverseKinematicsConstraint(
    Isometry3d.Identity(), info, constraint_var, seed_var
)
held = constraint.getKinematicInfo()
del constraint, info
junk = churn()
print(held.working_frame, held.tcp_frame, len(held.manip.getJointNames()))
""",
}

EXPECTED = {
    "var_from_set_outlives_set": "n0 set",
    "var_from_get_node_outlives_set": "set_0 set",
    "node_outlives_set": "n0 set",
    "var_added_before_handover_outlives_set": "n0 set",
    "set_getters_while_caller_keeps_node": "n0 n0 n0",
    "evaluator_joint_group_outlives_evaluator": "slider_joint",
    "continuous_evaluator_joint_group_outlives_evaluator": "slider_joint",
    "cart_line_info_outlives_constraint": "tool0 base_link 7",
    "ik_info_outlives_constraint": "base_link tool0 7",
}


def _run_child(body: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, "-c", _PRELUDE + body],
        capture_output=True,
        text=True,
        check=False,
        timeout=CHILD_TIMEOUT_S,
        cwd=REPO_ROOT,
        # TRAJOPT_LOG_THRESH set: trajopt otherwise prints a hint on stdout
        env={**os.environ, "MallocScribble": "1", "TRAJOPT_LOG_THRESH": "ERROR"},
    )


@pytest.mark.parametrize("scenario", sorted(SCENARIOS))
def test_result_keeps_its_owner_alive(scenario: str):
    proc = _run_child(SCENARIOS[scenario])
    traceback_at = proc.stderr.find("Traceback")
    assert proc.returncode == 0, (
        f"child failed (rc={proc.returncode}, SIGSEGV is -11/139): "
        f"{proc.stderr[traceback_at:] if traceback_at >= 0 else proc.stderr}"
    )
    assert proc.stdout.strip() == EXPECTED[scenario]


def test_a_kept_node_is_unusable_after_handover():
    proc = _run_child(
        """
import warnings
n0 = node("n0")
nv = ti.NodesVariables("set", [n0])
with warnings.catch_warnings(record=True) as caught:
    warnings.simplefilter("always")
    try:
        n0.getName()
    except TypeError:
        print("RAISED", *(str(w.message) for w in caught))
"""
    )
    assert proc.returncode == 0, proc.stderr
    # nanobind warns on the relinquished instance, then finds no overload for it
    assert proc.stdout.startswith("RAISED"), proc.stdout
    assert "relinquished" in proc.stdout


def test_handing_the_same_node_over_twice_raises():
    proc = _run_child(
        """
n0 = node("n0")
try:
    ti.NodesVariables("set", [n0, n0])
except ti.NodeOwnershipError as error:
    print("RAISED", error)
print(n0.getName())
"""
    )
    assert proc.returncode == 0, proc.stderr
    raised, name = proc.stdout.splitlines()
    assert raised.startswith("RAISED") and "'n0'" in raised, raised
    assert name == "n0", "a rejected handover leaves the caller's Node usable"


def test_a_node_from_a_set_cannot_be_handed_over_again():
    proc = _run_child(
        """
nv = ti.NodesVariables("set", [node("n0")])
try:
    ti.NodesVariables("other", [nv.getNode(0)])
except ti.NodeOwnershipError as error:
    print("RAISED", error)
print(nv.getNode(0).getName(), nv.getRows())
"""
    )
    assert proc.returncode == 0, proc.stderr
    raised, state = proc.stdout.splitlines()
    assert raised.startswith("RAISED"), raised
    assert state == "n0 2"
