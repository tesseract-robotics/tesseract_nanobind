"""Collision constraint getters before the first update() raise RuntimeError (#236, trajopt#617).

The collision data a collision constraint's getters read is computed by ``update()``. Before it,
trajopt 0.35.0 dereferenced a null pointer (SIGSEGV); the pinned trajopt throws
``std::runtime_error("<name>: collision data not computed, call update() first")``, which
nanobind raises as ``RuntimeError``. After ``update()`` the same call returns.

The calls run in a child interpreter: a regression to the null dereference kills the child, and
the test fails on its exit code instead of taking down the xdist worker.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

# s: a child imports the bindings and builds a dozen constraints on the two-box scene in ~2 s;
# a hang fails the test, not the session, with room for a loaded runner
CHILD_TIMEOUT_S = 60.0
REPO_ROOT = Path(__file__).resolve().parents[2]

# (class, getter): every getter that reads the collision data. The D variants' getValues()
# return cached values and never read it, so only their getJacobian() is listed.
CASES = [
    ("DiscreteCollisionConstraint", "getValues"),
    ("DiscreteCollisionConstraint", "getJacobian"),
    ("DiscreteCollisionNumericalConstraint", "getValues"),
    ("DiscreteCollisionNumericalConstraint", "getJacobian"),
    ("DiscreteCollisionNumericalConstraint", "calcValues"),
    ("DiscreteCollisionNumericalConstraint", "calcJacobianBlock"),
    ("DiscreteCollisionConstraintD", "getJacobian"),
    ("ContinuousCollisionConstraint", "getValues"),
    ("ContinuousCollisionConstraint", "getJacobian"),
    ("ContinuousCollisionNumericalConstraint", "getValues"),
    ("ContinuousCollisionNumericalConstraint", "getJacobian"),
    ("ContinuousCollisionConstraintD", "getJacobian"),
]

_CHILD_SCRIPT = """\
import json
import sys

from tesseract_robotics import trajopt_ifopt as ti
from tests.trajopt_ifopt.scenes import OVERLAP_X, two_boxes_at

SECOND_X = 0.15  # m: second slider position of the continuous pair


def build(cls_name):
    cls = getattr(ti, cls_name)
    if cls_name.startswith("Discrete"):
        boxes = two_boxes_at(OVERLAP_X)
        return boxes, cls(boxes.discrete_evaluator(), boxes.var, name=cls_name)
    boxes = two_boxes_at(OVERLAP_X, SECOND_X)
    var0, var1 = (node.getVar("joints") for node in boxes.nodes.getNodes())
    return boxes, cls(
        boxes.lvs_discrete_evaluator(), position_var0=var0, position_var1=var1, name=cls_name
    )


def call(constraint, boxes, getter):
    method = getattr(constraint, getter)
    return method(boxes.var.value()) if getter.startswith("calc") else method()


results = []
for cls_name, getter in json.loads(sys.argv[1]):
    boxes, constraint = build(cls_name)
    constraint.linkWithVariables(boxes.nodes)
    try:
        call(constraint, boxes, getter)
        before = None
    except RuntimeError as error:
        before = [type(error).__name__, str(error)]
    constraint.update()
    call(constraint, boxes, getter)
    results.append([cls_name, getter, before])
print(json.dumps(results))
"""


@pytest.fixture(scope="module")
def child_results() -> dict[tuple[str, str], list[str] | None]:
    child = subprocess.run(
        [sys.executable, "-c", _CHILD_SCRIPT, json.dumps(CASES)],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        timeout=CHILD_TIMEOUT_S,
        check=False,
    )
    assert child.returncode == 0, (
        f"child exited {child.returncode} (negative = killed by that signal)\n{child.stderr}"
    )
    return {(cls, getter): before for cls, getter, before in json.loads(child.stdout)}


@pytest.mark.parametrize(("cls_name", "getter"), CASES)
def test_collision_getters_before_update_raise(child_results, cls_name, getter):
    before = child_results[(cls_name, getter)]
    assert before is not None, f"{cls_name}.{getter}() returned before update()"
    error_type, message = before
    assert error_type == "RuntimeError"
    assert message.startswith(cls_name)
    assert "call update() first" in message
