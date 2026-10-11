"""ConstraintSet.getJacobian() before linkWithVariables raises ConstraintNotLinkedError (#202).

Every built-in trajopt_ifopt constraint sizes its Jacobian from the linked variable set,
``Jacobian jac(rows_, variables_->getRows())`` (trajopt 0.35.0, e.g.
joint_position_constraint.cpp:165), with no null check: unlinked, the call dereferenced a null
pointer and killed the interpreter. No other bound member reads ``variables_``. The binding
checks the link before calling C++. A Python subclass's own getJacobian shadows the bound one,
so the check never reaches it.

Each built-in runs in a child interpreter, so a regression fails its test, not the session.
"""

from __future__ import annotations

import subprocess
import sys

import pytest

from tesseract_robotics import trajopt_ifopt as ti

# s: a child loads the iiwa environment and builds one constraint; about 1 s on an M-series Mac
CHILD_TIMEOUT_S = 60.0
# waypoints in the child's trajectory: JointJerkConstraint needs 6, JointAccelConstraint 4
N_WAYPOINTS = 6
# m: CartLineConstraint's segment along x; any nonzero length (equal endpoints are invalid)
LINE_LENGTH = 0.1
# rows of a twist error: 3 linear + 3 angular
TWIST_SIZE = 6

_CHILD_SCRIPT = f"""\
import sys
from pathlib import Path

import numpy as np

from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics.tesseract_collision import CollisionEvaluatorType
from tesseract_robotics.tesseract_common import GeneralResourceLocator, Isometry3d
from tesseract_robotics.tesseract_environment import Environment

locator = GeneralResourceLocator()
urdf = locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf")
srdf = locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.srdf")
env = Environment()
assert env.init(Path(urdf.getFilePath()), Path(srdf.getFilePath()), locator)
manip = env.getKinematicGroup("manipulator")
joint_names = list(manip.getJointNames())
n = len(joint_names)
nodes = ti.createNodesVariables(
    "trajectory",
    joint_names,
    [np.full(n, 0.1 * i) for i in range({N_WAYPOINTS})],
    ti.toBounds(manip.getLimits().joint_limits),
)
v = [node.getVar("joints") for node in nodes.getNodes()]
identity = Isometry3d.Identity()
LINE_LENGTH = {LINE_LENGTH}
TWIST_SIZE = {TWIST_SIZE}


def cart_line():
    info = ti.CartLineInfo()
    info.manip = manip
    info.source_frame = "tool0"
    info.target_frame = "base_link"
    info.source_frame_offset = identity
    info.target_frame_offset1 = identity
    line_end = np.eye(4)
    line_end[0, 3] = LINE_LENGTH
    info.target_frame_offset2 = Isometry3d(line_end)
    # All six twist rows: trajopt 0.35.0's analytic Jacobian iterates six rows whatever the
    # indices' length (cartesian_line_constraint.cpp:236-243), so fewer corrupts the heap.
    info.indices = np.arange(TWIST_SIZE)
    return ti.CartLineConstraint(info, v[0], np.ones(TWIST_SIZE))


def inverse_kinematics():
    info = ti.InverseKinematicsInfo()
    info.manip = manip
    info.working_frame = "base_link"
    info.tcp_frame = "tool0"
    info.tcp_offset = identity
    return ti.InverseKinematicsConstraint(manip.calcFwdKin(np.zeros(n))["tool0"], info, v[0], v[1])


def discrete_evaluator():
    return ti.SingleTimestepCollisionEvaluator(manip, env, ti.TrajOptCollisionConfig(0.05, 20.0), True)


def lvs_evaluator():
    config = ti.TrajOptCollisionConfig(0.05, 20.0)
    config.collision_check_config.type = CollisionEvaluatorType.LVS_DISCRETE
    return ti.LVSDiscreteCollisionEvaluator(manip, env, config, True)


BUILDERS = {{
    "JointPosConstraint": lambda: ti.JointPosConstraint(np.zeros(n), v[0], np.ones(n)),
    "JointVelConstraint": lambda: ti.JointVelConstraint(np.zeros(n), v[:2], np.ones(n)),
    "JointAccelConstraint": lambda: ti.JointAccelConstraint(np.zeros(n), v[:4], np.ones(n)),
    "JointJerkConstraint": lambda: ti.JointJerkConstraint(np.zeros(n), v[:6], np.ones(n)),
    "CartPosConstraint": lambda: ti.CartPosConstraint(v[0], manip, "tool0", "base_link", identity, identity),
    "CartLineConstraint": cart_line,
    "InverseKinematicsConstraint": inverse_kinematics,
    "DiscreteCollisionConstraint": lambda: ti.DiscreteCollisionConstraint(discrete_evaluator(), v[0]),
    "DiscreteCollisionNumericalConstraint": lambda: ti.DiscreteCollisionNumericalConstraint(
        discrete_evaluator(), v[0]
    ),
    "ContinuousCollisionConstraint": lambda: ti.ContinuousCollisionConstraint(lvs_evaluator(), v[0], v[1]),
}}

constraint = BUILDERS[sys.argv[1]]()
try:
    constraint.getJacobian()
except ti.ConstraintNotLinkedError as exc:
    print(f"RAISED {{constraint.getName()}}|{{exc}}", flush=True)
else:
    print("RETURNED", flush=True)
constraint.linkWithVariables(nodes)
constraint.update()  # as TrajOptQPProblem.setup() does: the collision sets read data it computes
shape = constraint.getJacobian().shape
print(f"LINKED {{shape[0]}} {{shape[1]}} {{constraint.getRows()}} {{nodes.getRows()}}", flush=True)
"""

BUILT_IN_CONSTRAINTS = (
    "JointPosConstraint",
    "JointVelConstraint",
    "JointAccelConstraint",
    "JointJerkConstraint",
    "CartPosConstraint",
    "CartLineConstraint",
    "InverseKinematicsConstraint",
    "DiscreteCollisionConstraint",
    "DiscreteCollisionNumericalConstraint",
    "ContinuousCollisionConstraint",
)


@pytest.mark.parametrize("constraint", BUILT_IN_CONSTRAINTS)
def test_built_in_constraint_unlinked_jacobian_raises(constraint):
    proc = subprocess.run(
        [sys.executable, "-c", _CHILD_SCRIPT, constraint],
        capture_output=True,
        text=True,
        check=False,
        timeout=CHILD_TIMEOUT_S,
    )
    traceback_at = proc.stderr.find("Traceback")
    assert proc.returncode == 0, (
        f"child died (rc={proc.returncode}, SIGSEGV is -11/139): "
        f"{proc.stderr[traceback_at:] if traceback_at >= 0 else proc.stderr}"
    )
    lines = proc.stdout.splitlines()
    assert lines[0].startswith("RAISED "), proc.stdout
    name, message = lines[0].removeprefix("RAISED ").split("|", 1)
    assert f"'{name}'" in message and "linkWithVariables" in message, message
    jac_rows, jac_cols, rows, n_vars = map(int, lines[1].removeprefix("LINKED ").split())
    assert (jac_rows, jac_cols) == (rows, n_vars)


def test_constraint_not_linked_error_is_a_runtime_error():
    assert issubclass(ti.ConstraintNotLinkedError, RuntimeError)
