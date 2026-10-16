"""Component and Differentiable carry their pure-virtual interface (#287).

getValues / getBounds are declared on Component, getJacobian and Mode on Differentiable. Bound
on the bases, a caller typed on the base reaches them on every subclass.
"""

from __future__ import annotations

import numpy as np
import pytest

from tesseract_robotics import trajopt_ifopt as ti

from .scenes import Iiwa


@pytest.fixture
def nodes(iiwa: Iiwa) -> ti.NodesVariables:
    n = len(iiwa.joint_names)
    return iiwa.nodes(np.zeros(n), np.full(n, 0.1))


@pytest.fixture
def joint_pos(nodes: ti.NodesVariables) -> ti.JointPosConstraint:
    var = nodes.getNodes()[1].getVar("joints")
    n = var.size()
    constraint = ti.JointPosConstraint(np.zeros(n), var, np.ones(n), "JointPos")
    constraint.linkWithVariables(nodes)
    return constraint


def _limits(bounds):
    return [(b.getLower(), b.getUpper()) for b in bounds]


def test_component_reaches_nodes_variables(nodes: ti.NodesVariables):
    np.testing.assert_array_equal(ti.Component.getValues(nodes), nodes.getValues())
    assert _limits(ti.Component.getBounds(nodes)) == _limits(nodes.getBounds())


def test_component_reaches_a_constraint(joint_pos: ti.JointPosConstraint):
    np.testing.assert_array_equal(ti.Component.getValues(joint_pos), joint_pos.getValues())
    assert _limits(ti.Component.getBounds(joint_pos)) == _limits(joint_pos.getBounds())


def test_differentiable_reaches_a_constraint_jacobian(joint_pos: ti.JointPosConstraint):
    through_base = ti.Differentiable.getJacobian(joint_pos)
    np.testing.assert_array_equal(through_base.toarray(), joint_pos.getJacobian().toarray())


def test_differentiable_jacobian_checks_the_link(iiwa: Iiwa):
    n = len(iiwa.joint_names)
    var = iiwa.nodes(np.zeros(n)).getNodes()[0].getVar("joints")
    unlinked = ti.JointPosConstraint(np.zeros(n), var, np.ones(n), "JointPos")
    with pytest.raises(ti.ConstraintNotLinkedError):
        ti.Differentiable.getJacobian(unlinked)


def test_mode_enum():
    assert {m.name for m in ti.Differentiable.Mode} == {"STACK_ROWS", "SUM_TO_SCALAR"}
    assert ti.Differentiable.Mode.STACK_ROWS != ti.Differentiable.Mode.SUM_TO_SCALAR
