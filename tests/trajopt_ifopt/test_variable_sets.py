"""Python builds variable sets: Var and NodesVariables constructors (#297).

NodesVariables takes its Nodes by unique_ptr: the Python Node objects hand over ownership and
become unusable, which keeps each Var's parent pointer valid. A constraint indexes the decision
vector through its Vars, so linking it to a variable set (linkWithVariables, or adding it to a
problem) rejects a Var outside that set: a standalone Var, or one from another set, would put
Jacobian columns out of range. The collision constraints read the parent chain to the variable
set from construction on, so they reject such a Var already when built.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics import trajopt_sqp as tsqp

from .scenes import TwoBoxes


def _node(name: str, offset: float) -> ti.Node:
    node = ti.Node(name)
    node.addVar(
        "joints", ["j0", "j1"], np.array([offset, offset + 0.1]), [ti.Bounds(-1.0, 1.0)] * 2
    )
    node.addVar("time", offset + 0.5, ti.Bounds(0.0, 2.0))
    return node


def test_nodes_variables_from_nodes():
    nodes = ti.NodesVariables("trajectory", [_node("n0", 0.0), _node("n1", 0.2)])
    assert nodes.getName() == "trajectory"
    assert nodes.getRows() == 6
    np.testing.assert_allclose(nodes.getValues(), [0.0, 0.1, 0.5, 0.2, 0.3, 0.7])

    first, second = nodes.getNodes()
    assert first.getParent() is not None
    var = second.getVar("joints")
    assert var.getIndex() == 3
    node = var.getParent()
    assert node is not None
    assert node.getName() == "n1"
    owner = node.getParent()
    assert owner is not None
    assert owner.getName() == "trajectory"


def test_a_handed_over_node_is_unusable():
    node = _node("n0", 0.0)
    ti.NodesVariables("trajectory", [node])
    # nanobind warns on the relinquished instance, then finds no overload for it
    with pytest.warns(RuntimeWarning, match="relinquished"), pytest.raises(TypeError):
        node.getName()


def test_standalone_scalar_var():
    var = ti.Var(2, "time", 0.5, ti.Bounds(0.0, 1.0))
    assert var.getIndex() == 2
    assert var.getIdentifier() == "time"
    assert var.name() == ["time"]
    np.testing.assert_array_equal(var.value(), [0.5])
    assert var.getParent() is None

    unbounded = ti.Var(0, "free", 3.0)
    assert unbounded.bounds()[0].getLower() == -math.inf


def test_standalone_vector_var():
    var = ti.Var(1, "joints", ["a", "b"], np.array([0.2, 0.3]), [ti.Bounds(-1.0, 1.0)] * 2)
    assert var.getIndex() == 1
    assert var.size() == 2
    assert var.name() == ["a", "b"]
    assert var.getParent() is None


def test_standalone_var_in_a_joint_constraint():
    var = ti.Var(0, "joints", ["a", "b"], np.array([0.2, 0.3]), [ti.Bounds(-1.0, 1.0)] * 2)
    constraint = ti.JointPosConstraint(np.zeros(2), var, np.ones(2))
    np.testing.assert_allclose(constraint.getValues(), [0.2, 0.3])


def test_collision_constraint_rejects_a_var_without_a_variable_set(two_boxes: TwoBoxes):
    loose = ti.Var(0, "joints", ["slider_joint"], np.array([0.1]), [ti.Bounds(-1.0, 1.0)])
    with pytest.raises(ti.VarNotInVariableSetError):
        ti.DiscreteCollisionConstraint(two_boxes.discrete_evaluator(), loose)
    with pytest.raises(ti.VarNotInVariableSetError):
        ti.DiscreteCollisionConstraintD(two_boxes.discrete_evaluator(), loose)


def _two_joint_set(name: str) -> ti.NodesVariables:
    return ti.createNodesVariables(name, ["a", "b"], [np.zeros(2)], [ti.Bounds(-1.0, 1.0)] * 2)


def test_linking_a_standalone_var_raises():
    var = ti.Var(100, "joints", ["a", "b"], np.array([0.2, 0.3]), [ti.Bounds(-1.0, 1.0)] * 2)
    constraint = ti.JointPosConstraint(np.zeros(2), var, np.ones(2))
    with pytest.raises(ti.VarNotInVariableSetError, match="'JointPos'.*'joints'"):
        constraint.linkWithVariables(_two_joint_set("set"))


def test_linking_a_var_of_another_set_raises():
    other = _two_joint_set("other")
    var = other.getNode(0).getVar("joints")
    constraint = ti.JointVelConstraint(np.zeros(2), [var, var], np.ones(2))
    with pytest.raises(ti.VarNotInVariableSetError):
        constraint.linkWithVariables(_two_joint_set("set"))
    constraint.linkWithVariables(other)
    assert constraint.getJacobian().shape == (constraint.getRows(), other.getRows())


def test_adding_a_constraint_on_a_standalone_var_to_a_problem_raises():
    var = ti.Var(100, "joints", ["a", "b"], np.array([0.2, 0.3]), [ti.Bounds(-1.0, 1.0)] * 2)
    problem = tsqp.TrajOptQPProblem(_two_joint_set("set"))
    with pytest.raises(ti.VarNotInVariableSetError):
        problem.addConstraintSet(ti.JointPosConstraint(np.zeros(2), var, np.ones(2)))
    with pytest.raises(ti.VarNotInVariableSetError):
        problem.addCostSet(
            ti.JointPosConstraint(np.zeros(2), var, np.ones(2)), tsqp.CostPenaltyType.SQUARED
        )
