"""Python builds variable sets: Var and NodesVariables constructors (#297).

NodesVariables takes its Nodes by unique_ptr: the Python Node objects hand over ownership and
become unusable, which keeps each Var's parent pointer valid. A standalone Var (no parent) works
in constraints that read only its index and values; the collision constraints read the parent
chain to the variable set, so they reject a Var that is not in one.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from tesseract_robotics import trajopt_ifopt as ti

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
