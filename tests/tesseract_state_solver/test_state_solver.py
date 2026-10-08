"""`StateSolver` native overloads, their joint-name validation and `insertSceneGraph` (gh-219).

Upstream `OFKTStateSolver` dereferences `nodes_[name]` for an unknown joint name (the GH #43
null `unique_ptr`, ofkt_state_solver.cpp:226, :246, :268); `KDLStateSolver` logs and skips
it. The binding validates first, so both raise `ValueError`.
"""

from __future__ import annotations

import numpy as np
import pytest

import tesseract_robotics.tesseract_scene_graph as sg
from tesseract_robotics.tesseract_common import Isometry3d, Quaterniond
from tesseract_robotics.tesseract_state_solver import KDLStateSolver, OFKTStateSolver

SOLVERS = {"ofkt": OFKTStateSolver, "kdl": KDLStateSolver}
ACTIVE_JOINTS = ["joint_1", "joint_2", "joint_3"]
FLOATING_JOINT = "joint_f"
FLOATING_CHILD = "link_f"
TIP_LINK = "link_3"
# A joint position inside every test joint's limits (-2, 2), away from zero so links move.
JOINT_VALUES = np.array([0.3, -0.4, 0.5])
# Transforms are built from exact decimal inputs and composed a few times in double
# precision: agreement to ~1e-15 relative; 1e-12 leaves margin without hiding a wrong overload.
TRANSFORM_ATOL = 1e-12


def _translation(x: float, y: float, z: float) -> Isometry3d:
    return Isometry3d(np.array([x, y, z]), Quaterniond.Identity())


def _joint(name, parent, child, joint_type, axis, origin):
    joint = sg.Joint(name)
    joint.parent_link_name = parent
    joint.child_link_name = child
    joint.type = joint_type
    joint.axis = np.array(axis, dtype=float)
    joint.parent_to_joint_origin_transform = origin
    if joint_type != sg.JointType.FLOATING:
        joint.limits = sg.JointLimits(-2, 2, 1, 1, 1, 1)
    return joint


def _graph() -> sg.SceneGraph:
    """base_link -joint_1 (rev z)- link_1 -joint_2 (rev y)- link_2 -joint_3 (prismatic x)- link_3,
    and base_link -joint_f (floating)- link_f."""
    graph = sg.SceneGraph("state_solver_test")
    for name in ["base_link", "link_1", "link_2", "link_3", FLOATING_CHILD]:
        assert graph.addLink(sg.Link(name))
    for joint in [
        _joint(
            "joint_1",
            "base_link",
            "link_1",
            sg.JointType.REVOLUTE,
            [0, 0, 1],
            _translation(0, 0, 0.5),
        ),
        _joint(
            "joint_2", "link_1", "link_2", sg.JointType.REVOLUTE, [0, 1, 0], _translation(0, 0, 1.0)
        ),
        _joint(
            "joint_3",
            "link_2",
            "link_3",
            sg.JointType.PRISMATIC,
            [1, 0, 0],
            _translation(1.0, 0, 0),
        ),
        _joint(
            FLOATING_JOINT,
            "base_link",
            FLOATING_CHILD,
            sg.JointType.FLOATING,
            [0, 0, 1],
            _translation(0, 2.0, 0),
        ),
    ]:
        assert graph.addJoint(joint)
    return graph


def _solver(kind: str):
    return SOLVERS[kind](_graph())


def _assert_transforms_equal(actual: dict, expected: dict) -> None:
    assert set(actual) == set(expected)
    for name, transform in expected.items():
        np.testing.assert_allclose(
            actual[name].matrix, transform.matrix, atol=TRANSFORM_ATOL, err_msg=name
        )


def _assert_states_equal(actual, expected) -> None:
    assert actual.joints == pytest.approx(expected.joints)
    _assert_transforms_equal(actual.link_transforms, expected.link_transforms)


@pytest.fixture(params=sorted(SOLVERS))
def solver(request):
    return _solver(request.param)


# --- native overloads vs aliases --------------------------------------------------------------


def test_set_state_map_matches_alias_and_moves_links(solver):
    """Review focus: setState({name: float}) reaches the joint-value map overload."""
    before = solver.getState().link_transforms[TIP_LINK].translation.copy()
    alias = solver.clone()
    joints = dict(zip(ACTIVE_JOINTS, JOINT_VALUES))

    solver.setState(joints)
    alias.setStateByMap(joints)

    assert solver.getState().joints == pytest.approx(joints)
    assert not np.allclose(solver.getState().link_transforms[TIP_LINK].translation, before)
    _assert_states_equal(solver.getState(), alias.getState())


def test_set_state_names_values_matches_alias(solver):
    alias = solver.clone()
    names = list(reversed(ACTIVE_JOINTS))
    values = JOINT_VALUES[::-1].copy()

    solver.setState(names, values)
    alias.setStateByNamesAndValues(names, values)

    assert solver.getState().joints == pytest.approx(dict(zip(ACTIVE_JOINTS, JOINT_VALUES)))
    _assert_states_equal(solver.getState(), alias.getState())


def test_set_state_vector_matches_map(solver):
    by_map = solver.clone()
    solver.setState(JOINT_VALUES)
    by_map.setState(dict(zip(ACTIVE_JOINTS, JOINT_VALUES)))
    _assert_states_equal(solver.getState(), by_map.getState())


def test_get_state_overloads_agree(solver):
    expected = solver.getState(JOINT_VALUES)
    _assert_states_equal(solver.getState(dict(zip(ACTIVE_JOINTS, JOINT_VALUES))), expected)
    _assert_states_equal(solver.getState(ACTIVE_JOINTS, JOINT_VALUES), expected)
    # getState(...) leaves the solver's own state alone
    assert solver.getState().joints == pytest.approx(dict.fromkeys(ACTIVE_JOINTS, 0.0))


# --- unknown names ----------------------------------------------------------------------------

UNKNOWN_NAME_CALLS = {
    "setState-map": lambda s: s.setState({"nope": 1.0}),
    "setState-names": lambda s: s.setState(["nope"], np.array([1.0])),
    "setStateByMap": lambda s: s.setStateByMap({"nope": 1.0}),
    "setStateByNamesAndValues": lambda s: s.setStateByNamesAndValues(["nope"], np.array([1.0])),
    "getState-map": lambda s: s.getState({"nope": 1.0}),
    "getState-names": lambda s: s.getState(["nope"], np.array([1.0])),
    "getJacobian-map": lambda s: s.getJacobian({"nope": 1.0}, TIP_LINK),
    "getJacobian-names": lambda s: s.getJacobian(["nope"], np.array([1.0]), TIP_LINK),
    "getLinkTransforms": lambda s: s.getLinkTransforms(["nope"], np.array([1.0])),
    "setState-floating": lambda s: s.setState({"nope": _translation(1, 0, 0)}),
    "setState-vector-floating": lambda s: s.setState(np.zeros(3), {"nope": _translation(1, 0, 0)}),
    "getState-floating": lambda s: s.getState({"nope": _translation(1, 0, 0)}),
}


@pytest.mark.parametrize("call", sorted(UNKNOWN_NAME_CALLS))
def test_unknown_joint_name_raises_value_error(solver, call):
    """Every alias shares its native lambda, so the aliases are in the table too. Before the
    fix these ran in a subprocess: on OFKT the aliases died with SIGSEGV."""
    before = solver.getState()
    with pytest.raises(ValueError, match="nope"):
        UNKNOWN_NAME_CALLS[call](solver)
    _assert_states_equal(solver.getState(), before)


# --- lengths ----------------------------------------------------------------------------------

WRONG_LENGTH_CALLS = {
    "setState-vector": lambda s: s.setState(np.zeros(len(ACTIVE_JOINTS) + 1)),
    "setState-names": lambda s: s.setState(ACTIVE_JOINTS, JOINT_VALUES[:-1]),
    "setStateByNamesAndValues": lambda s: s.setStateByNamesAndValues(
        ACTIVE_JOINTS, JOINT_VALUES[:-1]
    ),
    "getState-vector": lambda s: s.getState(np.zeros(len(ACTIVE_JOINTS) + 1)),
    "getState-names": lambda s: s.getState(ACTIVE_JOINTS, JOINT_VALUES[:-1]),
    "getJacobian-vector": lambda s: s.getJacobian(np.zeros(len(ACTIVE_JOINTS) + 1), TIP_LINK),
    "getJacobian-names": lambda s: s.getJacobian(ACTIVE_JOINTS, JOINT_VALUES[:-1], TIP_LINK),
    "getLinkTransforms": lambda s: s.getLinkTransforms(ACTIVE_JOINTS, JOINT_VALUES[:-1]),
}


@pytest.mark.parametrize("call", sorted(WRONG_LENGTH_CALLS))
def test_wrong_length_raises_value_error(solver, call):
    before = solver.getState()
    with pytest.raises(ValueError, match="length"):
        WRONG_LENGTH_CALLS[call](solver)
    _assert_states_equal(solver.getState(), before)


# --- getJacobian / getLinkTransforms ---------------------------------------------------------


def test_get_jacobian_overloads_agree(solver):
    expected = solver.getJacobian(JOINT_VALUES, TIP_LINK)
    assert expected.shape == (6, len(ACTIVE_JOINTS))
    names = list(reversed(ACTIVE_JOINTS))
    np.testing.assert_allclose(
        solver.getJacobian(names, JOINT_VALUES[::-1].copy(), TIP_LINK), expected
    )
    np.testing.assert_allclose(
        solver.getJacobian(dict(zip(ACTIVE_JOINTS, JOINT_VALUES)), TIP_LINK), expected
    )


UNKNOWN_LINK_CALLS = {
    "vector": lambda s: s.getJacobian(JOINT_VALUES, "no_link"),
    "map": lambda s: s.getJacobian(dict(zip(ACTIVE_JOINTS, JOINT_VALUES)), "no_link"),
    "names": lambda s: s.getJacobian(ACTIVE_JOINTS, JOINT_VALUES, "no_link"),
}


@pytest.mark.parametrize("call", sorted(UNKNOWN_LINK_CALLS))
def test_get_jacobian_unknown_link_raises_key_error(solver, call):
    with pytest.raises(KeyError, match="no_link"):
        UNKNOWN_LINK_CALLS[call](solver)


def test_get_link_transforms_current_state(solver):
    """getLinkTransforms() lists the current transforms in getLinkNames() order."""
    solver.setState(JOINT_VALUES)
    current = solver.getState().link_transforms
    transforms = solver.getLinkTransforms()
    assert len(transforms) == len(solver.getLinkNames())
    for name, transform in zip(solver.getLinkNames(), transforms):
        np.testing.assert_allclose(transform.matrix, current[name].matrix, atol=TRANSFORM_ATOL)


def test_get_link_transforms_matches_get_state(solver):
    transforms = solver.getLinkTransforms(ACTIVE_JOINTS, JOINT_VALUES)
    _assert_transforms_equal(
        transforms, solver.getState(ACTIVE_JOINTS, JOINT_VALUES).link_transforms
    )


# --- floating joints --------------------------------------------------------------------------


def test_ofkt_set_state_floating_moves_child():
    """Review focus: {joint: Isometry3d} reaches the TransformMap overload (the values are not
    floats, so the joint-value map overload cannot match)."""
    solver = _solver("ofkt")
    target = _translation(1.0, -2.0, 3.0)

    solver.setState({FLOATING_JOINT: target})

    state = solver.getState()
    np.testing.assert_allclose(
        state.link_transforms[FLOATING_CHILD].matrix, target.matrix, atol=TRANSFORM_ATOL
    )


def test_ofkt_floating_values_on_every_overload():
    solver = _solver("ofkt")
    target = _translation(0.5, 0.5, 0.5)
    floating = {FLOATING_JOINT: target}

    for state in [
        solver.getState(floating),
        solver.getState(JOINT_VALUES, floating),
        solver.getState(dict(zip(ACTIVE_JOINTS, JOINT_VALUES)), floating),
        solver.getState(ACTIVE_JOINTS, JOINT_VALUES, floating),
    ]:
        np.testing.assert_allclose(
            state.link_transforms[FLOATING_CHILD].matrix, target.matrix, atol=TRANSFORM_ATOL
        )

    transforms = solver.getLinkTransforms(ACTIVE_JOINTS, JOINT_VALUES, floating)
    np.testing.assert_allclose(
        transforms[FLOATING_CHILD].matrix, target.matrix, atol=TRANSFORM_ATOL
    )

    solver.setState(ACTIVE_JOINTS, JOINT_VALUES, floating)
    np.testing.assert_allclose(
        solver.getState().link_transforms[FLOATING_CHILD].matrix, target.matrix, atol=TRANSFORM_ATOL
    )


def test_kdl_set_state_floating_is_unsupported():
    """KDLStateSolver::setState(TransformMap) throws "not supported" (kdl_state_solver.cpp:78-141)."""
    solver = _solver("kdl")
    with pytest.raises(RuntimeError, match="not supported"):
        solver.setState({FLOATING_JOINT: _translation(1.0, 0.0, 0.0)})


# --- insertSceneGraph -------------------------------------------------------------------------


def _tool_graph() -> sg.SceneGraph:
    graph = sg.SceneGraph("tool")
    assert graph.addLink(sg.Link("tool_base"))
    assert graph.addLink(sg.Link("tool_tip"))
    assert graph.addJoint(
        _joint(
            "tool_joint",
            "tool_base",
            "tool_tip",
            sg.JointType.FIXED,
            [0, 0, 1],
            _translation(0, 0, 0.1),
        )
    )
    return graph


def test_insert_scene_graph():
    solver = _solver("ofkt")
    mount = _joint(
        "p_mount", TIP_LINK, "p_tool_base", sg.JointType.FIXED, [0, 0, 1], _translation(0, 0, 0)
    )

    assert solver.insertSceneGraph(_tool_graph(), mount, "p_")

    assert {"p_tool_base", "p_tool_tip"} <= set(solver.getLinkNames())
    assert not solver.insertSceneGraph(_tool_graph(), mount, "p_")
