"""The (joint_ids, joint_values) overloads are tried first among their overloads.

nanobind tries a name's overloads in registration order. A JointId list tried against a
{joint: value} map overload costs an AttributeError (the dict caster calls `.items()` on it); tried
against a joint-vector overload, it costs failed `__dlpack__` lookups. At a few microseconds per
state call that is several percent of the call, paid on every call that passes ids and values.

A list subclass that records its attribute lookups shows which overloads were tried before the
right one: the (ids, values) overload reads the list by the sequence protocol and looks up no
attribute at all.
"""

from __future__ import annotations

import numpy as np
import pytest

from tesseract_robotics.common import LinkId
from tesseract_robotics.state_solver import KDLStateSolver, OFKTStateSolver

from ..tesseract_environment.test_tesseract_environment import _IIWA_JOINTS, _fresh_env

TIP_LINK = "tool0"


class _LookupRecordingList(list):
    """A list that records every attribute looked up on it."""

    def __init__(self, items) -> None:
        super().__init__(items)
        object.__setattr__(self, "lookups", [])

    def __getattribute__(self, name: str):
        if name != "lookups":
            object.__getattribute__(self, "lookups").append(name)
        return object.__getattribute__(self, name)


def _env():
    env = _fresh_env()
    return env, env.getStateSolver()


def _ids(solver) -> list:
    return list(solver.getActiveJointIds())


CALLS = {
    "Environment.getState": lambda env, solver, ids, q: env.getState(ids, q),
    "Environment.getState+floating": lambda env, solver, ids, q: env.getState(ids, q, {}),
    "Environment.setState": lambda env, solver, ids, q: env.setState(ids, q),
    "Environment.getLinkTransforms": lambda env, solver, ids, q: env.getLinkTransforms(ids, q),
    "StateSolver.getState": lambda env, solver, ids, q: solver.getState(ids, q),
    "StateSolver.getState+floating": lambda env, solver, ids, q: solver.getState(ids, q, {}),
    "StateSolver.setState": lambda env, solver, ids, q: solver.setState(ids, q),
    "StateSolver.getLinkTransforms": lambda env, solver, ids, q: solver.getLinkTransforms(ids, q),
    "StateSolver.getJacobian": lambda env, solver, ids, q: solver.getJacobian(
        ids, q, LinkId(TIP_LINK)
    ),
}


@pytest.mark.parametrize("call", sorted(CALLS))
def test_ids_values_call_probes_no_other_overload(call):
    env, solver = _env()
    ids = _LookupRecordingList(_ids(solver))
    CALLS[call](env, solver, ids, np.full(len(ids), 0.1))
    assert ids.lookups == []


@pytest.mark.parametrize("solver_type", [KDLStateSolver, OFKTStateSolver])
def test_solver_types_dispatch_the_same(solver_type):
    """The concrete solvers inherit the base overload order."""
    solver = solver_type(_fresh_env().getSceneGraph())
    ids = _LookupRecordingList(_ids(solver))
    solver.getState(ids, np.full(len(ids), 0.1))
    assert ids.lookups == []


def test_every_form_still_reaches_its_overload():
    """Moving the (ids, values) overload first changes no call's result."""
    env, solver = _env()
    ids = _ids(solver)
    q = np.linspace(-0.5, 0.5, len(ids))
    joints = dict(zip(ids, q))
    expected = solver.getState(q).link_transforms[TIP_LINK].matrix
    for state in (
        solver.getState(ids, q),
        solver.getState(_IIWA_JOINTS, q),
        solver.getState(ids, list(q)),
        solver.getState(joints),
        solver.getState(q, {}),
        solver.getState(joints, {}),
        env.getState(ids, q),
        env.getState(_IIWA_JOINTS, list(q)),
        env.getState(joints),
        env.getState(joints, {}),
    ):
        np.testing.assert_array_equal(state.link_transforms[TIP_LINK].matrix, expected)
    np.testing.assert_array_equal(
        solver.getJacobian(ids, q, LinkId(TIP_LINK)), solver.getJacobian(q, LinkId(TIP_LINK))
    )
    for setter in (env.setState, solver.setState):
        setter(ids, q)
        setter(joints)
    np.testing.assert_array_equal(env.getCurrentJointValues(), q)
    np.testing.assert_array_equal(solver.getState().link_transforms[TIP_LINK].matrix, expected)
