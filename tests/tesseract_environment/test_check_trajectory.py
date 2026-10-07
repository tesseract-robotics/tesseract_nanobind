"""checkTrajectory + ContactTrajectoryResults (gh-158).

The native out-param `std::vector<ContactResultMap>& contacts` is returned instead:
`checkTrajectory(...) -> (ContactTrajectoryResults, list[ContactResultMap])`.
"""

import numpy as np
import pytest

from tesseract_robotics.tesseract_collision import (
    CollisionCheckConfig,
    CollisionEvaluatorType,
    ContactRequest,
    ContactResultMap,
    ContactTrajectoryResults,
    ContinuousCollisionType,
)
from tesseract_robotics.tesseract_environment import (
    checkTrajectory,
    checkTrajectorySegment,
    checkTrajectoryState,
)

from .test_tesseract_environment import _fresh_env

GROUP = "manipulator"
NUM_STEPS = 10
# lbr_iiwa_14_r820 self-collides here (shoulder folded back onto the elbow)
COLLIDING_STATE = np.array([0.0, -2.0, 0.0, 2.0, 0.0, 0.0, 0.0])
# small wrist motion from home: no contact
FREE_STATE = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0])


def _traj(end_state):
    return np.linspace(np.zeros(7), end_state, NUM_STEPS)


def _config(evaluator):
    config = CollisionCheckConfig()
    config.type = evaluator
    return config


DISCRETE = CollisionEvaluatorType.DISCRETE
CONTINUOUS = CollisionEvaluatorType.CONTINUOUS


def _manager(env, evaluator):
    if evaluator == DISCRETE:
        return env.getDiscreteContactManager()
    return env.getContinuousContactManager()


def _call(env, evaluator, overload, traj):
    manager = _manager(env, evaluator)
    config = _config(evaluator)
    if overload == "state_solver":
        names = env.getGroupJointNames(GROUP)
        return checkTrajectory(manager, env.getStateSolver(), names, traj, config)
    return checkTrajectory(manager, env.getJointGroup(GROUP), traj, config)


EVALUATORS = pytest.mark.parametrize(
    "evaluator", [DISCRETE, CONTINUOUS], ids=["discrete", "continuous"]
)
OVERLOADS = pytest.mark.parametrize("overload", ["state_solver", "joint_group"])


@EVALUATORS
@OVERLOADS
def test_colliding_trajectory_reports_contacts(evaluator, overload):
    env = _fresh_env()
    results, contacts = _call(env, evaluator, overload, _traj(COLLIDING_STATE))
    assert isinstance(results, ContactTrajectoryResults)
    assert results
    assert results.numContacts() > 0
    assert list(results.joint_names) == list(env.getGroupJointNames(GROUP))
    assert any(c.count() > 0 for c in contacts)


@EVALUATORS
@OVERLOADS
def test_free_trajectory_reports_nothing(evaluator, overload):
    env = _fresh_env()
    results, contacts = _call(env, evaluator, overload, _traj(FREE_STATE))
    assert not results
    assert results.numContacts() == 0
    assert all(c.count() == 0 for c in contacts)


def test_continuous_contacts_carry_cc_time():
    env = _fresh_env()
    _, contacts = _call(env, CONTINUOUS, "state_solver", _traj(COLLIDING_STATE))
    none = ContinuousCollisionType.CCType_None
    flat = [cr for c in contacts for cr in c.flattenCopyResults()]
    assert len(flat) > 0
    for cr in flat:
        cast = [i for i in range(2) if cr.cc_type[i] != none]
        assert cast
        for i in cast:
            assert 0.0 <= cr.cc_time[i] <= 1.0


def test_results_structure_and_summaries():
    env = _fresh_env()
    results, _ = _call(env, DISCRETE, "state_solver", _traj(COLLIDING_STATE))
    assert results.numSteps() == len(results.steps)
    worst = results.worstStep()
    assert worst.numContacts() > 0
    assert worst.step >= 0
    assert worst.numSubsteps() == len(worst.substeps)
    sub = worst.worstSubstep()
    assert sub.numContacts() > 0
    assert sub.contacts.count() > 0
    assert len(sub.worstCollision()) > 0
    for text in (
        results.trajectoryCollisionResultsTable(),
        results.collisionFrequencyPerLink(),
        results.condensedSummary(),
    ):
        assert isinstance(text, str)
        assert text


# ---------- checkTrajectoryState / checkTrajectorySegment (gh-190) ----------


def _link_transforms(env, joint_values):
    names = env.getGroupJointNames(GROUP)
    return env.getState(names, joint_values).link_transforms


@EVALUATORS
def test_check_trajectory_state_colliding_reports_contacts(evaluator):
    env = _fresh_env()
    manager = _manager(env, evaluator)
    contacts = checkTrajectoryState(
        manager, _link_transforms(env, COLLIDING_STATE), ContactRequest()
    )
    assert isinstance(contacts, ContactResultMap)
    assert contacts.count() > 0


@EVALUATORS
def test_check_trajectory_state_free_reports_nothing(evaluator):
    env = _fresh_env()
    manager = _manager(env, evaluator)
    contacts = checkTrajectoryState(manager, _link_transforms(env, FREE_STATE), ContactRequest())
    assert isinstance(contacts, ContactResultMap)
    assert contacts.count() == 0


def test_check_trajectory_segment_free_to_colliding_reports_contacts():
    env = _fresh_env()
    manager = env.getContinuousContactManager()
    free = _link_transforms(env, FREE_STATE)
    colliding = _link_transforms(env, COLLIDING_STATE)
    contacts = checkTrajectorySegment(manager, free, colliding, ContactRequest())
    assert isinstance(contacts, ContactResultMap)
    assert contacts.count() > 0


def test_check_trajectory_segment_free_to_free_reports_nothing():
    env = _fresh_env()
    manager = env.getContinuousContactManager()
    free = _link_transforms(env, FREE_STATE)
    contacts = checkTrajectorySegment(manager, free, free, ContactRequest())
    assert contacts.count() == 0


@EVALUATORS
def test_check_trajectory_state_returns_a_fresh_map_per_call(evaluator):
    # C++ appends to the caller's map ("It does not get cleared"); the binding hands out a new
    # one each call, so a free check after a colliding one is empty and the maps are independent.
    env = _fresh_env()
    manager = _manager(env, evaluator)
    colliding = _link_transforms(env, COLLIDING_STATE)
    first = checkTrajectoryState(manager, colliding, ContactRequest())
    second = checkTrajectoryState(manager, colliding, ContactRequest())
    free = checkTrajectoryState(manager, _link_transforms(env, FREE_STATE), ContactRequest())
    assert first is not second
    assert first.count() == second.count() > 0
    assert free.count() == 0
    first.clear()
    assert first.count() == 0
    assert second.count() > 0


@EVALUATORS
def test_check_trajectory_state_missing_active_link_raises_key_error(evaluator):
    # Upstream does state.at(link) per active object (std::out_of_range -> IndexError, after
    # moving the links before it). The binding raises KeyError naming every missing link and
    # leaves the manager as it was: here link_1..link_6 at the colliding pose would touch base_link.
    env = _fresh_env()
    manager = _manager(env, evaluator)
    free = _link_transforms(env, FREE_STATE)
    assert checkTrajectoryState(manager, free, ContactRequest()).count() == 0
    partial = _link_transforms(env, COLLIDING_STATE)
    del partial["link_7"]
    del partial["tool0"]
    with pytest.raises(KeyError, match=r"checkTrajectoryState: state .*link_7, tool0"):
        checkTrajectoryState(manager, partial, ContactRequest())
    untouched = ContactResultMap()
    manager.contactTest(untouched, ContactRequest())
    assert untouched.count() == 0


@pytest.mark.parametrize("missing_arg", ["state0", "state1"])
def test_check_trajectory_segment_missing_active_link_raises_key_error(missing_arg):
    env = _fresh_env()
    manager = env.getContinuousContactManager()
    states = {
        "state0": _link_transforms(env, FREE_STATE),
        "state1": _link_transforms(env, COLLIDING_STATE),
    }
    del states[missing_arg]["link_7"]
    with pytest.raises(KeyError, match=rf"checkTrajectorySegment: {missing_arg} .*link_7"):
        checkTrajectorySegment(manager, states["state0"], states["state1"], ContactRequest())
