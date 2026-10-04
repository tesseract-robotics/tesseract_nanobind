"""checkTrajectory + ContactTrajectoryResults (gh-158).

The native out-param `std::vector<ContactResultMap>& contacts` is returned instead:
`checkTrajectory(...) -> (ContactTrajectoryResults, list[ContactResultMap])`.
"""

import numpy as np
import pytest

from tesseract_robotics.tesseract_collision import (
    CollisionCheckConfig,
    CollisionEvaluatorType,
    ContactTrajectoryResults,
    ContinuousCollisionType,
)
from tesseract_robotics.tesseract_environment import checkTrajectory

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
