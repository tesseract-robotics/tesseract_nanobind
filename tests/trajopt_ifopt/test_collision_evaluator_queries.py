"""Collision evaluator queries and the trajopt_common result types (#292).

The two evaluator bases bind their query methods, so Python sees which contacts and gradients
a collision constraint works from. calcCollisions / calcCollisionData fill a C++ out-param; in
Python they return a fresh CollisionCacheData.
"""

from __future__ import annotations

import numpy as np

from tesseract_robotics import trajopt_ifopt as ti

from .scenes import OVERLAP_X, TwoBoxes, two_boxes_at

PAIR = ("base_link", "slider_link")
# max contacts per link pair the evaluator keeps
MAX_ALLOWED = 3
# m: second slider position of the continuous pair, still overlapping the fixed box
SECOND_X = 0.15


def _pair_results(data: ti.CollisionCacheData):
    container = data.contact_results_map.getContainer()
    return container.get(PAIR) or container[PAIR[::-1]]


def test_calc_collisions_reports_the_pair(two_boxes: TwoBoxes):
    data = two_boxes.discrete_evaluator().calcCollisions(np.array([OVERLAP_X]), MAX_ALLOWED)
    assert isinstance(data, ti.CollisionCacheData)
    assert len(_pair_results(data)) >= 1
    assert len(data.gradient_results_sets) >= 1
    (results_set,) = data.gradient_results_sets
    assert set(results_set.key) == set(PAIR)
    assert results_set.getMaxError() > 0.0


def test_get_gradient_is_non_zero(two_boxes: TwoBoxes):
    evaluator = two_boxes.discrete_evaluator()
    dof = np.array([OVERLAP_X])
    contact = _pair_results(evaluator.calcCollisions(dof, MAX_ALLOWED))[0]
    gradient = evaluator.getGradient(dof, contact)
    assert isinstance(gradient, ti.GradientResults)
    assert gradient.error > 0.0
    moving = [g for g in gradient.gradients if g.has_gradient]
    assert moving, "the slider link has a gradient"
    assert np.any(np.asarray(moving[0].gradient) != 0.0)


def test_get_joint_group_names_the_joints(two_boxes: TwoBoxes):
    evaluator = two_boxes.discrete_evaluator()
    assert list(evaluator.getJointGroup().getJointNames()) == ["slider_joint"]


def test_continuous_queries():
    boxes = two_boxes_at(OVERLAP_X, SECOND_X)
    evaluator = boxes.lvs_discrete_evaluator()
    dof0, dof1 = np.array([OVERLAP_X]), np.array([SECOND_X])
    data = evaluator.calcCollisionData(dof0, dof1, False, False, MAX_ALLOWED)
    assert isinstance(data, ti.CollisionCacheData)
    contact = _pair_results(data)[0]
    gradient = evaluator.calcGradientData(dof0, dof1, contact)
    assert isinstance(gradient, ti.GradientResults)
    assert list(evaluator.getJointGroup().getJointNames()) == ["slider_joint"]


def test_result_types_are_value_types():
    link = ti.LinkGradientResults()
    assert link.has_gradient is False
    link.gradient = np.array([1.0, 2.0])
    np.testing.assert_array_equal(link.gradient, [1.0, 2.0])
    link.clear()
    assert link.has_gradient is False

    results = ti.GradientResults()
    assert len(results.gradients) == 2
    assert len(results.cc_gradients) == 2
    results.error = 0.5
    assert results.error == 0.5

    max_error = ti.LinkMaxError()
    assert max_error.has_error == [False, False]
    max_error.has_error = [True, False]
    max_error.error = [0.25, -1.0]
    max_error.error_with_buffer = [0.5, -1.0]
    assert max_error.getMaxError() == 0.25
    assert max_error.getMaxErrorWithBuffer() == 0.5

    results_set = ti.GradientResultsSet()
    results_set.key = PAIR
    assert results_set.key == PAIR
    assert results_set.results == []
    assert len(results_set.max_error) == 2

    data = ti.CollisionCacheData()
    assert data.gradient_results_sets == []
    assert data.contact_results_map.count() == 0
