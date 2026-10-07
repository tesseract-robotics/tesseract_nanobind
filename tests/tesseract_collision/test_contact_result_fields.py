"""ContactResult pair fields: every std::array<T, 2> member is bound with def_rw (gh-172).

nanobind's array caster accepts exactly two items, so a short or long list raises TypeError
instead of being ignored or truncated.
"""

import gc

import numpy as np
import pytest

from tesseract_robotics.tesseract_collision import (
    ContactRequest,
    ContactResult,
    ContactResultMap,
    ContactResultVector,
    ContactTestType,
    ContinuousCollisionType,
)
from tesseract_robotics.tesseract_common import CollisionMarginData, Isometry3d

from .test_contact_manager_config import _box, _get_discrete_factory


def _translation(x, y, z):
    m = np.eye(4)
    m[:3, 3] = [x, y, z]
    return m


def _plain(value):
    """Normalise a pair-field item for comparison: Isometry3d → 4x4, vectors → list."""
    if isinstance(value, Isometry3d):
        return np.asarray(value.matrix).tolist()
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value


# (field, valid 2-item value): one entry per std::array<T, 2> member of ContactResult
# (tesseract/collision/types.h:88-118), cc_time and cc_type are covered in
# test_contact_result_continuous.py.
PAIR_FIELDS = [
    ("type_id", lambda: [3, 4]),
    ("link_names", lambda: ["link_a", "link_b"]),
    ("shape_id", lambda: [1, 2]),
    ("subshape_id", lambda: [5, 6]),
    ("nearest_points", lambda: [np.array([1.0, 2.0, 3.0]), np.array([4.0, 5.0, 6.0])]),
    ("nearest_points_local", lambda: [np.array([1.0, 2.0, 3.0]), np.array([4.0, 5.0, 6.0])]),
    ("transform", lambda: [Isometry3d(_translation(1, 0, 0)), Isometry3d(_translation(0, 2, 0))]),
    (
        "cc_transform",
        lambda: [Isometry3d(_translation(1, 0, 0)), Isometry3d(_translation(0, 2, 0))],
    ),
]
PAIR_FIELD_IDS = [name for name, _ in PAIR_FIELDS]


@pytest.mark.parametrize(("field", "make"), PAIR_FIELDS, ids=PAIR_FIELD_IDS)
def test_pair_field_round_trips(field, make):
    cr = ContactResult()
    value = make()
    setattr(cr, field, value)
    assert [_plain(v) for v in getattr(cr, field)] == [_plain(v) for v in value]


@pytest.mark.parametrize(("field", "make"), PAIR_FIELDS, ids=PAIR_FIELD_IDS)
def test_pair_field_short_list_raises_and_keeps_old_value(field, make):
    cr = ContactResult()
    value = make()
    setattr(cr, field, value)
    with pytest.raises(TypeError):
        setattr(cr, field, value[:1])
    assert [_plain(v) for v in getattr(cr, field)] == [_plain(v) for v in value]


@pytest.mark.parametrize(("field", "make"), PAIR_FIELDS, ids=PAIR_FIELD_IDS)
def test_pair_field_long_list_raises(field, make):
    cr = ContactResult()
    value = make()
    with pytest.raises(TypeError):
        setattr(cr, field, [*value, value[0]])


def test_new_pair_field_defaults_match_header():
    cr = ContactResult()
    assert list(cr.subshape_id) == [-1, -1]
    assert [_plain(p) for p in cr.nearest_points_local] == [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    assert [_plain(t) for t in cr.cc_transform] == [np.eye(4).tolist(), np.eye(4).tolist()]


def test_continuous_contact_cc_transform_is_cast_end_pose():
    """types.h:113-117: cc_transform holds the cast object's end transform."""
    factory, locator = _get_discrete_factory()
    checker = factory.createContinuousContactManager("BulletCastBVHManager")
    try:
        shapes_a, poses_a = _box()
        shapes_b, poses_b = _box()
        checker.addCollisionObject("static_box", 0, shapes_a, poses_a)
        checker.addCollisionObject("moving_box", 0, shapes_b, poses_b)
        checker.setActiveCollisionObjects(["moving_box"])
        checker.setCollisionMarginData(CollisionMarginData(0.1))
        checker.setCollisionObjectsTransform("static_box", Isometry3d(np.eye(4)))

        start = _translation(-5.0, 0.0, 0.0)
        end = _translation(5.0, 0.0, 0.0)
        checker.setCollisionObjectsTransformCast("moving_box", Isometry3d(start), Isometry3d(end))

        result = ContactResultMap()
        checker.contactTest(result, ContactRequest(ContactTestType.ALL))
        flat = ContactResultVector()
        result.flattenMoveResults(flat)
        assert len(flat) > 0

        none = ContinuousCollisionType.CCType_None
        for cr in flat:
            moving = list(cr.link_names).index("moving_box")
            assert cr.cc_type[moving] != none
            np.testing.assert_array_equal(np.asarray(cr.cc_transform[moving].matrix), end)
    finally:
        del checker
        del factory
        del locator
        gc.collect()
