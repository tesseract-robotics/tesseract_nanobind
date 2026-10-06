"""ContactResult.cc_time / cc_type: the continuous-collision fields (gh-158)."""

import gc

import numpy as np
import pytest

from tesseract_robotics.tesseract_collision import (
    ContactRequest,
    ContactResult,
    ContactResultMap,
    ContactResultVector,
    ContactTestType_ALL,
    ContinuousCollisionType,
)
from tesseract_robotics.tesseract_common import CollisionMarginData, Isometry3d

from .test_contact_manager_config import _box, _get_discrete_factory


def test_cc_time_default():
    assert list(ContactResult().cc_time) == [-1.0, -1.0]


def test_cc_type_default():
    none = ContinuousCollisionType.CCType_None
    assert list(ContactResult().cc_type) == [none, none]


def test_cc_time_cc_type_round_trip():
    cr = ContactResult()
    cr.cc_time = [0.25, 0.75]
    cr.cc_type = [ContinuousCollisionType.CCType_Between, ContinuousCollisionType.CCType_Time1]
    assert list(cr.cc_time) == [0.25, 0.75]
    assert list(cr.cc_type) == [
        ContinuousCollisionType.CCType_Between,
        ContinuousCollisionType.CCType_Time1,
    ]


@pytest.mark.parametrize("bad", [[0.5], [0.1, 0.2, 0.3]])
def test_cc_time_rejects_wrong_length(bad):
    cr = ContactResult()
    with pytest.raises(TypeError):
        cr.cc_time = bad


def test_continuous_sweep_populates_cc_time():
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

        # sweep through the static box and out the other side: contact strictly inside the motion
        start = np.eye(4)
        start[0][3] = -5.0
        end = np.eye(4)
        end[0][3] = 5.0
        checker.setCollisionObjectsTransformCast("moving_box", Isometry3d(start), Isometry3d(end))

        result = ContactResultMap()
        checker.contactTest(result, ContactRequest(ContactTestType_ALL))
        flat = ContactResultVector()
        result.flattenMoveResults(flat)
        assert len(flat) > 0

        none = ContinuousCollisionType.CCType_None
        for cr in flat:
            cast = [i for i in range(2) if cr.cc_type[i] != none]
            assert cast, "continuous contact must mark the cast link's cc_type"
            for i in cast:
                assert 0.0 <= cr.cc_time[i] <= 1.0
    finally:
        del checker
        del factory
        del locator
        gc.collect()


# --------------------------------------------------------------------------- #
# gh-174: the cast overloads under the native setCollisionObjectsTransform name
# --------------------------------------------------------------------------- #
def _sweep_x(x):
    m = np.eye(4)
    m[0][3] = x
    return Isometry3d(m)


def _cast_contact(set_cast):
    """Sweep moving_box through static_box with `set_cast(checker, start, end)`; return the contact."""
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
        set_cast(checker, _sweep_x(-5.0), _sweep_x(5.0))

        result = ContactResultMap()
        checker.contactTest(result, ContactRequest(ContactTestType_ALL))
        flat = ContactResultVector()
        result.flattenMoveResults(flat)
        assert len(flat) == 1
        cr = flat[0]
        return list(cr.cc_time), list(cr.cc_type), list(cr.link_names)
    finally:
        del checker
        del factory
        del locator
        gc.collect()


CAST_SHAPES = {
    "name": (
        lambda c, s, e: c.setCollisionObjectsTransform("moving_box", s, e),
        lambda c, s, e: c.setCollisionObjectsTransformCast("moving_box", s, e),
    ),
    "names": (
        lambda c, s, e: c.setCollisionObjectsTransform(["moving_box"], [s], [e]),
        lambda c, s, e: c.setCollisionObjectsTransformCast(["moving_box"], [s], [e]),
    ),
    # header :199, (TransformMap, TransformMap): arity 2, invisible to the arity-only audit
    "transform_maps": (
        lambda c, s, e: c.setCollisionObjectsTransform({"moving_box": s}, {"moving_box": e}),
        lambda c, s, e: c.setCollisionObjectsTransformCast({"moving_box": s}, {"moving_box": e}),
    ),
}


@pytest.mark.parametrize("shape", CAST_SHAPES)
def test_native_cast_overload_matches_cast_alias(shape):
    native, alias = CAST_SHAPES[shape]
    expected = _cast_contact(alias)
    assert _cast_contact(native) == expected
    assert expected[1] != [ContinuousCollisionType.CCType_None] * 2


def _static_names_contacts_at(checker, x):
    checker.setCollisionObjectsTransform(["moving_box"], [_sweep_x(x)])
    result = ContactResultMap()
    checker.contactTest(result, ContactRequest(ContactTestType_ALL))
    return result.count()


def test_arity_two_static_names_overload_still_sets_static_pose():
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

        # unit boxes, margin 0.1: apart at x = 3, overlapping at x = 0.5
        assert _static_names_contacts_at(checker, 3.0) == 0
        assert _static_names_contacts_at(checker, 0.5) == 1
    finally:
        del checker
        del factory
        del locator
        gc.collect()
