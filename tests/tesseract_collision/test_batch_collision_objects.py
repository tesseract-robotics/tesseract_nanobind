"""Batch collision-object updates (#373): `CollisionObjectSpec` and the `*CollisionObjects*`
methods of `DiscreteContactManager` and `ContinuousContactManager`.

Upstream defines each batch call as observably equivalent to the single calls in order
(discrete_contact_manager.h). An id the manager does not hold is skipped: `removeCollisionObjects`
and `setCollisionObjectsEnabled` report it by a `False` return; `setCollisionObjectsTransform`
returns nothing, so there the skip is silent. An id missing from a transform map raises, and so
does `getCollisionObjectsTransform` of an id the manager does not hold.
"""

import gc

import numpy as np
import pytest

from tesseract_robotics.tesseract_collision import (
    CollisionObjectSpec,
    ContactRequest,
    ContactResultMap,
    ContactTestType,
    ContinuousContactManager,
)
from tesseract_robotics.tesseract_common import CollisionMarginData, Isometry3d, LinkId

from .test_contact_manager_config import _box, _get_discrete_factory

# m: contact margin, so separated boxes within it still report a distance
MARGIN = 0.1
# m: the three boxes' x positions; unit boxes, so 0 and 0.5 overlap and 3 is far from both
X_POSITIONS = (0.0, 0.5, 3.0)
NAMES = ("box_a", "box_b", "box_c")

MANAGERS = {
    "BulletDiscreteBVH": lambda f: f.createDiscreteContactManager("BulletDiscreteBVHManager"),
    "FCLDiscreteBVH": lambda f: f.createDiscreteContactManager("FCLDiscreteBVHManager"),
    "BulletCastBVH": lambda f: f.createContinuousContactManager("BulletCastBVHManager"),
}


@pytest.fixture(params=list(MANAGERS), ids=list(MANAGERS))
def make_manager(request):
    """A factory of fresh managers of one backend; the plugin factory outlives them."""
    factory, _locator = _get_discrete_factory()
    managers = []

    def make():
        manager = MANAGERS[request.param](factory)
        managers.append(manager)
        return manager

    yield make
    # Each manager keeps its plugin factory alive (gh-72); drop them before the next test.
    managers.clear()
    gc.collect()


def _pose(x: float) -> Isometry3d:
    m = np.eye(4)
    m[0, 3] = x
    return Isometry3d(m)


def _specs() -> list[CollisionObjectSpec]:
    specs = []
    for name in NAMES:
        shapes, poses = _box()
        specs.append(CollisionObjectSpec(name, 0, shapes, poses))
    return specs


def _place(manager, names=NAMES, xs=X_POSITIONS):
    """Put each object at its x; a continuous manager gets a zero-length cast."""
    for name, x in zip(names, xs):
        if isinstance(manager, ContinuousContactManager):
            manager.setCollisionObjectsTransform(name, _pose(x), _pose(x))
        else:
            manager.setCollisionObjectsTransform(name, _pose(x))


def _contacts(manager) -> list[tuple[str, str, float]]:
    manager.setActiveCollisionObjects(list(manager.getCollisionObjects()))
    manager.setCollisionMarginData(CollisionMarginData(MARGIN))
    _place(manager)
    result = ContactResultMap()
    manager.contactTest(result, ContactRequest(ContactTestType.ALL))
    found = []
    for contact in result.flattenCopyResults():
        a, b = sorted(link.name() for link in contact.link_ids)
        found.append((a, b, contact.distance))
    return sorted(found)


def test_spec_fields():
    shapes, poses = _box()
    spec = CollisionObjectSpec("box", 3, shapes, poses, enabled=False)
    assert spec.id == LinkId("box")
    assert spec.mask_id == 3
    assert len(spec.shapes) == 1
    assert len(spec.shape_poses) == 1
    assert spec.enabled is False
    default = CollisionObjectSpec()
    assert default.mask_id == 0
    assert default.enabled is True


def test_batch_add_equals_single_adds(make_manager):
    batch = make_manager()
    assert batch.addCollisionObjects(_specs()) is True
    single = make_manager()
    for spec in _specs():
        assert single.addCollisionObject(spec.id, spec.mask_id, spec.shapes, spec.shape_poses)
    assert list(batch.getCollisionObjects()) == list(single.getCollisionObjects())
    batch_contacts, single_contacts = _contacts(batch), _contacts(single)
    assert batch_contacts, "the overlapping boxes must touch"
    assert [c[:2] for c in batch_contacts] == [c[:2] for c in single_contacts]
    np.testing.assert_allclose([c[2] for c in batch_contacts], [c[2] for c in single_contacts])


def test_batch_transform_round_trip(make_manager):
    manager = make_manager()
    assert manager.addCollisionObjects(_specs())
    manager.setCollisionObjectsEnabled([NAMES[1]], False)
    xs = (1.0, 2.0, 3.0)
    manager.setCollisionObjectsTransform(list(NAMES), {n: _pose(x) for n, x in zip(NAMES, xs)})
    for name, x in zip(NAMES, xs):
        np.testing.assert_allclose(
            manager.getCollisionObjectsTransform(name).matrix, _pose(x).matrix
        )


def test_batch_transform_from_set(make_manager):
    manager = make_manager()
    assert manager.addCollisionObjects(_specs())
    state = {n: _pose(x) for n, x in zip(NAMES, (4.0, 5.0, 6.0))}
    manager.setCollisionObjectsTransform({LinkId(NAMES[0])}, state)
    np.testing.assert_allclose(
        manager.getCollisionObjectsTransform(NAMES[0]).matrix, _pose(4.0).matrix
    )


def test_batch_disable_removes_contacts(make_manager):
    manager = make_manager()
    assert manager.addCollisionObjects(_specs())
    assert _contacts(manager)
    assert manager.setCollisionObjectsEnabled([NAMES[0]], False) is True
    assert not manager.isCollisionObjectEnabled(NAMES[0])
    assert _contacts(manager) == []
    assert manager.setCollisionObjectsEnabled({NAMES[0]: True}) is True
    assert _contacts(manager)


def test_batch_remove(make_manager):
    manager = make_manager()
    assert manager.addCollisionObjects(_specs())
    assert manager.removeCollisionObjects([NAMES[0], NAMES[2]]) is True
    assert [link.name() for link in manager.getCollisionObjects()] == [NAMES[1]]
    assert not manager.hasCollisionObject(NAMES[0])


def test_batch_unknown_name(make_manager):
    """An absent id is skipped and reported (`False`); the present ids are still applied."""
    manager = make_manager()
    assert manager.addCollisionObjects(_specs())
    assert manager.removeCollisionObjects([NAMES[0], "missing"]) is False
    assert not manager.hasCollisionObject(NAMES[0])
    assert manager.setCollisionObjectsEnabled([NAMES[1], "missing"], False) is False
    assert not manager.isCollisionObjectEnabled(NAMES[1])
    assert manager.setCollisionObjectsEnabled({"missing": True}) is False
    with pytest.raises(IndexError):
        manager.getCollisionObjectsTransform("missing")
    with pytest.raises(IndexError):
        manager.setCollisionObjectsTransform(["missing"], {NAMES[1]: _pose(0.0)})


# The two id containers of the (ids, state0, state1) cast overloads
CAST_IDS = {"list": list, "set": lambda names: {LinkId(name) for name in names}}


@pytest.fixture
def cast_manager():
    """A Bullet cast manager holding the three boxes, placed; box_a and box_b can move."""
    factory, _locator = _get_discrete_factory()
    manager = factory.createContinuousContactManager("BulletCastBVHManager")
    assert manager.addCollisionObjects(_specs())
    manager.setActiveCollisionObjects(list(NAMES[:2]))
    manager.setCollisionMarginData(CollisionMarginData(MARGIN))
    _place(manager, NAMES[:2], X_POSITIONS[:2])
    manager.setCollisionObjectsTransform(NAMES[2], _pose(X_POSITIONS[2]))
    yield manager
    del manager
    gc.collect()


def _cast_contact_pairs(manager) -> list[tuple[str, str]]:
    result = ContactResultMap()
    manager.contactTest(result, ContactRequest(ContactTestType.ALL))
    return sorted(
        tuple(sorted(link.name() for link in contact.link_ids))
        for contact in result.flattenCopyResults()
    )


@pytest.mark.parametrize("container", CAST_IDS)
def test_batch_cast_transform_from_two_maps(cast_manager, container):
    """state0 is the start pose, state1 the end; an id the batch does not name stays put."""
    far = 10.0  # m: clear of every box
    ids = CAST_IDS[container](NAMES[:2])
    state0 = {NAMES[0]: _pose(far), NAMES[1]: _pose(2 * far), NAMES[2]: _pose(3 * far)}

    # box_a ends on box_c, so only the end pose in state1 can produce this contact
    state1 = {NAMES[0]: _pose(X_POSITIONS[2]), NAMES[1]: _pose(2 * far), NAMES[2]: _pose(3 * far)}
    cast_manager.setCollisionObjectsTransform(ids, state0, state1)
    assert _cast_contact_pairs(cast_manager) == [(NAMES[0], NAMES[2])]
    for name, x in zip(NAMES, (far, 2 * far, X_POSITIONS[2])):
        np.testing.assert_allclose(
            cast_manager.getCollisionObjectsTransform(name).matrix, _pose(x).matrix
        )

    # the same start with an end that stays clear: no sweep reaches box_c
    cast_manager.setCollisionObjectsTransform(ids, state0, state0)
    assert _cast_contact_pairs(cast_manager) == []


@pytest.mark.parametrize("container", CAST_IDS)
@pytest.mark.parametrize("lacking", ["state0", "state1"])
def test_batch_cast_transform_lacking_id_raises_and_moves_nothing(cast_manager, container, lacking):
    """Upstream gathers both maps before it moves anything: IndexError, every pose unchanged."""
    ids = CAST_IDS[container](NAMES[:2])
    states = {key: {name: _pose(7.0) for name in NAMES[:2]} for key in ("state0", "state1")}
    # The second id: an implementation moving as it goes has already moved the first.
    del states[lacking][NAMES[1]]
    with pytest.raises(IndexError):
        cast_manager.setCollisionObjectsTransform(ids, states["state0"], states["state1"])
    for name, x in zip(NAMES, X_POSITIONS):
        np.testing.assert_allclose(
            cast_manager.getCollisionObjectsTransform(name).matrix, _pose(x).matrix
        )


@pytest.mark.parametrize("container", CAST_IDS)
def test_batch_cast_transform_skips_an_id_the_manager_does_not_hold(cast_manager, container):
    """Both maps name it, the manager does not hold it: no error, and the held id still moves."""
    moved = 7.0  # m: clear of every box
    absent = "missing"
    state = {NAMES[0]: _pose(moved), absent: _pose(moved)}
    ids = CAST_IDS[container]([NAMES[0], absent])
    assert cast_manager.setCollisionObjectsTransform(ids, state, state) is None
    assert not cast_manager.hasCollisionObject(absent)
    np.testing.assert_allclose(
        cast_manager.getCollisionObjectsTransform(NAMES[0]).matrix, _pose(moved).matrix
    )


def test_batch_duplicate_add_last_entry_wins(make_manager):
    """A repeated id in one batch behaves as repeated single adds: the last entry decides."""
    manager = make_manager()
    shapes, poses = _box()
    small_shapes, small_poses = _box(0.5)
    specs = [
        CollisionObjectSpec("box", 0, shapes, poses),
        CollisionObjectSpec("box", 0, small_shapes, small_poses),
    ]
    assert manager.addCollisionObjects(specs) is True
    assert [link.name() for link in manager.getCollisionObjects()] == ["box"]
    (geometry,) = manager.getCollisionObjectGeometries("box")
    assert geometry.getX() == pytest.approx(0.5)


def test_spec_rejects_none_shape(make_manager):
    manager = make_manager()
    _, poses = _box()
    with pytest.raises((TypeError, ValueError)):
        spec = CollisionObjectSpec("box", 0, [None], poses)  # ty: ignore[invalid-argument-type]
        manager.addCollisionObjects([spec])
