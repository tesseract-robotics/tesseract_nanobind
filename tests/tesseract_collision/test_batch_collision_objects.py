"""Batch collision-object updates (#373): `CollisionObjectSpec` and the `*CollisionObjects*`
methods of `DiscreteContactManager` and `ContinuousContactManager`.

Upstream defines each batch call as observably equivalent to the single calls in order
(discrete_contact_manager.h). An id the manager does not hold is skipped: `removeCollisionObjects`
and `setCollisionObjectsEnabled` report it by a `False` return; `setCollisionObjectsTransform`
returns nothing, so there the skip is silent. An id missing from a transform map raises, and so
does `getCollisionObjectsTransform` of an id the manager does not hold.

`setCollisionObjectsTransform(ids, transforms)` with a `(K, 4, 4)` array checks every transform
as `Isometry3d(matrix)` does before it moves any object.
"""

import gc

import numpy as np
import pytest

from tesseract_robotics import collision
from tesseract_robotics.collision import (
    CollisionObjectSpec,
    ContactRequest,
    ContactResultMap,
    ContactTestType,
    ContinuousContactManager,
)
from tesseract_robotics.common import CollisionMarginData, Isometry3d, LinkId, NotRotationError

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


# The (ids, transforms) overload: K transforms as one (K, 4, 4) float64 array, row-major 4 x 4 per
# transform, as compose_each and Environment.link_transforms_batch hold them.


def _rigid_stack(xs=X_POSITIONS) -> np.ndarray:
    """A (K, 4, 4) stack of rigid transforms, translated along x and turned about z.

    Each rotation is a different one, so a transposed or misindexed read moves an object
    to a pose the test can tell apart.
    """
    stack = np.zeros((len(xs), 4, 4))
    for k, x in enumerate(xs):
        angle = 0.3 * (k + 1)  # rad
        c, s = np.cos(angle), np.sin(angle)
        stack[k, :3, :3] = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
        stack[k, :3, 3] = [x, 0.1 * k, -0.2 * k]
        stack[k, 3, 3] = 1.0
    return stack


def _poses(manager, names=NAMES) -> np.ndarray:
    return np.stack([manager.getCollisionObjectsTransform(name).matrix for name in names])


def _contacts_in_place(manager) -> list[tuple[str, str, float]]:
    """Contacts at the poses the manager holds (no _place), sorted."""
    manager.setActiveCollisionObjects(list(manager.getCollisionObjects()))
    manager.setCollisionMarginData(CollisionMarginData(MARGIN))
    result = ContactResultMap()
    manager.contactTest(result, ContactRequest(ContactTestType.ALL))
    found = []
    for contact in result.flattenCopyResults():
        a, b = sorted(link.name() for link in contact.link_ids)
        found.append((a, b, contact.distance))
    return sorted(found)


def test_stack_transform_equals_list_overload(make_manager):
    """Same poses and same contacts as the (ids, list[Isometry3d]) overload, both manager kinds."""
    stack = _rigid_stack()
    from_stack = make_manager()
    assert from_stack.addCollisionObjects(_specs())
    from_list = make_manager()
    assert from_list.addCollisionObjects(_specs())

    assert from_stack.setCollisionObjectsTransform(list(NAMES), stack) is None
    from_list.setCollisionObjectsTransform(list(NAMES), [Isometry3d(m) for m in stack])

    np.testing.assert_array_equal(_poses(from_stack), _poses(from_list))
    np.testing.assert_allclose(_poses(from_stack), stack, atol=1e-12)
    stack_contacts = _contacts_in_place(from_stack)
    assert stack_contacts, "the boxes at x = 0 and 0.5 must touch"
    assert stack_contacts == _contacts_in_place(from_list)


def test_stack_transform_takes_link_ids(make_manager):
    manager = make_manager()
    assert manager.addCollisionObjects(_specs())
    stack = _rigid_stack()
    manager.setCollisionObjectsTransform([LinkId(name) for name in NAMES], stack)
    np.testing.assert_allclose(_poses(manager), stack, atol=1e-12)


def test_ndarray_dispatches_to_stack_overload_and_list_to_list_overload(make_manager):
    """The count check tells the overloads apart: the stack overload raises its own error, the
    list overload upstream's RuntimeError."""
    manager = make_manager()
    assert manager.addCollisionObjects(_specs())
    stack = _rigid_stack()
    with pytest.raises(collision.TransformCountMismatchError):
        manager.setCollisionObjectsTransform(list(NAMES[:2]), stack)
    with pytest.raises(RuntimeError, match="received 2 ids but 3 poses"):
        manager.setCollisionObjectsTransform(list(NAMES[:2]), [Isometry3d(m) for m in stack])


def test_stack_count_mismatch_is_a_value_error_naming_both_counts(make_manager):
    manager = make_manager()
    assert manager.addCollisionObjects(_specs())
    before = _poses(manager)
    assert issubclass(collision.TransformCountMismatchError, ValueError)
    with pytest.raises(collision.TransformCountMismatchError, match=r"3 ids but 2 transforms"):
        manager.setCollisionObjectsTransform(list(NAMES), _rigid_stack()[:2])
    np.testing.assert_array_equal(_poses(manager), before)


@pytest.mark.parametrize("shape", [(3, 3, 3), (3, 4, 3), (3, 16), (4, 4)])
def test_stack_wrong_shape_matches_no_overload(make_manager, shape):
    manager = make_manager()
    assert manager.addCollisionObjects(_specs())
    with pytest.raises(TypeError):
        manager.setCollisionObjectsTransform(list(NAMES), np.zeros(shape))


@pytest.mark.parametrize(
    "layout",
    ["fortran_slices", "strided", "float32"],
)
def test_stack_other_layouts_are_converted(make_manager, layout):
    """Like compose_each: a stack that is not C-contiguous float64 is converted, not refused."""
    manager = make_manager()
    assert manager.addCollisionObjects(_specs())
    stack = _rigid_stack()
    if layout == "fortran_slices":
        given = np.ascontiguousarray(stack.transpose(0, 2, 1)).transpose(0, 2, 1)
        assert not given.flags.c_contiguous
    elif layout == "strided":
        doubled = np.repeat(stack, 2, axis=0)
        given = doubled[::2]
        assert not given.flags.c_contiguous
    else:
        # float32 rounding leaves the rotation block orthonormal only to ~1e-7: use the identity
        stack[:, :3, :3] = np.eye(3)
        given = stack.astype(np.float32)
    manager.setCollisionObjectsTransform(list(NAMES), given)
    np.testing.assert_allclose(_poses(manager), stack, atol=1e-6)


def _bad(row: int, defect: str) -> np.ndarray:
    stack = _rigid_stack()
    if defect == "bottom_row":
        stack[row, 3] = [0.0, 0.0, 0.0, 2.0]
    elif defect == "not_orthonormal":
        stack[row, :3, :3] *= 1.01
    else:
        stack[row, 1, 3] = np.nan
    return stack


@pytest.mark.parametrize(
    ("defect", "match"),
    [
        ("bottom_row", "bottom row"),
        ("not_orthonormal", "not orthonormal"),
        ("non_finite", "not finite"),
    ],
)
@pytest.mark.parametrize("row", [0, 2])
def test_stack_non_rigid_row_raises_naming_it_and_moves_nothing(make_manager, defect, match, row):
    """Isometry3d(matrix)'s rule and error, per row, checked for every row before any id moves."""
    manager = make_manager()
    assert manager.addCollisionObjects(_specs())
    before = _poses(manager)
    with pytest.raises(NotRotationError, match=rf"transforms\[{row}\].*{match}"):
        manager.setCollisionObjectsTransform(list(NAMES), _bad(row, defect))
    np.testing.assert_array_equal(_poses(manager), before)


@pytest.mark.parametrize("defect", ["bottom_row", "not_orthonormal", "non_finite"])
def test_stack_rule_is_isometry3d_rule(defect):
    """Every row the stack overload refuses, Isometry3d(matrix) refuses with the same error."""
    with pytest.raises(NotRotationError):
        Isometry3d(_bad(1, defect)[1])


def test_stack_unknown_id_behaves_like_list_overload(make_manager):
    """An id the manager does not hold is skipped without error; the held ids still move."""
    stack = _rigid_stack()
    ids = [NAMES[0], "missing", NAMES[2]]
    from_stack = make_manager()
    assert from_stack.addCollisionObjects(_specs())
    from_list = make_manager()
    assert from_list.addCollisionObjects(_specs())
    from_stack.setCollisionObjectsTransform(ids, stack)
    from_list.setCollisionObjectsTransform(ids, [Isometry3d(m) for m in stack])
    assert not from_stack.hasCollisionObject("missing")
    np.testing.assert_array_equal(_poses(from_stack), _poses(from_list))
    np.testing.assert_allclose(_poses(from_stack, [NAMES[0], NAMES[2]]), stack[[0, 2]], atol=1e-12)


def test_stack_empty(make_manager):
    manager = make_manager()
    assert manager.addCollisionObjects(_specs())
    before = _poses(manager)
    assert manager.setCollisionObjectsTransform([], np.empty((0, 4, 4))) is None
    np.testing.assert_array_equal(_poses(manager), before)
    # An empty stack is still counted against the ids.
    with pytest.raises(collision.TransformCountMismatchError, match=r"1 ids but 0 transforms"):
        manager.setCollisionObjectsTransform([NAMES[0]], np.empty((0, 4, 4)))
