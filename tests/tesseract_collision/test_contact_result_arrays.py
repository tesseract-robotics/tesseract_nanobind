"""ContactResultMap.to_arrays / ContactResultVector.to_arrays: every result in one set of arrays.

Row r of every array describes element r of `flattenCopyResults()` (the vector's own element r),
and every array equals what a per-field Python loop over those elements builds, bit for bit.
The slot tests pin what the docs state about `[k]` fields and `normal`.
"""

import gc

import numpy as np
import pytest

from tesseract_robotics.collision import (
    ContactArrays,
    ContactRequest,
    ContactResult,
    ContactResultMap,
    ContactResultVector,
    ContactTestType,
    ContinuousCollisionType,
)
from tesseract_robotics.common import CollisionMarginData, Isometry3d, LinkId
from tesseract_robotics.geometry import Box, Capsule, Sphere

from .test_contact_manager_config import _get_discrete_factory

# [m] Contact threshold of the discrete scene: larger than every gap in it, so each pair reports.
SCENE_MARGIN_M = 0.3
# [m] Contact threshold of the slot tests: larger than the 0.2 m gap between the two bodies.
SLOT_MARGIN_M = 0.5
# [m] Radius of the slot tests' sphere and half-extent of their box.
SLOT_SPHERE_RADIUS_M = 0.1
SLOT_BOX_HALF_M = 0.1
# [m] Sphere at the origin, box centre on +x: separated by 0.5 - 0.1 - 0.1 = 0.3 m, or
# penetrating by 0.1 + 0.1 - 0.15 = 0.05 m.
SLOT_BOX_X_SEPARATED_M = 0.5
SLOT_BOX_X_PENETRATING_M = 0.15
# [m] Absolute tolerance on the slot tests' analytic distances, witness points and normals. The
# tests pin which body each slot describes, not narrowphase accuracy: FCL's sphere-box penetration
# witness is off the analytic point by up to 1.4e-4 m (observed), and the two candidate points
# (the sphere's and the box's facing surfaces) are at least 0.05 m apart, so 1e-3 m still tells
# them apart with a factor of 50 to spare.
WITNESS_TOL_M = 1e-3

DISCRETE_MANAGERS = [
    "BulletDiscreteBVHManager",
    "BulletDiscreteSimpleManager",
    "FCLDiscreteBVHManager",
]
CAST_MANAGERS = ["BulletCastBVHManager", "BulletCastSimpleManager"]

FLOAT_FIELDS = {
    "distance": (),
    "normal": (3,),
    "nearest_points": (2, 3),
    "nearest_points_local": (2, 3),
    "transform": (2, 4, 4),
    "cc_time": (2,),
    "cc_transform": (2, 4, 4),
}
INT32_FIELDS = ("type_id", "shape_id", "subshape_id")
ARRAY_DTYPES = {
    **dict.fromkeys(FLOAT_FIELDS, np.float64),
    **dict.fromkeys(INT32_FIELDS, np.int32),
    "single_contact_point": np.bool_,
    "cc_type": np.uint8,
    "link_index": np.int32,
}
ARRAY_SHAPES = {
    **FLOAT_FIELDS,
    **dict.fromkeys(INT32_FIELDS, (2,)),
    "single_contact_point": (),
    "cc_type": (2,),
    "link_index": (2,),
}


# --------------------------------------------------------------------------- #
# Reference: the per-field Python loop
# --------------------------------------------------------------------------- #
def _loop_arrays(results):
    """The arrays `to_arrays()` must return, read one field of one element at a time."""
    n = len(results)
    out = {
        name: np.empty((n, *ARRAY_SHAPES[name]), dtype=dtype)
        for name, dtype in ARRAY_DTYPES.items()
    }
    links = []
    for r in range(n):
        c = results[r]
        out["distance"][r] = c.distance
        out["normal"][r] = c.normal
        out["single_contact_point"][r] = c.single_contact_point
        for k in range(2):
            out["nearest_points"][r, k] = c.nearest_points[k]
            out["nearest_points_local"][r, k] = c.nearest_points_local[k]
            out["transform"][r, k] = c.transform[k].matrix
            out["cc_transform"][r, k] = c.cc_transform[k].matrix
            out["cc_time"][r, k] = c.cc_time[k]
            out["cc_type"][r, k] = c.cc_type[k].value
            out["type_id"][r, k] = c.type_id[k]
            out["shape_id"][r, k] = c.shape_id[k]
            out["subshape_id"][r, k] = c.subshape_id[k]
            link = c.link_ids[k]
            if link not in links:
                links.append(link)
            out["link_index"][r, k] = links.index(link)
        del c  # an element of a ContactResultVector is a reference into it
    return out, tuple(links)


def _assert_bitwise_equal(arrays, results):
    expected, expected_links = _loop_arrays(results)
    for name, want in expected.items():
        got = getattr(arrays, name)
        assert isinstance(got, np.ndarray), name
        assert got.dtype == want.dtype, name
        assert got.shape == want.shape, name
        assert got.flags.c_contiguous, name
        assert got.tobytes() == want.tobytes(), name
    assert arrays.links == expected_links
    assert all(type(link) is LinkId for link in arrays.links)


# --------------------------------------------------------------------------- #
# Scenes
# --------------------------------------------------------------------------- #
def _pose(x, y, z, rotation=None):
    m = np.eye(4)
    if rotation is not None:
        m[:3, :3] = rotation
    m[:3, 3] = [x, y, z]
    return Isometry3d(m)


def _rot_z(angle_rad):
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


@pytest.fixture
def factory():
    plugin_factory, locator = _get_discrete_factory()
    yield plugin_factory
    del plugin_factory, locator
    gc.collect()


def _discrete_scene(factory, manager_name):
    """Four bodies within the margin of each other; body_a has two shapes, every body a type id."""
    manager = factory.createDiscreteContactManager(manager_name)
    assert manager.addCollisionObject(
        "body_a", 1, [Box(0.2, 0.2, 0.2), Sphere(0.05)], [_pose(0, 0, 0), _pose(0, 0, 0.2)]
    )
    assert manager.addCollisionObject("body_b", 2, [Sphere(0.1)], [_pose(0, 0, 0)])
    assert manager.addCollisionObject("body_c", 3, [Capsule(0.05, 0.2)], [_pose(0, 0, 0)])
    assert manager.addCollisionObject("body_d", 4, [Box(0.1, 0.1, 0.1)], [_pose(0, 0, 0)])
    manager.setCollisionObjectsTransform(
        {
            "body_a": _pose(0.0, 0.0, 0.0, _rot_z(0.3)),
            "body_b": _pose(0.25, 0.0, 0.0),
            "body_c": _pose(0.0, 0.25, 0.1),
            "body_d": _pose(0.15, 0.15, 0.2, _rot_z(0.7)),
        }
    )
    manager.setActiveCollisionObjects(["body_a", "body_b", "body_c", "body_d"])
    manager.setCollisionMarginData(CollisionMarginData(SCENE_MARGIN_M))
    return manager


def _cast_scene(factory, manager_name):
    """A static box; one sphere cast through it and two capsules cast through each other."""
    manager = factory.createContinuousContactManager(manager_name)
    assert manager.addCollisionObject("wall", 5, [Box(0.1, 1.0, 1.0)], [_pose(0, 0, 0)])
    assert manager.addCollisionObject("ball", 6, [Sphere(0.05)], [_pose(0, 0, 0)])
    assert manager.addCollisionObject("rod_a", 7, [Capsule(0.03, 0.3)], [_pose(0, 0, 0)])
    assert manager.addCollisionObject("rod_b", 8, [Capsule(0.03, 0.3)], [_pose(0, 0, 0)])
    manager.setActiveCollisionObjects(["ball", "rod_a", "rod_b"])
    manager.setCollisionMarginData(CollisionMarginData(0.05))
    manager.setCollisionObjectsTransform("wall", _pose(0, 0, 0))
    manager.setCollisionObjectsTransform("ball", _pose(-1.0, 0.1, 0.0), _pose(1.0, 0.1, 0.0))
    manager.setCollisionObjectsTransform("rod_a", _pose(3.0, -1.0, 0.0), _pose(3.0, 1.0, 0.0))
    manager.setCollisionObjectsTransform("rod_b", _pose(2.0, 0.0, 0.0), _pose(4.0, 0.0, 0.0))
    return manager


def _contact_test(manager, test_type):
    results = ContactResultMap()
    manager.contactTest(results, ContactRequest(test_type))
    return results


# --------------------------------------------------------------------------- #
# Equality with the per-field loop
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize(
    "test_type", [ContactTestType.CLOSEST, ContactTestType.ALL], ids=["CLOSEST", "ALL"]
)
@pytest.mark.parametrize("manager_name", DISCRETE_MANAGERS)
def test_discrete_map_arrays_equal_field_loop(factory, manager_name, test_type):
    results = _contact_test(_discrete_scene(factory, manager_name), test_type)
    flat = results.flattenCopyResults()
    assert len(flat) >= 4  # every body is within the margin of another
    arrays = results.to_arrays()
    assert isinstance(arrays, ContactArrays)
    _assert_bitwise_equal(arrays, flat)


def test_all_reports_at_least_closest(factory):
    manager = _discrete_scene(factory, "BulletDiscreteBVHManager")
    closest = _contact_test(manager, ContactTestType.CLOSEST).to_arrays()
    every = _contact_test(manager, ContactTestType.ALL).to_arrays()
    assert len(every.distance) >= len(closest.distance)


@pytest.mark.parametrize("manager_name", CAST_MANAGERS)
def test_cast_map_arrays_equal_field_loop(factory, manager_name):
    results = _contact_test(_cast_scene(factory, manager_name), ContactTestType.ALL)
    flat = results.flattenCopyResults()
    arrays = results.to_arrays()
    _assert_bitwise_equal(arrays, flat)
    pairs = {
        frozenset(link.name() for link in (arrays.links[i], arrays.links[j]))
        for i, j in arrays.link_index
    }
    assert pairs == {frozenset({"wall", "ball"}), frozenset({"rod_a", "rod_b"})}
    # The cc_* fields carry data: some slot is not CCType_None and has a time in [0, 1].
    cast = arrays.cc_type != ContinuousCollisionType.CCType_None.value
    assert cast.any()
    assert np.all((arrays.cc_time[cast] >= 0.0) & (arrays.cc_time[cast] <= 1.0))


@pytest.mark.parametrize("manager_name", ["BulletDiscreteBVHManager", "FCLDiscreteBVHManager"])
def test_vector_arrays_equal_map_arrays(factory, manager_name):
    results = _contact_test(_discrete_scene(factory, manager_name), ContactTestType.ALL)
    from_map = results.to_arrays()
    from_vector = results.flattenCopyResults().to_arrays()
    _assert_bitwise_equal(from_vector, results.flattenCopyResults())
    for name in ARRAY_DTYPES:
        assert getattr(from_vector, name).tobytes() == getattr(from_map, name).tobytes(), name
    assert from_vector.links == from_map.links


def test_link_index_resolves_to_link_ids(factory):
    results = _contact_test(
        _discrete_scene(factory, "BulletDiscreteBVHManager"), ContactTestType.ALL
    )
    flat = results.flattenCopyResults()
    arrays = results.to_arrays()
    for r in range(len(flat)):
        for k in range(2):
            assert arrays.links[arrays.link_index[r, k]] == flat[r].link_ids[k]
    assert len(set(arrays.links)) == len(arrays.links) == 4


def test_links_in_first_appearance_order():
    v = ContactResultVector()
    for a, b in [("c", "a"), ("a", "b"), ("d", "c")]:
        r = ContactResult()
        r.link_ids = [a, b]
        v.append(r)
    arrays = v.to_arrays()
    assert [link.name() for link in arrays.links] == ["c", "a", "b", "d"]
    assert arrays.link_index.tolist() == [[0, 1], [1, 2], [3, 0]]


def test_field_values_round_trip_from_hand_built_vector():
    """Every field reaches its column, including values no contact manager produces."""
    r = ContactResult()
    r.distance = -0.25
    r.normal = [0.0, 0.0, -1.0]
    r.link_ids = ["x", "y"]
    r.type_id = [-7, 2**31 - 1]
    r.shape_id = [3, -(2**31)]
    r.subshape_id = [11, 12]
    r.nearest_points = [np.array([1.0, 2.0, 3.0]), np.array([4.0, 5.0, 6.0])]
    r.nearest_points_local = [np.array([-1.0, -2.0, -3.0]), np.array([-4.0, -5.0, -6.0])]
    r.transform = [_pose(1, 2, 3, _rot_z(0.5)), _pose(-1, 0, 0)]
    r.cc_transform = [_pose(0, 0, 1), _pose(0, 1, 0, _rot_z(-1.0))]
    r.cc_time = [0.25, 1.0]
    r.cc_type = [ContinuousCollisionType.CCType_Between, ContinuousCollisionType.CCType_Time1]
    r.single_contact_point = True
    v = ContactResultVector()
    v.append(r)
    arrays = v.to_arrays()
    _assert_bitwise_equal(arrays, v)
    assert arrays.type_id.tolist() == [[-7, 2**31 - 1]]
    assert arrays.shape_id.tolist() == [[3, -(2**31)]]
    assert arrays.cc_type.tolist() == [[3, 2]]
    assert arrays.single_contact_point.tolist() == [True]


# --------------------------------------------------------------------------- #
# Shapes, dtypes, the empty case
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize(
    "make_empty",
    [
        ContactResultMap,
        ContactResultVector,
        # clear() keeps the keys with empty vectors: they contribute no rows.
        lambda: _cleared_map(),
    ],
    ids=["new_map", "new_vector", "cleared_map"],
)
def test_empty_gives_zero_rows(make_empty):
    arrays = make_empty().to_arrays()
    for name, dtype in ARRAY_DTYPES.items():
        a = getattr(arrays, name)
        assert a.dtype == dtype, name
        assert a.shape == (0, *ARRAY_SHAPES[name]), name
    assert arrays.links == ()


def _cleared_map():
    m = ContactResultMap()
    r = ContactResult()
    r.link_ids = ["a", "b"]
    m.addContactResult(("a", "b"), r)
    m.clear()
    assert len(m.getContainer()) == 1
    return m


def test_shapes_and_dtypes(factory):
    arrays = _contact_test(
        _discrete_scene(factory, "BulletDiscreteBVHManager"), ContactTestType.ALL
    ).to_arrays()
    n = len(arrays.distance)
    for name, dtype in ARRAY_DTYPES.items():
        a = getattr(arrays, name)
        assert a.dtype == dtype, name
        assert a.shape == (n, *ARRAY_SHAPES[name]), name
    assert isinstance(arrays.links, tuple)


# --------------------------------------------------------------------------- #
# Ownership
# --------------------------------------------------------------------------- #
def test_arrays_survive_clear_and_release(factory):
    results = _contact_test(
        _discrete_scene(factory, "BulletDiscreteBVHManager"), ContactTestType.ALL
    )
    flat = results.flattenCopyResults()
    arrays = results.to_arrays()
    distance = arrays.distance
    results.clear()
    results.release()
    del results
    gc.collect()
    _assert_bitwise_equal(arrays, flat)
    assert distance.tobytes() == arrays.distance.tobytes()


def test_arrays_outlive_contact_arrays_object(factory):
    results = _contact_test(
        _discrete_scene(factory, "BulletDiscreteBVHManager"), ContactTestType.ALL
    )
    expected = results.flattenCopyResults()[0].distance
    normal = results.to_arrays().normal  # the ContactArrays is dropped at once
    distance = results.to_arrays().distance
    gc.collect()
    assert distance[0] == expected
    assert normal.shape[1] == 3


def test_writing_arrays_leaves_source_unchanged(factory):
    results = _contact_test(
        _discrete_scene(factory, "BulletDiscreteBVHManager"), ContactTestType.ALL
    )
    before = results.flattenCopyResults()
    arrays = results.to_arrays()
    for name in ARRAY_DTYPES:
        getattr(arrays, name)[...] = 0
    after = results.flattenCopyResults()
    assert len(after) == len(before)
    for r in range(len(before)):
        assert after[r] == before[r]
    _assert_bitwise_equal(results.to_arrays(), after)


def test_vector_to_arrays_while_element_held():
    """Reading moves nothing, so a held element does not block it (unlike clear/append)."""
    v = ContactResultVector()
    r = ContactResult()
    r.link_ids = ["a", "b"]
    r.distance = 0.5
    v.append(r)
    held = v[0]
    arrays = v.to_arrays()
    arrays.distance[0] = 9.0
    assert held.distance == 0.5
    del held


def test_contact_arrays_not_constructible():
    with pytest.raises(TypeError):
        ContactArrays()


def test_repr_names_rows_and_links():
    v = ContactResultVector()
    for a, b in [("a", "b"), ("a", "c")]:
        r = ContactResult()
        r.link_ids = [a, b]
        v.append(r)
    assert repr(v.to_arrays()) == "ContactArrays(n=2, links=3)"


# --------------------------------------------------------------------------- #
# Slot semantics: what the docs state about [k] fields and normal
# --------------------------------------------------------------------------- #
def _two_body_result(factory, manager_name, box_x, sphere_first):
    """One CLOSEST result between a sphere at the origin and a box centred at (box_x, 0, 0)."""
    manager = factory.createDiscreteContactManager(manager_name)
    sphere = ("sphere", 21, [Sphere(SLOT_SPHERE_RADIUS_M)], [_pose(0, 0, 0)])
    box = (
        "box",
        22,
        [Box(2 * SLOT_BOX_HALF_M, 2 * SLOT_BOX_HALF_M, 2 * SLOT_BOX_HALF_M)],
        [_pose(0, 0, 0)],
    )
    for name, mask, shapes, poses in (sphere, box) if sphere_first else (box, sphere):
        assert manager.addCollisionObject(name, mask, shapes, poses)
    manager.setCollisionObjectsTransform(
        {"sphere": _pose(0, 0, 0), "box": _pose(box_x, 0, 0, _rot_z(0.0))}
    )
    manager.setActiveCollisionObjects(["sphere", "box"])
    manager.setCollisionMarginData(CollisionMarginData(SLOT_MARGIN_M))
    arrays = _contact_test(manager, ContactTestType.CLOSEST).to_arrays()
    assert len(arrays.distance) == 1
    return arrays


@pytest.mark.parametrize(
    "sphere_first", [True, False], ids=["sphere_added_first", "box_added_first"]
)
@pytest.mark.parametrize(
    "box_x", [SLOT_BOX_X_SEPARATED_M, SLOT_BOX_X_PENETRATING_M], ids=["apart", "penetrating"]
)
@pytest.mark.parametrize("manager_name", DISCRETE_MANAGERS)
def test_slot_k_fields_belong_to_link_ids_k(factory, manager_name, box_x, sphere_first):
    a = _two_body_result(factory, manager_name, box_x, sphere_first)
    names = [a.links[a.link_index[0, k]].name() for k in range(2)]
    assert sorted(names) == ["box", "sphere"]
    centre = {"sphere": np.zeros(3), "box": np.array([box_x, 0.0, 0.0])}
    surface = {"sphere": SLOT_SPHERE_RADIUS_M, "box": box_x - SLOT_BOX_HALF_M}
    mask = {"sphere": 21, "box": 22}
    for k, name in enumerate(names):
        # transform[k] is link_ids[k]'s world pose, type_id[k] its mask id
        np.testing.assert_array_equal(a.transform[0, k, :3, 3], centre[name])
        assert a.type_id[0, k] == mask[name]
        # nearest_points[k] lies on link_ids[k], on the x axis at its surface facing the other
        np.testing.assert_allclose(
            a.nearest_points[0, k], [surface[name], 0.0, 0.0], atol=WITNESS_TOL_M
        )
        # nearest_points_local[k] is nearest_points[k] in link_ids[k]'s frame
        local = np.linalg.inv(a.transform[0, k]) @ np.append(a.nearest_points[0, k], 1.0)
        np.testing.assert_allclose(a.nearest_points_local[0, k], local[:3], atol=WITNESS_TOL_M)
    np.testing.assert_allclose(
        a.distance[0], box_x - SLOT_BOX_HALF_M - SLOT_SPHERE_RADIUS_M, atol=WITNESS_TOL_M
    )
    # normal points from link_ids[0] towards link_ids[1]: moving link_ids[1] along it separates them
    towards_1 = centre[names[1]] - centre[names[0]]
    np.testing.assert_allclose(
        a.normal[0], towards_1 / np.linalg.norm(towards_1), atol=WITNESS_TOL_M
    )


@pytest.mark.parametrize(
    "static_first", [True, False], ids=["static_added_first", "cast_added_first"]
)
@pytest.mark.parametrize("cast_name", ["a_mover", "z_mover"])
@pytest.mark.parametrize("manager_name", CAST_MANAGERS)
def test_bullet_cast_object_is_slot_1(factory, manager_name, cast_name, static_first):
    """Bullet's cast managers put the cast object of a cast-vs-static pair in slot 1."""
    manager = factory.createContinuousContactManager(manager_name)
    static = ("m_static", [Box(0.1, 1.0, 1.0)])
    cast = (cast_name, [Sphere(0.05)])
    for name, shapes in (static, cast) if static_first else (cast, static):
        assert manager.addCollisionObject(name, 0, shapes, [_pose(0, 0, 0)])
    manager.setActiveCollisionObjects([cast_name])
    manager.setCollisionMarginData(CollisionMarginData(0.05))
    manager.setCollisionObjectsTransform("m_static", _pose(0, 0, 0))
    manager.setCollisionObjectsTransform(cast_name, _pose(-1.0, 0.0, 0.0), _pose(1.0, 0.0, 0.0))
    a = _contact_test(manager, ContactTestType.CLOSEST).to_arrays()
    assert len(a.distance) == 1
    assert [a.links[i].name() for i in a.link_index[0]] == ["m_static", cast_name]
    none = ContinuousCollisionType.CCType_None.value
    assert a.cc_type[0, 0] == none and a.cc_time[0, 0] == -1.0
    assert a.cc_type[0, 1] != none and 0.0 <= a.cc_time[0, 1] <= 1.0
    # transform[1] / cc_transform[1] are the cast object's start / end poses
    np.testing.assert_array_equal(a.transform[0, 1, :3, 3], [-1.0, 0.0, 0.0])
    np.testing.assert_array_equal(a.cc_transform[0, 1, :3, 3], [1.0, 0.0, 0.0])
