"""ContactResultMap: read by key, iterate, and mutate from Python (gh-173).

Every value handed to Python is a copy (except the vector passed to a filter callback), so
Python never holds a reference into the map that a later insert or shrinkToFit invalidates.
"""

import pytest

import tesseract_robotics.tesseract_collision as tc
from tesseract_robotics.tesseract_collision import (
    ContactResult,
    ContactResultMap,
    ContactResultVector,
    ContinuousCollisionType,
)
from tesseract_robotics.tesseract_common import LinkIdPair

KEY_AB = ("a", "b")
KEY_AC = ("a", "c")
KEY_BC = ("b", "c")


def _result(link1, link2, distance=-0.1):
    r = ContactResult()
    r.link_ids = [link1, link2]
    r.distance = distance
    return r


def _vector(*results):
    v = ContactResultVector()
    for r in results:
        v.append(r)
    return v


def test_add_then_at_returns_copy():
    m = ContactResultMap()
    m.addContactResult(KEY_AB, _result("a", "b"))
    assert m.count() == 1

    v = m.at(KEY_AB)
    assert len(v) == 1
    assert list(v[0].link_ids) == ["a", "b"]

    v.append(_result("a", "b"))
    assert m.count() == 1
    assert len(m.at(KEY_AB)) == 1


def test_add_returns_copy_of_inserted_result():
    m = ContactResultMap()
    returned = m.addContactResult(KEY_AB, _result("a", "b", distance=-0.5))
    assert returned.distance == pytest.approx(-0.5)
    returned.distance = 1.0
    assert m.at(KEY_AB)[0].distance == pytest.approx(-0.5)


def test_set_replaces_results_for_key():
    m = ContactResultMap()
    m.addContactResult(KEY_AB, _result("a", "b", distance=-0.1))
    m.addContactResult(KEY_AB, _result("a", "b", distance=-0.2))
    assert m.count() == 2

    m.setContactResult(KEY_AB, _result("a", "b", distance=-0.3))
    assert m.count() == 1
    assert m.at(KEY_AB)[0].distance == pytest.approx(-0.3)


def test_vector_overloads_add_and_set_several():
    m = ContactResultMap()
    last = m.addContactResult(KEY_AB, _vector(_result("a", "b", -0.1), _result("a", "b", -0.2)))
    assert last.distance == pytest.approx(-0.2)
    assert m.count() == 2

    m.addContactResult(KEY_AB, _vector(_result("a", "b", -0.3)))
    assert m.count() == 3

    last = m.setContactResult(KEY_AB, _vector(_result("a", "b", -0.4), _result("a", "b", -0.5)))
    assert last.distance == pytest.approx(-0.5)
    assert m.count() == 2
    assert [r.distance for r in m.at(KEY_AB)] == pytest.approx([-0.4, -0.5])


@pytest.mark.parametrize("method", ["addContactResult", "setContactResult"])
def test_vector_overloads_reject_empty_results(method):
    m = ContactResultMap()
    with pytest.raises(tc.EmptyContactResultsError):
        getattr(m, method)(KEY_AB, ContactResultVector())
    assert issubclass(tc.EmptyContactResultsError, ValueError)
    assert m.count() == 0
    assert len(list(m)) == 0


@pytest.mark.parametrize("method", ["addContactResult", "setContactResult"])
@pytest.mark.parametrize("vector", [False, True], ids=["single", "vector"])
def test_reversed_key_addresses_same_entry(method, vector):
    # 0.36: the key is a canonical LinkIdPair (common/types.h OrderedIdPair), so ("b", "a")
    # and ("a", "b") name one entry; 0.35's UnorderedLinkPairError is unreachable.
    m = ContactResultMap()
    value = _vector(_result("b", "a")) if vector else _result("b", "a")
    getattr(m, method)(("b", "a"), value)
    assert (m.count(), m.size()) == (1, 1)
    assert len(m.at(KEY_AB)) == 1
    assert list(m.getContainer()) == [LinkIdPair(*KEY_AB)]

    getattr(m, method)(KEY_AB, value)
    assert m.size() == 1
    assert m.count() == (2 if method == "addContactResult" else 1)
    assert len(m.at(("b", "a"))) == m.count()


def test_at_missing_key_raises_key_error():
    m = ContactResultMap()
    m.addContactResult(KEY_AB, _result("a", "b"))
    with pytest.raises(KeyError):
        m.at(("x", "y"))


def test_get_container_is_dict_keyed_by_link_pair():
    m = ContactResultMap()
    m.addContactResult(KEY_AB, _result("a", "b"))
    m.addContactResult(KEY_BC, _result("b", "c"))
    container = m.getContainer()
    assert isinstance(container, dict)
    assert set(container) == {LinkIdPair(*KEY_AB), LinkIdPair(*KEY_BC)}
    assert len(container[LinkIdPair(*KEY_AB)]) == 1


def test_iteration_yields_key_vector_pairs_and_follows_clear_semantics():
    m = ContactResultMap()
    m.addContactResult(KEY_AB, _result("a", "b"))
    m.addContactResult(KEY_BC, _result("b", "c"))

    items = list(m)
    # unordered_map container (collision/types.h ContainerType): compare in name order
    assert sorted(key.orderedNameView() for key, _ in items) == [KEY_AB, KEY_BC]
    assert all(isinstance(v, ContactResultVector) and len(v) == 1 for _, v in items)

    # clear() empties the vectors but keeps the keys (types.h:142-144)
    m.clear()
    assert len(m) == 0
    assert len(list(m)) == 2
    m.shrinkToFit()
    assert len(list(m)) == 0


def test_iteration_survives_shrink_to_fit_mid_loop():
    m = ContactResultMap()
    m.addContactResult(KEY_AB, _result("a", "b"))
    m.addContactResult(KEY_BC, _result("b", "c"))
    m.clear()

    seen = []
    for key, vector in m:
        m.shrinkToFit()
        seen.append((key.orderedNameView(), len(vector)))
    assert sorted(seen) == [(KEY_AB, 0), (KEY_BC, 0)]


def test_filter_clears_pairs_involving_link():
    m = ContactResultMap()
    m.addContactResult(KEY_AB, _result("a", "b"))
    m.addContactResult(KEY_AC, _vector(_result("a", "c"), _result("a", "c")))
    m.addContactResult(KEY_BC, _result("b", "c"))
    assert (m.count(), m.size()) == (4, 3)

    calls = []

    def drop_a(key, results):
        calls.append(key.orderedNameView())
        if "a" in key.orderedNameView():
            results.clear()

    m.filter(drop_a)
    assert sorted(calls) == [KEY_AB, KEY_AC, KEY_BC]
    assert (m.count(), m.size()) == (1, 1)
    assert len(m.at(KEY_BC)) == 1


def test_filter_callback_exception_propagates():
    m = ContactResultMap()
    m.addContactResult(KEY_AB, _result("a", "b"))

    def boom(key, results):
        raise ValueError("rejected")

    with pytest.raises(ValueError, match="rejected"):
        m.filter(boom)


def _sub_segment_with_one_contact():
    sub = ContactResultMap()
    sub.addContactResult(KEY_AB, _result("a", "b"))
    return sub


def test_add_interpolated_collision_results_sets_cc_fields():
    full = ContactResultMap()
    sub = _sub_segment_with_one_contact()
    # sub-segment 1 of 0..3, dt 0.25: an active link with cc_time < 0 gets 1 * 0.25
    full.addInterpolatedCollisionResults(sub, 1, 3, ["a"], 0.25, False)

    assert full.count() == 1
    r = full.at(KEY_AB)[0]
    assert r.cc_time[0] == pytest.approx(0.25)
    assert r.cc_type[0] == ContinuousCollisionType.CCType_Between
    assert r.cc_time[1] == pytest.approx(-1.0)
    assert r.cc_type[1] == ContinuousCollisionType.CCType_None


def test_add_interpolated_collision_results_filter_rejects_pair():
    full = ContactResultMap()
    sub = _sub_segment_with_one_contact()

    def reject_all(key, results):
        results.clear()

    full.addInterpolatedCollisionResults(sub, 1, 3, ["a"], 0.25, False, reject_all)
    assert full.count() == 0
    assert len(list(full)) == 0
