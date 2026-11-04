"""CollisionCoeffData whole-table reads (#293); the non-C++ alias names are gone (#298)."""

from __future__ import annotations

from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics.tesseract_common import LinkIdPair


def _unordered(pairs):
    return {frozenset(pair.orderedNameView()) for pair in pairs}


def test_pair_tables():
    data = ti.CollisionCoeffData()
    data.setCollisionCoeff("a", "b", 0.0)
    data.setCollisionCoeff("a", "c", 2.0)

    table = data.getCollisionCoeffPairData()
    assert {frozenset(pair.orderedNameView()): coeff for pair, coeff in table.items()} == {
        frozenset(("a", "b")): 0.0,
        frozenset(("a", "c")): 2.0,
    }
    assert _unordered(data.getPairsWithZeroCoeff()) == {frozenset(("a", "b"))}


def test_alias_names_are_gone():
    data = ti.CollisionCoeffData()
    assert not hasattr(data, "setPairCollisionCoeff")
    assert not hasattr(data, "getPairCollisionCoeff")


def test_pairs_with_zero_coeff_is_a_set_of_link_id_pairs():
    """0.36 returns `unordered_set<LinkIdPair>`: it needs nanobind's unordered_set caster."""
    data = ti.CollisionCoeffData()
    data.setCollisionCoeff("b", "a", 0.0)
    data.setCollisionCoeff("a", "c", 2.0)
    assert data.getPairsWithZeroCoeff() == {LinkIdPair("a", "b")}
