"""CollisionCoeffData whole-table reads (#293); the non-C++ alias names are gone (#298)."""

from __future__ import annotations

from tesseract_robotics import trajopt_ifopt as ti


def _unordered(pairs):
    return {frozenset(pair) for pair in pairs}


def test_pair_tables():
    data = ti.CollisionCoeffData()
    data.setCollisionCoeff("a", "b", 0.0)
    data.setCollisionCoeff("a", "c", 2.0)

    table = data.getCollisionCoeffPairData()
    assert {frozenset(pair): coeff for pair, coeff in table.items()} == {
        frozenset(("a", "b")): 0.0,
        frozenset(("a", "c")): 2.0,
    }
    assert _unordered(data.getPairsWithZeroCoeff()) == {frozenset(("a", "b"))}


def test_alias_names_are_gone():
    data = ti.CollisionCoeffData()
    assert not hasattr(data, "setPairCollisionCoeff")
    assert not hasattr(data, "getPairCollisionCoeff")
