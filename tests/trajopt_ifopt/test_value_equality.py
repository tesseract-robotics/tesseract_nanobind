"""trajopt_common value types compare by value and are unhashable (#261)."""

from __future__ import annotations

import pytest

from tesseract_robotics import trajopt_ifopt as ti

# collision margin / coefficient of the configs under test; any values work
MARGIN = 0.025
COEFF = 20.0


def test_collision_coeff_data():
    a, b = ti.CollisionCoeffData(), ti.CollisionCoeffData()
    assert a == b
    assert not (a != b)
    b.setCollisionCoeff("link_a", "link_b", 2.0)
    assert a != b
    a.setCollisionCoeff("link_b", "link_a", 2.0)
    assert a == b


def test_trajopt_collision_config():
    a, b = ti.TrajOptCollisionConfig(MARGIN, COEFF), ti.TrajOptCollisionConfig(MARGIN, COEFF)
    assert a == b
    b.max_num_cnt = a.max_num_cnt + 1
    assert a != b
    assert ti.TrajOptCollisionConfig() == ti.TrajOptCollisionConfig()


@pytest.mark.parametrize("cls", [ti.CollisionCoeffData, ti.TrajOptCollisionConfig])
def test_unhashable(cls):
    assert cls.__hash__ is None
    with pytest.raises(TypeError, match="unhashable"):
        hash(cls())
