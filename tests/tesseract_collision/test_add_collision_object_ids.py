"""addCollisionObject takes the `LinkId` its C++ signature names; `str` converts implicitly."""

import pytest

from tesseract_robotics.tesseract_common import LinkId

from .test_contact_manager_config import _box, _get_discrete_factory


@pytest.mark.parametrize(
    "create",
    [
        lambda factory: factory.createDiscreteContactManager("BulletDiscreteBVHManager"),
        lambda factory: factory.createContinuousContactManager("BulletCastBVHManager"),
    ],
    ids=["discrete", "continuous"],
)
@pytest.mark.parametrize("object_id", [LinkId("box"), "box"], ids=["LinkId", "str"])
def test_add_collision_object_takes_a_link_id(create, object_id):
    factory, locator = _get_discrete_factory()
    manager = create(factory)
    try:
        shapes, poses = _box()
        assert manager.addCollisionObject(object_id, 0, shapes, poses)
        assert manager.hasCollisionObject(LinkId("box"))
    finally:
        # The manager comes from a plugin the factory keeps loaded.
        del manager, factory, locator
