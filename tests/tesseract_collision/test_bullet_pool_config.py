"""The #140 workaround: smaller Bullet pool allocators set in the contact-manager plugin config.

Bullet's default pools (4096 persistent manifolds, 4096 collision algorithms) cost ~7 MiB per
manager clone; `max_persistent_manifold_pool_size` / `max_collision_algorithm_pool_size` in the
plugin's `config:` shrink them. Pool size is capacity, not semantics: the contacts are the same.
"""

import gc

import numpy as np
import pytest

from tesseract_robotics.collision import (
    ContactManagersPluginFactory,
    ContactRequest,
    ContactResultMap,
    ContactTestType,
)
from tesseract_robotics.common import CollisionMarginData, Isometry3d

from ..tesseract_support_resource_locator import TesseractSupportResourceLocator
from .test_contact_manager_config import _box

# entries: the pool sizes #140 measured with (16/16 against Bullet's 4096/4096)
SMALL_POOL = 16
# m: contact margin, so near-misses also report a distance
MARGIN = 0.1
# m: x of the second unit box against one at the origin: overlapping, touching within the
# margin, and apart
POSES = (0.5, 1.05, 3.0)

_CONFIG = """
contact_manager_plugins:
  search_libraries:
    - tesseract_collision_bullet_factories
  discrete_plugins:
    default: BulletDiscreteBVHManager
    plugins:
      BulletDiscreteBVHManager:
        class: BulletDiscreteBVHManagerFactory
{config}
"""
_POOLS = f"""        config:
          max_persistent_manifold_pool_size: {SMALL_POOL}
          max_collision_algorithm_pool_size: {SMALL_POOL}"""


def _pose(x: float) -> Isometry3d:
    m = np.eye(4)
    m[0, 3] = x
    return Isometry3d(m)


def _contacts_at(manager, x: float) -> list[tuple[str, str, float]]:
    manager.setCollisionObjectsTransform("b", _pose(x))
    result = ContactResultMap()
    manager.contactTest(result, ContactRequest(ContactTestType.ALL))
    found = []
    for contact in result.flattenCopyResults():
        a, b = sorted(link.name() for link in contact.link_ids)
        found.append((a, b, contact.distance))
    return sorted(found)


def _contacts(config: str) -> list[list[tuple[str, str, float]]]:
    factory = ContactManagersPluginFactory(
        _CONFIG.format(config=config), TesseractSupportResourceLocator()
    )
    manager = factory.createDiscreteContactManager("BulletDiscreteBVHManager")
    try:
        for name in ("a", "b"):
            shapes, poses = _box()
            assert manager.addCollisionObject(name, 0, shapes, poses)
        manager.setActiveCollisionObjects(["a", "b"])
        manager.setCollisionMarginData(CollisionMarginData(MARGIN))
        # A clone carries the configured pools too (TesseractCollisionConfigurationInfo::clone)
        clone = manager.clone()
        found = [_contacts_at(m, x) for m in (manager, clone) for x in POSES]
        del clone
        return found
    finally:
        del manager, factory
        gc.collect()


def test_small_pools_same_contacts():
    default = _contacts("")
    small = _contacts(_POOLS)
    assert default[0], "the overlapping boxes must touch"
    assert default[-1] == [], "the separated boxes must not touch"
    assert [[c[:2] for c in at] for at in small] == [[c[:2] for c in at] for at in default]
    for small_at, default_at in zip(small, default):
        assert [c[2] for c in small_at] == pytest.approx([c[2] for c in default_at])
