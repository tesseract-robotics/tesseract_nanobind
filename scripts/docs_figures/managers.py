"""Bare contact managers from the support plugin config, for the collision figures."""

from __future__ import annotations

from pathlib import Path
from typing import Final

import numpy as np

from scripts.docs_figures import lock
from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.collision import (
    ContactManagersPluginFactory,
    ContactRequest,
    ContactResult,
    ContactResultMap,
    ContactTestType,
    ContinuousContactManager,
    DiscreteContactManager,
)
from tesseract_robotics.common import GeneralResourceLocator, Isometry3d

PLUGINS: Final[str] = "support/urdf/contact_manager_plugins.yaml"


class NoContactError(LookupError):
    """A contact test the figure relies on reported no contact."""


def factory() -> ContactManagersPluginFactory:
    """The plugin factory of upstream's support config (Bullet defaults)."""
    locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    return ContactManagersPluginFactory(lock.support_root() / PLUGINS, locator)


def discrete(name: str | None = None) -> DiscreteContactManager:
    """A discrete manager: the config's default, or the named plugin."""
    f = factory()
    return f.createDiscreteContactManager(name or f.getDefaultDiscreteContactManagerPlugin())


def continuous() -> ContinuousContactManager:
    """The config's default continuous (cast) manager."""
    f = factory()
    return f.createContinuousContactManager(f.getDefaultContinuousContactManagerPlugin())


def translation(x: float, y: float = 0.0, z: float = 0.0) -> Isometry3d:
    """A pure translation, m."""
    matrix = np.eye(4)
    matrix[:3, 3] = [x, y, z]
    return Isometry3d(matrix)


def closest(manager: DiscreteContactManager | ContinuousContactManager) -> ContactResult | None:
    """The closest contact within the margin, or None when there is none."""
    results = ContactResultMap()
    manager.contactTest(results, ContactRequest(ContactTestType.CLOSEST))
    flat = results.flattenCopyResults()
    if len(flat) == 0:
        return None
    return min((flat[i] for i in range(len(flat))), key=lambda c: c.distance)


def inputs() -> list[Path]:
    """This module and the plugin config the managers come from."""
    return [Path(__file__), *lock.support_files([PLUGINS])]
