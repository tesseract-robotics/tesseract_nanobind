"""ContactManagersPluginFactory: the full plugin API, and managers tied to the factory (gh-170).

Managers created by the factory run code from plugin libraries the factory's PluginLoader
loaded, so each manager keeps its factory alive (the gh-72 keep_alive<0, 1> rule).
"""

import gc
import subprocess
import sys

import pytest
import yaml

from tesseract_robotics.tesseract_collision import (
    ContactManagersPluginFactory,
    ContactRequest,
    ContactResultMap,
    ContactTestType_ALL,
)
from tesseract_robotics.tesseract_common import Isometry3d

from .test_contact_manager_config import _get_discrete_factory, _two_overlapping_boxes

DISCRETE = "Discrete"
CONTINUOUS = "Continuous"
KINDS = [DISCRETE, CONTINUOUS]


@pytest.fixture
def factory():
    factory, locator = _get_discrete_factory()
    yield factory
    del factory
    del locator
    gc.collect()


def _method(factory, template, kind):
    return getattr(factory, template.format(kind=kind))


@pytest.mark.parametrize("kind", KINDS)
def test_get_plugins_contains_default(factory, kind):
    plugins = _method(factory, "get{kind}ContactManagerPlugins", kind)()
    assert isinstance(plugins, dict)
    assert _method(factory, "getDefault{kind}ContactManagerPlugin", kind)() in plugins


@pytest.mark.parametrize("kind", KINDS)
def test_add_set_default_remove_plugin(factory, kind):
    plugins = _method(factory, "get{kind}ContactManagerPlugins", kind)()
    default = _method(factory, "getDefault{kind}ContactManagerPlugin", kind)()

    _method(factory, "add{kind}ContactManagerPlugin", kind)("Copied", plugins[default])
    added = _method(factory, "get{kind}ContactManagerPlugins", kind)()
    assert added["Copied"].class_name == plugins[default].class_name

    _method(factory, "setDefault{kind}ContactManagerPlugin", kind)("Copied")
    assert _method(factory, "getDefault{kind}ContactManagerPlugin", kind)() == "Copied"

    _method(factory, "remove{kind}ContactManagerPlugin", kind)("Copied")
    assert "Copied" not in _method(factory, "get{kind}ContactManagerPlugins", kind)()


@pytest.mark.parametrize("kind", KINDS)
@pytest.mark.parametrize(
    "template", ["remove{kind}ContactManagerPlugin", "setDefault{kind}ContactManagerPlugin"]
)
def test_unknown_plugin_name_raises_key_error(factory, kind, template):
    with pytest.raises(KeyError, match="NoSuchPlugin"):
        _method(factory, template, kind)("NoSuchPlugin")


def test_create_discrete_from_plugin_info(factory):
    info = factory.getDiscreteContactManagerPlugins()["BulletDiscreteBVHManager"]
    checker = factory.createDiscreteContactManager("from_info", info)
    _two_overlapping_boxes(checker)
    result = ContactResultMap()
    checker.contactTest(result, ContactRequest(ContactTestType_ALL))
    assert result.count() == 1


def test_create_continuous_from_plugin_info(factory):
    info = factory.getContinuousContactManagerPlugins()["BulletCastBVHManager"]
    checker = factory.createContinuousContactManager("from_info", info)
    _two_overlapping_boxes(checker)
    for name in ("box_a", "box_b"):
        checker.setCollisionObjectsTransform(name, Isometry3d())
    result = ContactResultMap()
    checker.contactTest(result, ContactRequest(ContactTestType_ALL))
    assert result.count() == 1


def test_clear_search_paths_and_libraries(factory):
    assert factory.getSearchLibraries()
    factory.clearSearchPaths()
    factory.clearSearchLibraries()
    assert factory.getSearchPaths() == []
    assert factory.getSearchLibraries() == []


def test_get_config_is_yaml_with_both_plugin_sections(factory):
    config = yaml.safe_load(factory.getConfig())
    section = config["contact_manager_plugins"]
    assert "BulletDiscreteBVHManager" in section["discrete_plugins"]["plugins"]
    assert "BulletCastBVHManager" in section["continuous_plugins"]["plugins"]


def test_save_config_round_trips(factory, tmp_path):
    path = tmp_path / "cm.yaml"
    factory.saveConfig(path)
    _, locator = _get_discrete_factory()
    reloaded = ContactManagersPluginFactory(path, locator)
    assert set(reloaded.getDiscreteContactManagerPlugins()) == set(
        factory.getDiscreteContactManagerPlugins()
    )
    assert set(reloaded.getContinuousContactManagerPlugins()) == set(
        factory.getContinuousContactManagerPlugins()
    )


def test_save_config_missing_directory_raises(factory, tmp_path):
    with pytest.raises(FileNotFoundError):
        factory.saveConfig(tmp_path / "missing" / "cm.yaml")


_MANAGER_OUTLIVES_FACTORY_SCRIPT = """\
import gc
import os
from pathlib import Path

import numpy as np

import tesseract_robotics  # noqa: F401 - sets TESSERACT_SUPPORT_DIR
from tesseract_robotics.tesseract_collision import (
    ContactManagersPluginFactory, ContactRequest, ContactResultMap, ContactTestType_ALL,
)
from tesseract_robotics.tesseract_common import (
    CollisionMarginData, GeneralResourceLocator, Isometry3d, VectorIsometry3d,
)
from tesseract_robotics.tesseract_geometry import Box, GeometriesConst

cfg = Path(os.environ["TESSERACT_SUPPORT_DIR"]) / "urdf" / "contact_manager_plugins.yaml"
factory = ContactManagersPluginFactory(cfg, GeneralResourceLocator())
checker = {create}
del factory
gc.collect()

for name in ("box_a", "box_b"):
    shapes = GeometriesConst()
    shapes.append(Box(1.0, 1.0, 1.0))
    poses = VectorIsometry3d()
    poses.append(Isometry3d(np.eye(4)))
    checker.addCollisionObject(name, 0, shapes, poses)
checker.setActiveCollisionObjects(["box_a", "box_b"])
checker.setCollisionMarginData(CollisionMarginData(0.1))
result = ContactResultMap()
checker.contactTest(result, ContactRequest(ContactTestType_ALL))
print(f"OK: {{result.count()}}")
"""


@pytest.mark.parametrize(
    "create",
    [
        'factory.createDiscreteContactManager("BulletDiscreteBVHManager")',
        "factory.createDiscreteContactManager(\n"
        '    "from_info", factory.getDiscreteContactManagerPlugins()["BulletDiscreteBVHManager"]\n'
        ")",
    ],
    ids=["by_name", "by_plugin_info"],
)
def test_manager_outlives_factory(create):
    """gh-72 rule: `del factory` must not dlclose the plugin the manager runs from.

    Before keep_alive<0, 1> was added, the by_name case already passed on macOS
    (2026-10-06, `.scratch/phaseB-a-170-lifetime-red.log`): this process did not crash
    after the factory was dropped. The tie is kept anyway, as the gh-72 rule requires,
    since unmapping on dlclose is platform-dependent.
    """
    script = _MANAGER_OUTLIVES_FACTORY_SCRIPT.format(create=create)
    proc = subprocess.run(
        [sys.executable, "-c", script], capture_output=True, text=True, check=False
    )
    assert proc.returncode == 0, f"rc={proc.returncode} (SIGSEGV is -11): {proc.stderr[-800:]}"
    assert "OK: 1" in proc.stdout
