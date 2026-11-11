"""The 0.35 module names (`tesseract_robotics.tesseract_x`) alias the 0.36 names (`tesseract_robotics.x`).

An old name imports, warns once with a DeprecationWarning, and resolves to the very module
object of the new name, so classes and enums are identical across both spellings.
"""

from __future__ import annotations

import importlib
import sys

import pytest

# Modules whose package keeps Python code around the compiled `_x` extension.
WRAPPED = {
    "common",
    "scene_graph",
    "motion_planners",
    "motion_planners_simple",
    "motion_planners_descartes",
    "motion_planners_trajopt_ifopt",
}
RENAMED = sorted(
    WRAPPED
    | {
        "collision",
        "command_language",
        "environment",
        "geometry",
        "kinematics",
        "motion_planners_ompl",
        "motion_planners_trajopt",
        "serialization",
        "srdf",
        "state_solver",
        "task_composer",
        "task_composer_planning",
        "time_parameterization",
        "urdf",
    }
)
PKG = "tesseract_robotics"


def _forget(old: str) -> None:
    """Drop `old` from the import caches so the next import runs the alias finder again."""
    sys.modules.pop(old, None)
    parent, _, leaf = old.rpartition(".")
    parent_module = sys.modules.get(parent)
    if parent_module is not None and hasattr(parent_module, leaf):
        delattr(parent_module, leaf)


@pytest.mark.parametrize("name", RENAMED)
def test_new_name_imports(name: str) -> None:
    assert importlib.import_module(f"{PKG}.{name}").__name__ == f"{PKG}.{name}"


@pytest.mark.parametrize("name", RENAMED)
def test_old_name_warns_and_is_the_new_module(name: str) -> None:
    new = importlib.import_module(f"{PKG}.{name}")
    old_name = f"{PKG}.tesseract_{name}"
    _forget(old_name)
    with pytest.warns(DeprecationWarning, match=rf"{old_name} is now {PKG}\.{name}\b"):
        old = importlib.import_module(old_name)
    assert old is new


@pytest.mark.parametrize("name", sorted(WRAPPED))
def test_old_extension_path_is_the_new_extension(name: str) -> None:
    """`tesseract_robotics.tesseract_x._tesseract_x` was the compiled module.

    Only for the wrapped packages: a direct-install module is a compiled file, not a package,
    so it has no private submodule to alias.
    """
    new = importlib.import_module(f"{PKG}.{name}._{name}")
    old_name = f"{PKG}.tesseract_{name}._tesseract_{name}"
    _forget(old_name)
    _forget(f"{PKG}.tesseract_{name}")
    with pytest.warns(DeprecationWarning):
        old = importlib.import_module(old_name)
    assert old is new


def test_from_import_through_old_name() -> None:
    from tesseract_robotics.collision import ContactResult as new_cls

    _forget(f"{PKG}.tesseract_collision")
    with pytest.warns(DeprecationWarning):
        from tesseract_robotics.tesseract_collision import ContactResult as old_cls
    assert old_cls is new_cls


def test_wrapped_helper_submodule_through_old_name() -> None:
    new = importlib.import_module(f"{PKG}.motion_planners._profile_overrides")
    _forget(f"{PKG}.tesseract_motion_planners._profile_overrides")
    _forget(f"{PKG}.tesseract_motion_planners")
    with pytest.warns(DeprecationWarning):
        old = importlib.import_module(f"{PKG}.tesseract_motion_planners._profile_overrides")
    assert old is new


def test_trajopt_names_are_unchanged() -> None:
    for name in ("trajopt_ifopt", "trajopt_sqp"):
        assert importlib.import_module(f"{PKG}.{name}").__name__ == f"{PKG}.{name}"


def test_unknown_old_name_is_not_found() -> None:
    with pytest.raises(ModuleNotFoundError):
        importlib.import_module(f"{PKG}.tesseract_nonexistent")
    with pytest.raises(ModuleNotFoundError):
        importlib.import_module(f"{PKG}.tesseract_collision.not_a_module")
