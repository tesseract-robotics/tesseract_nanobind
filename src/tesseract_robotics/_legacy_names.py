"""Aliases for the 0.35 module names, kept for one release.

Up to 0.35 every component was `tesseract_robotics.tesseract_<x>` (the compiled module under
`tesseract_robotics.tesseract_<x>._tesseract_<x>`). 0.36 names them after tesseract's CMake
components: `tesseract_robotics.<x>`. Importing an old name emits a `DeprecationWarning` and
returns the module object of the new name, so both spellings share one set of types.
"""

from __future__ import annotations

import importlib
import importlib.abc
import importlib.machinery
import sys
import warnings
from types import ModuleType

PACKAGE = "tesseract_robotics"

# Components renamed in 0.36 (`tesseract_<x>` -> `<x>`). `trajopt_ifopt` and `trajopt_sqp` kept their names.
RENAMED = (
    "collision",
    "command_language",
    "common",
    "environment",
    "geometry",
    "kinematics",
    "motion_planners",
    "motion_planners_descartes",
    "motion_planners_ompl",
    "motion_planners_simple",
    "motion_planners_trajopt",
    "motion_planners_trajopt_ifopt",
    "scene_graph",
    "serialization",
    "srdf",
    "state_solver",
    "task_composer",
    "task_composer_planning",
    "time_parameterization",
    "urdf",
)

# Packages that keep Python code beside the compiled `_<x>` extension; every other component is
# the compiled module itself.
WRAPPED = frozenset(
    {
        "common",
        "scene_graph",
        "motion_planners",
        "motion_planners_simple",
        "motion_planners_descartes",
        "motion_planners_trajopt_ifopt",
    }
)

_OLD_PREFIX = f"{PACKAGE}.tesseract_"


def _target(fullname: str) -> str | None:
    """The 0.36 module name for a 0.35 module name, or None if `fullname` is not an alias."""
    if not fullname.startswith(_OLD_PREFIX):
        return None
    old_leaf, _, rest = fullname[len(_OLD_PREFIX) :].partition(".")
    if old_leaf not in RENAMED:
        return None
    new_package = f"{PACKAGE}.{old_leaf}"
    if not rest:
        return new_package
    if rest == f"_tesseract_{old_leaf}":
        return f"{new_package}._{old_leaf}" if old_leaf in WRAPPED else new_package
    if old_leaf in WRAPPED and "." not in rest:
        return f"{new_package}.{rest}"
    return None


class LegacyNameFinder(importlib.abc.MetaPathFinder, importlib.abc.Loader):
    """Resolves `tesseract_robotics.tesseract_<x>[...]` to the module of its 0.36 name."""

    def find_spec(
        self,
        fullname: str,
        path: object = None,
        target: ModuleType | None = None,
    ) -> importlib.machinery.ModuleSpec | None:
        new = _target(fullname)
        if new is None:
            return None
        try:
            importlib.import_module(new)
        except ModuleNotFoundError:
            return None
        return importlib.machinery.ModuleSpec(fullname, self)

    def create_module(self, spec: importlib.machinery.ModuleSpec) -> ModuleType:
        new = _target(spec.name)
        assert new is not None  # find_spec only returns specs for aliases
        if spec.name == _top_of(spec.name):
            warnings.warn(
                f"{spec.name} is now {new}; the old name is removed in a future release",
                DeprecationWarning,
                stacklevel=2,
            )
        return importlib.import_module(new)

    def exec_module(self, module: ModuleType) -> None:
        """Nothing to execute: `create_module` returned the already-initialised target."""


def _top_of(fullname: str) -> str:
    """`tesseract_robotics.tesseract_x` for any `tesseract_robotics.tesseract_x[.y]`."""
    return ".".join(fullname.split(".")[:2])


def install() -> None:
    """Register the finder once."""
    if not any(isinstance(f, LegacyNameFinder) for f in sys.meta_path):
        sys.meta_path.insert(
            0, LegacyNameFinder()
        )  # before PathFinder: it would load a wrapped package's children a second time
