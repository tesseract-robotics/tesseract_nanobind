"""
tesseract_robotics - Python bindings for Tesseract motion planning.

Importing the package changes nothing in the process environment. Data is located through
functions, which callers pass on explicitly:

    get_task_composer_config_path(): the task composer plugin config (bundled in the wheel)
    get_tesseract_resource_path(): the directory holding tesseract's `support` package, for
        GeneralResourceLocator(paths=[...]) or GeneralResourceLocator.addPath(); it comes from
        the tesseract-robotics-nanobind-support wheel (the `[support]` extra)
    get_tesseract_support_path(): tesseract's support/ directory itself (urdf/, meshes/, sdf/)

Plugins need no configuration: the plugin factory libraries are linked into the extension
modules, and each module's init registers them with the plugin loaders by path through their
anchors (TESSERACT_CONTACT_MANAGERS_PLUGINS, TESSERACT_KINEMATICS_PLUGINS,
TESSERACT_TASK_COMPOSER_PLUGINS), so the copies the wheel bundles are the ones loaded.
"""

from __future__ import annotations

import os
import sys
from importlib.metadata import PackageNotFoundError, version
from importlib.resources import files
from pathlib import Path


def _tesseract_dll_path_() -> None:
    """Windows: add the directories in TESSERACT_PYTHON_DLL_PATH to the DLL search path.

    Opt-in, as in the tesseract_python (SWIG) bindings: for a build whose tesseract DLLs live
    outside the package. A wheel needs nothing; delvewheel adds its bundled DLL directory.
    """
    if sys.platform == "win32":
        dll_path = os.environ.get("TESSERACT_PYTHON_DLL_PATH")
        if dll_path:
            for p in filter(None, dll_path.split(os.pathsep)):
                os.add_dll_directory(p)  # type: ignore[attr-defined]


_tesseract_dll_path_()
del _tesseract_dll_path_

try:
    __version__ = version("tesseract-robotics-nanobind")
except PackageNotFoundError:
    __version__ = "0.0.0.dev"  # fallback for editable installs without build


def _filesystem_path(resource: object) -> Path | None:
    """`resource` (an importlib.resources Traversable) as a filesystem path, if it is one."""
    return resource if isinstance(resource, Path) else None


def _conda_share(relative: str) -> Path | None:
    """`$CONDA_PREFIX/share/<relative>` (or conda on Windows' `Library/share`), if it exists.

    The editable install (pixi dev env) gets its data from the tesseract-robotics conda packages.
    """
    prefix = os.environ.get("CONDA_PREFIX")
    if not prefix:
        return None
    for share in (Path(prefix) / "share", Path(prefix) / "Library" / "share"):
        if (share / relative).exists():
            return share
    return None


def get_data_path() -> Path:
    """Directory of the data the bindings wheel bundles (the task composer config)."""
    path = _filesystem_path(files(__name__) / "data")
    if path is None:
        raise FileNotFoundError("tesseract_robotics is not installed as a directory on disk")
    return path


class SupportDataNotFoundError(FileNotFoundError):
    """tesseract's support data is not installed (the tesseract-robotics-nanobind-support wheel)."""


def get_tesseract_resource_path() -> Path:
    """Directory holding `tesseract/package.xml` and `tesseract/support/`.

    Pass it to a resource locator to resolve `package://tesseract/support/...`:

        GeneralResourceLocator(paths=[get_tesseract_resource_path()])

    It comes from the tesseract-robotics-nanobind-support wheel, or for an editable install from
    the conda env's share/.

    Raises:
        SupportDataNotFoundError: neither is installed.
    """
    try:
        import tesseract_robotics_support
    except ImportError:
        pass
    else:
        return tesseract_robotics_support.data_dir()
    share = _conda_share("tesseract/package.xml")
    if share is not None:
        return share
    raise SupportDataNotFoundError(
        "tesseract's support data (robot models and meshes) is not installed: "
        "pip install 'tesseract-robotics-nanobind[support]'"
    )


def get_tesseract_support_path() -> Path:
    """tesseract's support/ directory (urdf/, meshes/, sdf/).

    Raises:
        SupportDataNotFoundError: the support data is not installed.
    """
    return get_tesseract_resource_path() / "tesseract" / "support"


class TaskComposerConfigNotFoundError(FileNotFoundError):
    """No task composer config found via env var, bundled data, or conda share."""


_TASK_COMPOSER_CONFIG = "task_composer_plugins.yaml"


def get_task_composer_config_path() -> Path:
    """The task composer plugin config file.

    TESSERACT_TASK_COMPOSER_CONFIG_FILE when set, else the config the wheel bundles, else (editable
    install) the conda env's.

    Raises:
        TaskComposerConfigNotFoundError: nothing resolves to an existing file, or the env var
            names a file that does not exist.
    """
    env_cfg = os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE")
    if env_cfg:
        if Path(env_cfg).is_file():
            return Path(env_cfg)
        raise TaskComposerConfigNotFoundError(
            f"TESSERACT_TASK_COMPOSER_CONFIG_FILE={env_cfg!r} does not exist"
        )
    bundled = _filesystem_path(
        files(__name__) / "data" / "task_composer_config" / _TASK_COMPOSER_CONFIG
    )
    if bundled is not None and bundled.is_file():
        return bundled
    relative = f"tesseract_planning/task_composer/config/{_TASK_COMPOSER_CONFIG}"
    share = _conda_share(relative)
    if share is not None:
        return share / relative
    raise TaskComposerConfigNotFoundError(
        "no task composer config found: checked TESSERACT_TASK_COMPOSER_CONFIG_FILE, the bundled "
        "data/task_composer_config/, and $CONDA_PREFIX share"
    )
