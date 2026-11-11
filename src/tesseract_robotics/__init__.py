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

import importlib
import importlib.util
import os
import sys
from importlib.metadata import distribution, version
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

# Raises importlib.metadata.PackageNotFoundError when the distribution is not installed: an
# editable install carries metadata too, so a missing version means a broken install.
__version__ = version("tesseract-robotics-nanobind")


_DISTRIBUTION = "tesseract-robotics-nanobind"


def get_data_path() -> Path:
    """Directory of the data the bindings distribution installs (the task composer config).

    Located through the distribution's own file record, not the package directory: an editable
    install imports `tesseract_robotics` from the source tree while CMake installs the data
    next to the extension modules in site-packages, and `importlib.metadata` resolves the
    installed location for both a wheel and an editable install.

    Raises:
        FileNotFoundError: the distribution's data directory does not exist on disk.
    """
    path = Path(str(distribution(_DISTRIBUTION).locate_file("tesseract_robotics/data")))
    if not path.is_dir():
        raise FileNotFoundError(f"{_DISTRIBUTION} data directory {path} does not exist")
    return path


# Import name of the tesseract-robotics-nanobind-support wheel (the [support] extra).
SUPPORT_PACKAGE = "tesseract_robotics_support"


class SupportDataNotFoundError(FileNotFoundError):
    """tesseract's support data is not installed (the tesseract-robotics-nanobind-support wheel)."""


def get_tesseract_resource_path() -> Path:
    """Directory holding `tesseract/package.xml` and `tesseract/support/`.

    Pass it to a resource locator to resolve `package://tesseract/support/...`:

    ```python
    GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    ```

    The one source is the tesseract-robotics-nanobind-support distribution, installed by the
    `[support]` extra and by the development environment's `install` task.

    Raises:
        SupportDataNotFoundError: the support distribution is not installed.
    """
    # The [support] extra is optional: absent means no spec. A present but broken install raises
    # its own error from the unguarded import instead of reading as absent.
    if importlib.util.find_spec(SUPPORT_PACKAGE) is None:
        raise SupportDataNotFoundError(
            "tesseract's support data (robot models and meshes) is not installed: "
            "pip install 'tesseract-robotics-nanobind[support]'"
        )
    return importlib.import_module(SUPPORT_PACKAGE).data_dir()


def get_tesseract_support_path() -> Path:
    """tesseract's support/ directory (urdf/, meshes/, sdf/).

    Raises:
        SupportDataNotFoundError: the support data is not installed.
    """
    return get_tesseract_resource_path() / "tesseract" / "support"


class TaskComposerConfigNotFoundError(FileNotFoundError):
    """No task composer config: the env var names a missing file, or the install lacks one."""


_TASK_COMPOSER_CONFIG = "task_composer_plugins.yaml"


def get_task_composer_config_path() -> Path:
    """The task composer plugin config file.

    TESSERACT_TASK_COMPOSER_CONFIG_FILE when set, else the config the distribution installs
    under `data/task_composer_config/` (wheel and editable install alike).

    Raises:
        TaskComposerConfigNotFoundError: the env var names a file that does not exist, or the
            installed config is missing.
    """
    env_cfg = os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE")
    if env_cfg:
        if Path(env_cfg).is_file():
            return Path(env_cfg)
        raise TaskComposerConfigNotFoundError(
            f"TESSERACT_TASK_COMPOSER_CONFIG_FILE={env_cfg!r} does not exist"
        )
    installed = get_data_path() / "task_composer_config" / _TASK_COMPOSER_CONFIG
    if not installed.is_file():
        raise TaskComposerConfigNotFoundError(
            f"{_DISTRIBUTION} installed no task composer config at {installed}"
        )
    return installed
