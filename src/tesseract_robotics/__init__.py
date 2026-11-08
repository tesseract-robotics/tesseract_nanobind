"""
tesseract_robotics - Python bindings for Tesseract motion planning.

Environment Variables (auto-configured on first use):
    TESSERACT_SUPPORT_DIR: Path to tesseract_support (URDF/meshes)
    TESSERACT_RESOURCE_PATH: Resource search path for URDFs
    TESSERACT_TASK_COMPOSER_CONFIG_FILE: Task composer YAML config
    TESSERACT_TASK_COMPOSER_DIR: Task composer config directory

Priority: User env vars > Bundled data (installed) > conda share (editable)

Plugins need no configuration: the plugin factory libraries are linked into the extension
modules, and each module's init registers them with the plugin loaders by path through their
anchors (TESSERACT_CONTACT_MANAGERS_PLUGINS, TESSERACT_KINEMATICS_PLUGINS,
TESSERACT_TASK_COMPOSER_PLUGINS), so the copies the wheel bundles are the ones loaded.
"""

from __future__ import annotations

import os
import sys
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

# Linux: libstdc++ is intentionally NOT bundled in the wheels, and never
# preloaded. Two libstdc++ copies in one process (bundled via $ORIGIN rpath +
# system via numpy/scipy) unify their STB_GNU_UNIQUE locale statics across
# copies, and std::regex corrupts the heap during Environment.init — see
# docs/developer/linux-wheels.md (gh-119; supersedes the GH #35 preload, whose
# "two copies coexist safely" premise was falsified). The host or conda/pixi
# env must provide a gcc-13+ era libstdc++ (ubuntu 24.04+, debian 13+); older
# distros fail at import with a clear missing-GLIBCXX error instead of
# corrupting silently.

# Windows: extend the DLL search path before any C extension import.
#
# os.add_dll_directory() (delvewheel auto-injects one for tesseract_robotics_nanobind.libs/)
# only takes effect for LoadLibrary calls that opt in via LOAD_LIBRARY_SEARCH_USER_DIRS.
# Plain LoadLibrary("foo.dll") from C++ — which is what boost::dll (and therefore
# boost_plugin_loader) does — uses the legacy DLL search order, which respects PATH
# but NOT add_dll_directory. So plugin DLLs LoadLibrary fine in pkg_dir, but their
# transitive imports of bundled deps in tesseract_robotics_nanobind.libs/ go
# unresolved and every plugin instantiation fails with "Failed to load symbol 'X'".
#
# Prepending PATH covers boost::dll's plain LoadLibrary; add_dll_directory keeps the
# Python-native loader paths working in parallel.
if sys.platform == "win32":
    _pkg_dir = Path(__file__).parent.resolve()
    # delvewheel uses the project metadata name (tesseract-robotics-nanobind), not
    # the Python package name (tesseract_robotics).
    _libs_dir = _pkg_dir.parent / "tesseract_robotics_nanobind.libs"
    _extra = [str(_pkg_dir)] + ([str(_libs_dir)] if _libs_dir.is_dir() else [])
    os.environ["PATH"] = os.pathsep.join(filter(None, [*_extra, os.environ.get("PATH")]))
    for _d in _extra:
        os.add_dll_directory(_d)  # type: ignore[attr-defined]

try:
    __version__ = version("tesseract-robotics-nanobind")
except PackageNotFoundError:
    __version__ = "0.0.0.dev"  # fallback for editable installs without build


def _set_env_if_missing(var_name: str, *candidates: Path, use_parent: bool = False) -> None:
    """Set env var to first existing path if not already set."""
    if var_name in os.environ:
        return
    for path in candidates:
        if path.is_dir():
            os.environ[var_name] = str(path.parent if use_parent else path)
            return


def _configure_environment():
    """Set env vars using bundled data paths or dev workspace (if not already set)."""
    pkg_dir = Path(__file__).parent.resolve()

    # Try bundled data first (installed package)
    data_dir = pkg_dir / "data"
    support_dir = data_dir / "tesseract" / "support"
    config_dir = data_dir / "task_composer_config"

    # Fallback: editable install (pixi dev env). The tesseract C++ libs + data come
    # from the tesseract-robotics conda packages under $CONDA_PREFIX — data under
    # share/ (Library/share on conda Windows), plugin libs under lib/ (Library/bin
    # on Windows). Mirrors the layout the wheel bundles into the package's data/ dir.
    conda_prefix = os.environ.get("CONDA_PREFIX")
    conda_share = Path(conda_prefix) / "share" if conda_prefix else None
    if conda_share and not (conda_share / "tesseract").is_dir():
        # conda on Windows installs data under the Library/ prefix
        alt = Path(conda_prefix) / "Library" / "share"
        if (alt / "tesseract").is_dir():
            conda_share = alt
    ws_support = (conda_share / "tesseract" / "support") if conda_share else pkg_dir / "_no_dev"
    ws_resource = (conda_share / "tesseract") if conda_share else pkg_dir / "_no_dev"
    ws_composer = (
        (conda_share / "tesseract_planning" / "task_composer")
        if conda_share
        else pkg_dir / "_no_dev"
    )
    ws_config = ws_composer / "config" / "task_composer_plugins.yaml"

    # TESSERACT_SUPPORT_DIR: path to tesseract_support (bundled or dev)
    _set_env_if_missing("TESSERACT_SUPPORT_DIR", support_dir, ws_support)

    # TESSERACT_RESOURCE_PATH: directory containing `tesseract` as a subdir
    # so locator resolves package://tesseract/support/X via `<path>/tesseract/support/X`.
    # bundled:   data/tesseract/support       → use parent's parent (= data)
    # dev (conda): $CONDA_PREFIX/share/tesseract → use parent (= share)
    _set_env_if_missing("TESSERACT_RESOURCE_PATH", support_dir.parent, ws_resource, use_parent=True)

    # TESSERACT_TASK_COMPOSER_CONFIG_FILE: bundled config (wheel) or the conda share (editable)
    if "TESSERACT_TASK_COMPOSER_CONFIG_FILE" not in os.environ:
        for cfg in (config_dir / "task_composer_plugins.yaml", ws_config):
            if cfg.is_file():
                os.environ["TESSERACT_TASK_COMPOSER_CONFIG_FILE"] = str(cfg)
                break

    # TESSERACT_TASK_COMPOSER_DIR (needed by some code paths)
    _set_env_if_missing("TESSERACT_TASK_COMPOSER_DIR", config_dir, ws_composer)


_configured = False


def ensure_configured() -> None:
    """Configure environment on first use. Safe to call multiple times."""
    global _configured  # noqa: PLW0603
    if not _configured:
        _configure_environment()
        _configured = True


def get_data_path() -> Path:
    """Get path to bundled data directory."""
    return Path(__file__).parent / "data"


def get_tesseract_support_path() -> Path:
    """Get path to bundled tesseract_support directory."""
    return Path(__file__).parent / "data" / "tesseract" / "support"


class TaskComposerConfigNotFoundError(FileNotFoundError):
    """No task composer config found via env var, bundled data, or conda share."""


def get_task_composer_config_path() -> Path:
    """Get the resolved task composer plugin config file.

    Resolution is owned by `_configure_environment()` (env var → bundled data →
    conda share) and published via
    TESSERACT_TASK_COMPOSER_CONFIG_FILE; this returns that single source of
    truth as a `Path`.

    Raises:
        TaskComposerConfigNotFoundError: nothing resolves to an existing file.
    """
    ensure_configured()
    env_cfg = os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE")
    if env_cfg and Path(env_cfg).is_file():
        return Path(env_cfg)
    raise TaskComposerConfigNotFoundError(
        f"no task composer config found (TESSERACT_TASK_COMPOSER_CONFIG_FILE={env_cfg!r}); "
        "checked env var, bundled data/task_composer_config/, and $CONDA_PREFIX share"
    )


# Run at import of tesseract_robotics itself so env vars are set before any
# submodule is loaded. Submodules used to trigger ensure_configured() from
# their own __init__.py, but that missed direct imports of e.g.
# `tesseract_kinematics` which then couldn't find plugins.
ensure_configured()
