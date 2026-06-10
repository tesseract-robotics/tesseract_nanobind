"""
tesseract_robotics - Python bindings for Tesseract motion planning.

Environment Variables (auto-configured on first use):
    TESSERACT_SUPPORT_DIR: Path to tesseract_support (URDF/meshes)
    TESSERACT_RESOURCE_PATH: Resource search path for URDFs
    TESSERACT_TASK_COMPOSER_CONFIG_FILE: Task composer YAML config
    TESSERACT_TASK_COMPOSER_DIR: Task composer config directory
    TESSERACT_PLUGIN_PATH: Override plugin search path
    TESSERACT_CONTACT_MANAGERS_PLUGIN_DIRECTORIES: Collision plugin path
    TESSERACT_KINEMATICS_PLUGIN_DIRECTORIES: Kinematics plugin path
    TESSERACT_TASK_COMPOSER_PLUGIN_DIRECTORIES: Composer plugin path

Priority: Bundled data (installed) > Dev workspace (editable) > User env vars
"""

from __future__ import annotations

import ctypes
import hashlib
import json
import os
import sys
import tempfile
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

from loguru import logger

# Linux: preload the bundled libstdc++ BEFORE anything can pull in the system one.
# (Everything imported above this point is stdlib or pure python — no libstdc++.)
#
# The wheels bundle libstdc++.so.6 (the prebuilt tesseract conda libs are built with
# gcc 13/14 and need GLIBCXX > what e.g. ubuntu 22.04 ships), but numpy/scipy link
# the SYSTEM libstdc++ (manylinux-whitelisted, never vendored). ld.so dedupes by
# SONAME: whichever copy enters the link map first serves every later request. If
# numpy loads first on an old distro, our libs bind to the too-old system copy and
# die with "GLIBCXX_3.4.31 not found" (GH #35). Loading our newer copy RTLD_GLOBAL
# here wins the race for any process that imports tesseract_robotics before numpy —
# libstdc++ is forward-compatible, so numpy/scipy are happy binding to it. No-op for
# editable installs (no bundled copy) and for processes where numpy already won
# (unchanged from today's behavior).
#
# Conda envs are excluded: there the env's own libstdc++ is canonical and already
# new enough (the C++ stack comes from the same solve), and force-loading a second
# copy RTLD_GLOBAL alongside it segfaulted the wheel-in-pixi-env ABI canary on
# aarch64. The rescue is only for non-conda hosts (bare venvs, system python, Rhino).
if sys.platform == "linux" and "CONDA_PREFIX" not in os.environ:
    _bundled_libstdcxx = Path(__file__).parent / "libstdc++.so.6"
    if _bundled_libstdcxx.is_file():
        ctypes.CDLL(str(_bundled_libstdcxx), mode=ctypes.RTLD_GLOBAL)

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


def _is_editable_install() -> bool:
    """Check if package was installed in editable mode (pip install -e .)."""
    try:
        from importlib.metadata import distribution

        dist = distribution("tesseract-robotics-nanobind")
        # Check for direct_url.json which indicates editable install
        direct_url = dist.read_text("direct_url.json")
        if direct_url:
            data = json.loads(direct_url)
            return data.get("dir_info", {}).get("editable", False)
    except (FileNotFoundError, KeyError, TypeError, json.JSONDecodeError) as e:
        logger.debug(f"Editable install check failed: {type(e).__name__}: {e}")
    # Fallback: check if __file__ is outside site-packages
    pkg_path = Path(__file__).parent
    return "site-packages" not in str(pkg_path)


def _set_env_if_missing(var_name: str, *candidates: Path, use_parent: bool = False) -> None:
    """Set env var to first existing path if not already set."""
    if var_name in os.environ:
        return
    for path in candidates:
        if path.is_dir():
            os.environ[var_name] = str(path.parent if use_parent else path)
            return


def _resolve_config_paths(config_path: Path, plugin_path: str | None) -> Path:
    """
    Resolve plugin path placeholders in task composer YAML configs.

    Handles both:
    - @PLUGIN_PATH@ placeholder (bundled wheels)
    - /usr/local/lib (the conda task_composer config's default search path)
    """
    if plugin_path is None:
        return config_path

    content = config_path.read_text()

    # Check if config needs patching
    needs_patch = "@PLUGIN_PATH@" in content or "/usr/local/lib" in content
    if not needs_patch:
        return config_path

    # Generate resolved config in system temp dir (not inside package dir)
    cache_dir = Path(tempfile.gettempdir()) / "tesseract_robotics"
    cache_dir.mkdir(exist_ok=True)

    # Use hash of plugin_path to allow different installs
    path_hash = hashlib.md5(plugin_path.encode()).hexdigest()[:8]
    resolved_path = cache_dir / f"{config_path.stem}_{path_hash}.yaml"

    # YAML double-quoted strings treat backslashes as escapes, so Windows paths
    # must be written with forward slashes.
    yaml_plugin_path = plugin_path.replace("\\", "/")

    # Only regenerate if source changed or cache missing
    if not resolved_path.exists() or resolved_path.stat().st_mtime < config_path.stat().st_mtime:
        resolved_content = content.replace("@PLUGIN_PATH@", yaml_plugin_path)
        resolved_content = resolved_content.replace("/usr/local/lib", yaml_plugin_path)
        resolved_path.write_text(resolved_content)

    return resolved_path


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

    # Plugin search paths - env var, bundled plugins, or $CONDA_PREFIX/lib (editable)
    # Linux:   pkg_dir (all deps bundled in package root with $ORIGIN rpath)
    # macOS:   .dylibs (delocate-repaired)
    # Windows: pkg_dir (plugin factory DLLs bundled there; delvewheel libs go to
    #          sibling tesseract_robotics.libs/ but plugin dlopen targets pkg_dir)
    plugin_path = os.environ.get("TESSERACT_PLUGIN_PATH")  # explicit override
    editable = _is_editable_install()

    if not plugin_path:
        bundled_plugin = pkg_dir / "libtesseract_collision_bullet_factories.so"
        dylibs_dir = pkg_dir / ".dylibs"
        bundled_plugin_win = pkg_dir / "tesseract_collision_bullet_factories.dll"
        # editable install: plugin factory libs live alongside the conda libs
        # ($CONDA_PREFIX/lib, or Library/bin for the DLLs on conda Windows).
        conda_lib = None
        if conda_prefix:
            for cand in (Path(conda_prefix) / "lib", Path(conda_prefix) / "Library" / "bin"):
                if cand.is_dir():
                    conda_lib = cand
                    break

        if bundled_plugin.exists():
            plugin_path = str(pkg_dir)
        elif dylibs_dir.is_dir():
            plugin_path = str(dylibs_dir)
        elif bundled_plugin_win.exists():
            plugin_path = str(pkg_dir)
        elif editable and conda_lib is not None:
            plugin_path = str(conda_lib)

    # TESSERACT_TASK_COMPOSER_CONFIG_FILE
    # Check if patching is needed (for bundled wheels or if env.sh set a path)
    env_cfg = os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE")
    if env_cfg and plugin_path:
        env_cfg_path = Path(env_cfg)
        if env_cfg_path.is_file():
            cfg_resolved = _resolve_config_paths(env_cfg_path, plugin_path)
            os.environ["TESSERACT_TASK_COMPOSER_CONFIG_FILE"] = str(cfg_resolved)
    elif "TESSERACT_TASK_COMPOSER_CONFIG_FILE" not in os.environ:
        cfg = config_dir / "task_composer_plugins.yaml"
        if cfg.is_file():
            cfg_resolved = _resolve_config_paths(cfg, plugin_path) if plugin_path else cfg
            os.environ["TESSERACT_TASK_COMPOSER_CONFIG_FILE"] = str(cfg_resolved)
        elif ws_config.is_file():
            cfg_resolved = (
                _resolve_config_paths(ws_config, plugin_path) if plugin_path else ws_config
            )
            os.environ["TESSERACT_TASK_COMPOSER_CONFIG_FILE"] = str(cfg_resolved)

    # TESSERACT_TASK_COMPOSER_DIR (needed by some code paths)
    _set_env_if_missing("TESSERACT_TASK_COMPOSER_DIR", config_dir, ws_composer)

    # Set plugin directories for all plugin factories (if not overridden)
    if plugin_path:
        for env_var in [
            "TESSERACT_CONTACT_MANAGERS_PLUGIN_DIRECTORIES",
            "TESSERACT_KINEMATICS_PLUGIN_DIRECTORIES",
            "TESSERACT_TASK_COMPOSER_PLUGIN_DIRECTORIES",
        ]:
            if env_var not in os.environ:
                os.environ[env_var] = plugin_path


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
    conda share, with plugin-path placeholders patched) and published via
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
