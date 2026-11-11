"""Where the native stack the bindings link against lives, and which of its dirs are its own."""

from __future__ import annotations

import os
from pathlib import Path

import native_stack
from audit_ownership import REPO_ROOT

# The native stack the wheels link against (scripts/native_stack.py): `ws/install` holds
# tesseract, trajopt, tesseract_planning; `vcpkg/installed/<triplet>` the third-party libraries.
# CI points this at the restored native install; locally it is `pixi run native-build`'s output.
NATIVE_ROOT_ENV = "AUDIT_NATIVE_ROOT"


class IncludeDirError(FileNotFoundError):
    """A versioned include dir of the binding build is missing or ambiguous."""


class NativeStackMissingError(FileNotFoundError):
    """The native stack the bindings build against is absent or ambiguous under its root."""


def ompl_include(include: Path) -> Path:
    """The include dir that holds `ompl/`: `include/ompl-<version>` or `include` itself.

    Raises:
        IncludeDirError: no OMPL headers, or more than one versioned dir.
    """
    found = sorted(d for d in include.glob("ompl-*") if d.is_dir())
    if len(found) > 1:
        raise IncludeDirError(f"expected one {include / 'ompl'}-<version> dir, found {len(found)}")
    if found:
        return found[0]
    if (include / "ompl").is_dir():
        return include
    raise IncludeDirError(f"no OMPL headers under {include}: no ompl/ and no ompl-<version>/")


def native_root() -> Path:
    """The native stack root: `$AUDIT_NATIVE_ROOT`, else the one dev build under `.native/`.

    Raises:
        NativeStackMissingError: no variable and not exactly one `.native/<triplet>` dir.
    """
    explicit = os.environ.get(NATIVE_ROOT_ENV)
    if explicit:
        return Path(explicit)
    builds = sorted(d for d in (REPO_ROOT / ".native").glob("*") if d.is_dir())
    if len(builds) != 1:
        raise NativeStackMissingError(
            f"set {NATIVE_ROOT_ENV}, or build the native stack (`pixi run native-build`) so that "
            f"exactly one .native/<triplet> exists; found {[d.name for d in builds]}"
        )
    return builds[0]


def native_include_roots(root: Path) -> tuple[Path, Path]:
    """The two include roots of a native stack: the colcon install and vcpkg's installed prefix.

    The vcpkg prefix is the triplet the stack's vcpkg stage recorded in its identity file
    (`scripts/native_stack.py`); `vcpkg/installed` also holds the host triplet's build tools.

    Raises:
        NativeStackMissingError: either root is missing, or vcpkg has no recorded triplet.
    """
    install = root / "ws" / "install" / "include"
    installed = root / "vcpkg" / "installed"
    identity = native_stack.read_identity(installed)
    if identity is None:
        raise NativeStackMissingError(
            f"no {native_stack.IDENTITY_FILE} under {installed}; build the native stack "
            f"(`pixi run native-build`) or set {NATIVE_ROOT_ENV} to its root"
        )
    vcpkg = installed / identity["triplet"] / "include"
    for include in (install, vcpkg):
        if not include.is_dir():
            raise NativeStackMissingError(f"{include} does not exist (native stack at {root})")
    return install, vcpkg


def native_stack_dirs(root: Path) -> tuple[Path, Path]:
    """The stack's own directories under `root`: the colcon workspace and vcpkg.

    `root` itself can hold more: in CI it is the workspace, which also holds the repo checkout.
    """
    return root / "ws", root / "vcpkg"


NATIVE_ROOT = native_root()
NATIVE_INCLUDES = native_include_roots(NATIVE_ROOT)
NATIVE_DIRS = native_stack_dirs(NATIVE_ROOT)
