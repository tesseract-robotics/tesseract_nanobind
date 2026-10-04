"""Audit each binding's C++ API against its committed Python stub.

For each extension module, libclang parses `src/<module>/<module>_bindings.cpp`
(exactly the headers and macros the binding sees) and `ast` parses the
committed `_<module>.pyi` and the package `__init__.py`. No extension module is
imported. Gaps (C++ without Python), deviations (Python without C++) and
accepted deviations go to `docs/developer/binding-api-audit.md`.

Usage:
    pixi run -e audit audit-bindings [MODULE ...] [--json PATH]
"""

from __future__ import annotations

import os
import sysconfig
from collections.abc import Sequence
from pathlib import Path

import clang.cindex as ci
import nanobind

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
STUB_ROOT = SRC / "tesseract_robotics"

CONDA_PREFIX = Path(os.environ["CONDA_PREFIX"])
CONDA_INCLUDE = CONDA_PREFIX / "include"
# Major version of the `clang-22` package in the pixi `audit` feature (pyproject.toml).
LIBCLANG_MAJOR = 22
# Builtin headers (stdarg.h, stddef.h) shipped by `clang-22`; libclang does not find
# them on its own when loaded from the conda prefix.
CLANG_RESOURCE_DIR = CONDA_PREFIX / "lib" / "clang" / str(LIBCLANG_MAJOR)
# Matches CMakeLists.txt `set(CMAKE_CXX_STANDARD 17)`.
CXX_STD = "c++17"
# Header search path of the binding build: tesseract, yaml-cpp and console_bridge
# from the conda prefix; Eigen under include/eigen3; tesseract_nb.h from src/;
# nanobind and Python headers from their packages.
INCLUDE_DIRS = (
    CONDA_INCLUDE,
    CONDA_INCLUDE / "eigen3",
    SRC,
    Path(nanobind.include_dir()),
    Path(sysconfig.get_paths()["include"]),
)
PARSE_ARGS = (
    "-x",
    "c++",
    f"-std={CXX_STD}",
    "-fsyntax-only",
    "-resource-dir",
    str(CLANG_RESOURCE_DIR),
)
# Declarations only: binding bodies (the NB_MODULE block) carry no API.
PARSE_OPTIONS = ci.TranslationUnit.PARSE_SKIP_FUNCTION_BODIES
# The first 20 diagnostics locate a missing -I or define; the count reports the rest.
MAX_REPORTED_DIAGNOSTICS = 20


class HeaderParseError(RuntimeError):
    """libclang reported an error; a partial AST would silently under-report."""


def parse_tu(cpp: Path, extra_include_dirs: Sequence[Path] = ()) -> ci.TranslationUnit:
    """Parse one binding translation unit.

    Args:
        cpp: The binding `.cpp`.
        extra_include_dirs: Searched before `INCLUDE_DIRS` (test fixtures).

    Returns:
        The translation unit, free of error diagnostics.

    Raises:
        HeaderParseError: any diagnostic of severity >= error.
    """
    args = [*PARSE_ARGS, *(f"-I{d}" for d in (*extra_include_dirs, *INCLUDE_DIRS))]
    tu = ci.Index.create().parse(str(cpp), args=args, options=PARSE_OPTIONS)
    errors = [d for d in tu.diagnostics if d.severity >= ci.Diagnostic.Error]
    if errors:
        shown = "\n".join(str(d) for d in errors[:MAX_REPORTED_DIAGNOSTICS])
        raise HeaderParseError(f"{cpp}: {len(errors)} error diagnostics\n{shown}")
    return tu
