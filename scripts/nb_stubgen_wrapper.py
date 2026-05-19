"""Run nanobind.stubgen with Windows DLL search dirs added.

CMake's `nanobind_add_stub` runs `python -m nanobind.stubgen` to introspect
binding modules at build time. On Windows this imports the freshly-built
`.pyd`, which in turn loads its transitive DLL deps (tesseract C++ libs in
`ws/install/bin`, conda libs in `Library/bin`). Python 3.8+ doesn't use
PATH for those transitive loads — it requires `os.add_dll_directory()`.

This wrapper reads search dirs from `NB_STUB_DLL_DIRS` (os.pathsep-separated),
registers them via `os.add_dll_directory()`, then dispatches to the same
`nanobind.stubgen` CLI. CMake invokes it via:

    cmake -E env NB_STUB_DLL_DIRS=... python nb_stubgen_wrapper.py <args>

On non-Windows platforms the env-var loop is a no-op, so the same command
works everywhere.
"""

from __future__ import annotations

import os
import runpy
import sys


def _add_dll_search_dirs() -> None:
    if sys.platform != "win32":
        return
    # Comma separator (not os.pathsep / `;`) because `;` collides with CMake's
    # list separator when the env var is passed via `cmake -E env`.
    dirs = os.environ.get("NB_STUB_DLL_DIRS", "")
    for directory in dirs.split(","):
        directory = directory.strip()
        if directory and os.path.isdir(directory):
            os.add_dll_directory(directory)  # type: ignore[attr-defined]


def main() -> None:
    _add_dll_search_dirs()
    # Re-dispatch to nanobind.stubgen with the same argv (sys.argv[0] gets
    # rewritten to nanobind.stubgen so its argparse displays the right prog).
    runpy.run_module("nanobind.stubgen", run_name="__main__", alter_sys=True)


if __name__ == "__main__":
    main()
