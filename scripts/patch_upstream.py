#!/usr/bin/env python3
"""Patch upstream C++ sources for cross-platform compatibility.

Idempotent — safe to run multiple times. Detects already-patched files.

Patches applied:

1. Upstream sources use std::runtime_error (and friends — out_of_range,
   invalid_argument, etc.) without #include <stdexcept>. Compiles on macOS
   (transitive include from Clang stdlib) but fails on Linux GCC. The 0.35
   consolidated tesseract repo spreads these across many packages (common,
   collision, kinematics, srdf, ...), so we scan the whole workspace tree
   rather than a single hardcoded directory.

2. trajopt osqp_eigen ExternalProject_Add doesn't forward OSQP_IS_V1_FINAL
   to the nested cmake. On fresh macOS arm64 builds, osqp_eigen's try_compile
   for OSQPCscMatrix_set_data fails (reason not fully isolated — include path
   not propagating through the nested cmake), so it falls back to csc_set_data
   which doesn't exist in osqp v1.0.0 final. We know we're building osqp v1.0.0
   final (pinned in trajopt_ext/osqp), so force OSQP_IS_V1_FINAL=ON directly.
"""

import re
from pathlib import Path
from typing import Optional

# Directories that are never upstream source (build outputs, vcs, installs).
_SKIP_DIRS = {"build", "install", "log", ".git"}

_SOURCE_SUFFIXES = {".h", ".hpp", ".cpp", ".cc"}

# std types that live in <stdexcept>.
_STDEXCEPT_TYPES = (
    "runtime_error",
    "logic_error",
    "out_of_range",
    "invalid_argument",
    "domain_error",
    "length_error",
    "range_error",
    "overflow_error",
    "underflow_error",
)
_STDEXCEPT_RE = re.compile(r"\bstd::(?:" + "|".join(_STDEXCEPT_TYPES) + r")\b")


def _iter_sources(ws: Path):
    """Yield C/C++ source and header files under ws, skipping build/vcs/install dirs."""
    for path in sorted(ws.rglob("*")):
        if path.suffix not in _SOURCE_SUFFIXES:
            continue
        if any(part in _SKIP_DIRS for part in path.parts):
            continue
        yield path


def _insert_include_after_block(text: str, include_line: str) -> Optional[str]:
    """Insert include_line after the last contiguous top-of-file #include block.

    Returns the new text, or None if the file has no #include to anchor against.
    """
    lines = text.splitlines(keepends=True)
    first_include = next((i for i, line in enumerate(lines) if line.lstrip().startswith("#include")), None)
    if first_include is None:
        return None

    last_include = first_include
    for i in range(first_include + 1, len(lines)):
        stripped = lines[i].strip()
        if stripped.startswith("#include"):
            last_include = i
        elif stripped == "" or stripped.startswith("//"):
            continue
        else:
            break

    lines.insert(last_include + 1, f"{include_line}\n")
    return "".join(lines)


def _patch_missing_include(ws: Path, uses_re: re.Pattern, marker: str, insert: str) -> tuple[int, int]:
    """Add `insert` to any source file matching `uses_re` but missing `marker`.

    Returns (patched_count, anchor_failures).
    """
    patched = 0
    skipped = 0
    for path in _iter_sources(ws):
        try:
            text = path.read_text()
        except (UnicodeDecodeError, OSError):
            continue
        if marker in text or not uses_re.search(text):
            continue
        new_text = _insert_include_after_block(text, insert)
        if new_text is None:
            print(f"  WARN: {path.relative_to(ws)} — no #include found to anchor {insert}")
            skipped += 1
            continue
        path.write_text(new_text)
        print(f"  patched: {path.relative_to(ws)}")
        patched += 1

    return patched, skipped


def patch_stdexcept_includes(ws: Path) -> tuple[int, int]:
    """Add #include <stdexcept> to files using std::runtime_error (and friends).

    Returns (patched_count, anchor_failures).
    """
    return _patch_missing_include(ws, _STDEXCEPT_RE, "<stdexcept>", "#include <stdexcept>")


def patch_osqp_eigen_final_flag(ws: Path) -> bool:
    """Force OSQP_IS_V1_FINAL=ON in trajopt's osqp_eigen ExternalProject_Add.

    Returns True if patched, False if skipped (already patched or file missing).
    """
    cmakelists = ws / "trajopt" / "trajopt_ext" / "osqp_eigen" / "CMakeLists.txt"

    if not cmakelists.exists():
        print(
            f"  skipped: {cmakelists.relative_to(ws) if cmakelists.is_relative_to(ws) else cmakelists} not found"
        )
        return False

    text = cmakelists.read_text()

    marker = "OSQP_IS_V1_FINAL:BOOL=ON"
    if marker in text:
        return False  # already patched

    target = "-DOSQP_EIGEN_DEBUG_OUTPUT=OFF)"
    replacement = "-DOSQP_EIGEN_DEBUG_OUTPUT=OFF\n      -DOSQP_IS_V1_FINAL:BOOL=ON)"

    if target not in text:
        print(f"  WARN: {cmakelists.relative_to(ws)} — can't find target line to patch")
        return False

    cmakelists.write_text(text.replace(target, replacement, 1))
    print(f"  patched: {cmakelists.relative_to(ws)} (forced OSQP_IS_V1_FINAL=ON)")
    return True


def main():
    ws = Path.cwd()

    print("Patch 1: tesseract_planning poly headers (<stdexcept>)")
    patched, skipped = patch_stdexcept_includes(ws)
    if patched == 0:
        print("  all headers already patched (or no patches needed)")
    else:
        print(f"  {patched} patched, {skipped} skipped")

    print("Patch 2: trajopt osqp_eigen (force OSQP_IS_V1_FINAL=ON)")
    if not patch_osqp_eigen_final_flag(ws):
        print("  already patched (or file missing)")


if __name__ == "__main__":
    main()
