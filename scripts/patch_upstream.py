#!/usr/bin/env python3
"""Patch upstream C++ sources for cross-platform compatibility.

Idempotent — safe to run multiple times. Detects already-patched files.

Patches applied:

1. tesseract_planning 0.34.0 poly headers use std::runtime_error without
   #include <stdexcept>. Compiles on macOS (transitive include from Clang
   stdlib) but fails on Linux GCC.

2. trajopt osqp_eigen ExternalProject_Add doesn't forward OSQP_IS_V1_FINAL
   to the nested cmake. On fresh macOS arm64 builds, osqp_eigen's try_compile
   for OSQPCscMatrix_set_data fails (reason not fully isolated — include path
   not propagating through the nested cmake), so it falls back to csc_set_data
   which doesn't exist in osqp v1.0.0 final. We know we're building osqp v1.0.0
   final (pinned in trajopt_ext/osqp), so force OSQP_IS_V1_FINAL=ON directly.
"""

from pathlib import Path


def patch_stdexcept_includes(ws: Path) -> tuple[int, int]:
    """Add #include <stdexcept> to tesseract_planning poly headers that use std::runtime_error.

    Returns (patched_count, skipped_count).
    """
    poly_dir = (
        ws
        / "tesseract_planning"
        / "tesseract_command_language"
        / "include"
        / "tesseract_command_language"
        / "poly"
    )

    if not poly_dir.exists():
        print("  no poly headers found (workspace not yet populated?)")
        return 0, 0

    marker = "<stdexcept>"
    insert = "#include <stdexcept>"
    after = "#include <string>"
    uses = "std::runtime_error"

    patched = 0
    skipped = 0
    for header in sorted(poly_dir.glob("*.h")):
        text = header.read_text()
        if marker in text or uses not in text:
            skipped += 1
            continue
        if after not in text:
            print(f"  WARN: {header.relative_to(ws)} — can't find '{after}' to insert after")
            skipped += 1
            continue
        header.write_text(text.replace(after, f"{after}\n{insert}", 1))
        print(f"  patched: {header.relative_to(ws)}")
        patched += 1

    return patched, skipped


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
