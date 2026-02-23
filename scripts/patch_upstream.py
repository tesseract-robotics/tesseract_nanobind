#!/usr/bin/env python3
"""Patch upstream C++ headers for missing includes.

Idempotent — safe to run multiple times. Detects already-patched files.

tesseract_planning 0.34.0 poly headers use std::runtime_error without
#include <stdexcept>. Compiles on macOS (transitive include from Clang
stdlib) but fails on Linux GCC.
"""

from pathlib import Path

PATCHES = [
    # (marker_include, missing_include, insert_after)
    ("<stdexcept>", "#include <stdexcept>", "#include <string>"),
]


def patch_file(path: Path, uses: str, marker: str, insert: str, after: str) -> bool:
    """Add missing include to a file if needed.

    Returns True if file was patched, False if skipped.
    """
    text = path.read_text()

    if marker in text:
        return False  # already patched

    if uses not in text:
        return False  # doesn't use the symbol

    if after not in text:
        print(f"  WARN: {path} — can't find '{after}' to insert after")
        return False

    patched = text.replace(after, f"{after}\n{insert}", 1)
    path.write_text(patched)
    return True


def main():
    ws = Path.cwd()
    poly_dir = ws / "tesseract_planning" / "tesseract_command_language" / "include" / "tesseract_command_language" / "poly"

    if not poly_dir.exists():
        print("  no poly headers found (workspace not yet populated?)")
        return

    headers = sorted(poly_dir.glob("*.h"))
    patched = 0
    skipped = 0

    for header in headers:
        for marker, insert, after in PATCHES:
            if patch_file(header, "std::runtime_error", marker, insert, after):
                print(f"  patched: {header.relative_to(ws)}")
                patched += 1
            else:
                skipped += 1

    if patched == 0:
        print("  all headers already patched (or no patches needed)")
    else:
        print(f"  {patched} patched, {skipped} skipped")


if __name__ == "__main__":
    main()
