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

3. tesseract_planning 0.34.0 poly .cpp files use std::as_const without
   #include <utility>. GCC 13+ libstdc++ tightened transitive includes so
   files with minimal includes (e.g. state_waypoint_poly.cpp) no longer get
   <utility> pulled in for free.

4. Some upstream CMake macros bake `-mno-avx` directly into a multi-line
   set(<COMPILE_OPTIONS> ...) list (e.g. descartes_light core-macros). The
   flag is x86-only — aarch64/armv7l GCC errors out with "unrecognized
   command-line option '-mno-avx'". opw_kinematics, tesseract_common, and
   trajopt_common already guard the flag via execute_process(uname -p) +
   if(NOT CMAKE_SYSTEM_NAME2 MATCHES "aarch64" AND NOT ... "armv7l" AND NOT
   ... "unknown"). Apply the same guard wherever it's missing.
"""

from __future__ import annotations

import re
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


def patch_utility_includes(ws: Path) -> tuple[int, int]:
    """Add #include <utility> to tesseract_planning poly .cpp files that use std::as_const.

    Returns (patched_count, skipped_count).
    """
    poly_dir = ws / "tesseract_planning" / "tesseract_command_language" / "src" / "poly"

    if not poly_dir.exists():
        print("  no poly sources found (workspace not yet populated?)")
        return 0, 0

    marker = "<utility>"
    insert = "#include <utility>"
    uses = "std::as_const"

    patched = 0
    skipped = 0
    for source in sorted(poly_dir.glob("*.cpp")):
        text = source.read_text()
        if marker in text or uses not in text:
            skipped += 1
            continue

        # Insert after the last #include at the top of the file. Find the first
        # #include, then scan forward to the last contiguous #include block.
        lines = text.splitlines(keepends=True)
        first_include = next((i for i, line in enumerate(lines) if line.startswith("#include")), None)
        if first_include is None:
            print(f"  WARN: {source.relative_to(ws)} — no #include found to anchor insertion")
            skipped += 1
            continue

        last_include = first_include
        for i in range(first_include + 1, len(lines)):
            stripped = lines[i].strip()
            if stripped.startswith("#include"):
                last_include = i
            elif stripped == "" or stripped.startswith("//"):
                continue
            else:
                break

        lines.insert(last_include + 1, f"{insert}\n")
        source.write_text("".join(lines))
        print(f"  patched: {source.relative_to(ws)}")
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


_MNO_AVX_GUARD_MARKER = "# tesseract_nanobind: guard -mno-avx (x86-only) for aarch64/armv7l"


def _find_enclosing_set(lines: list[str], idx: int) -> tuple[str, int, int] | None:
    """If lines[idx] is a continuation `-mno-avx` inside a multi-line set(VAR ...),
    return (var_name, set_open_idx, set_close_idx). Else None.

    Detects "continuation" by paren balance: walk backwards counting `)` minus `(`,
    and the line where balance first goes negative is the enclosing `set(` opener.
    """
    if lines[idx].strip() != "-mno-avx":
        return None

    balance = lines[idx].count(")") - lines[idx].count("(")
    for j in range(idx - 1, -1, -1):
        balance += lines[j].count(")") - lines[j].count("(")
        if balance < 0:
            m = re.match(r"^\s*set\(\s*([A-Za-z_][A-Za-z0-9_]*)\b", lines[j])
            if not m:
                return None
            var_name = m.group(1)
            fwd = lines[j].count("(") - lines[j].count(")")
            for k in range(j + 1, len(lines)):
                fwd += lines[k].count("(") - lines[k].count(")")
                if fwd == 0:
                    return var_name, j, k
            return None
    return None


def _build_mno_avx_guard(indent: str, var_name: str) -> str:
    """The opw_kinematics_macros.cmake guard, parameterized for any compile-options var."""
    return (
        f'{indent}{_MNO_AVX_GUARD_MARKER}\n'
        f'{indent}execute_process(COMMAND uname -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)\n'
        f'{indent}if(NOT CMAKE_SYSTEM_NAME2 MATCHES "aarch64"\n'
        f'{indent}   AND NOT CMAKE_SYSTEM_NAME2 MATCHES "armv7l"\n'
        f'{indent}   AND NOT CMAKE_SYSTEM_NAME2 MATCHES "unknown")\n'
        f'{indent}  list(APPEND {var_name} -mno-avx)\n'
        f'{indent}endif()\n'
    )


def patch_mno_avx_guards(ws: Path) -> tuple[int, int]:
    """Wrap unguarded `-mno-avx` in the (NOT aarch64/armv7l/unknown) guard.

    Scans every CMakeLists.txt and *.cmake in the workspace, finds `-mno-avx`
    inlined into a multi-line set(VAR ...) list, removes it, and appends a
    guarded list(APPEND VAR -mno-avx) right after the set's closing paren.
    Single-line set(VAR -mno-avx) usages (the opw/tesseract/trajopt pattern,
    already inside an if-NOT block) are skipped automatically because they
    don't match the "continuation line" detector.

    Returns (files_patched, files_skipped_or_clean).
    """
    targets = sorted(set(list(ws.rglob("*.cmake")) + list(ws.rglob("CMakeLists.txt"))))

    patched = 0
    skipped = 0
    for path in targets:
        # Skip build artifacts if any happen to be under the workspace tree
        if any(part in ("build", ".git") for part in path.parts):
            continue
        try:
            text = path.read_text()
        except (UnicodeDecodeError, OSError):
            continue
        if "-mno-avx" not in text:
            continue
        if _MNO_AVX_GUARD_MARKER in text:
            skipped += 1
            continue

        lines = text.splitlines(keepends=True)
        sites = []
        for i in range(len(lines)):
            info = _find_enclosing_set(lines, i)
            if info is not None:
                sites.append((i, info))

        if not sites:
            # File contains -mno-avx but only in already-guarded single-line form.
            skipped += 1
            continue

        lines_to_remove = {i for i, _ in sites}
        inserts_after: dict[int, str] = {}
        for _, (var_name, open_idx, close_idx) in sites:
            indent = re.match(r"^(\s*)", lines[open_idx]).group(1)
            # Multiple sites in the same file may share a close_idx in pathological
            # cases — collapse to a single guard insertion.
            inserts_after.setdefault(close_idx, _build_mno_avx_guard(indent, var_name))

        out: list[str] = []
        for i, line in enumerate(lines):
            if i in lines_to_remove:
                continue
            out.append(line)
            if i in inserts_after:
                out.append(inserts_after[i])

        path.write_text("".join(out))
        print(f"  patched: {path.relative_to(ws)} ({len(sites)} site{'s' if len(sites) != 1 else ''})")
        patched += 1

    return patched, skipped


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

    print("Patch 3: tesseract_planning poly sources (<utility>)")
    patched, skipped = patch_utility_includes(ws)
    if patched == 0:
        print("  all sources already patched (or no patches needed)")
    else:
        print(f"  {patched} patched, {skipped} skipped")

    print("Patch 4: guard unguarded -mno-avx for aarch64/armv7l")
    patched, skipped = patch_mno_avx_guards(ws)
    if patched == 0:
        print("  all CMake files already guarded (or no -mno-avx found)")
    else:
        print(f"  {patched} patched, {skipped} already-guarded/clean")


if __name__ == "__main__":
    main()
