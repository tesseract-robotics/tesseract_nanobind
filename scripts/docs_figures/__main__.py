"""Regenerate the docs figures, check them, or re-measure their timed data.

    pixi run docs-figures                 # every figure into docs/assets/figures, and the lock
    pixi run docs-figures frames          # one figure (its lock entry only)
    pixi run docs-figures --check         # regenerate into build/ and compare (see lock.compare)
    pixi run docs-figures --preview DIR   # also PNG previews on each theme's page colour
    pixi run docs-figures-measure NAME    # re-run a figure's benchmark into its data JSON

A figure module exposes `compute()` (its data, from the library or its committed data JSON),
`draw(data, theme)` and `inputs()` (the files besides code it reads). A measured figure also
exposes `measure()`, which writes its data JSON; rendering never measures.
"""

from __future__ import annotations

import argparse
import importlib
import shutil
import sys
from pathlib import Path
from types import ModuleType
from typing import Final

from scripts.docs_figures import lock
from scripts.docs_figures.style import FIGURES_DIR, REPO_ROOT, render

FIGURES: Final[tuple[str, ...]] = (
    "frames",
    "collision_distance",
    "continuous_collision",
    "time_parameterization",
    "raster_toolpath",
    "planner_comparison",
    "ik_branches",
)
CHECK_DIR: Final[Path] = REPO_ROOT / "build" / "docs-figures-check"


def _module(name: str) -> ModuleType:
    return importlib.import_module(f"scripts.docs_figures.{name}")


def regenerate(
    names: list[str], out_dir: Path, preview: Path | None
) -> dict[str, lock.FigureEntry]:
    """Render `names` into `out_dir`; return their lock entries (hashes as if in FIGURES_DIR)."""
    entries = {}
    for name in names:
        module = _module(name)
        data = module.compute()
        outputs = render(
            name,
            lambda theme, data=data, module=module: module.draw(data, theme),
            out_dir=out_dir,
            preview_dir=preview,
        )
        entries[name] = lock.FigureEntry(
            inputs=lock.input_hashes(name, module.inputs()),
            outputs=lock.output_hashes(outputs, out_dir),
        )
        for path in outputs:
            print(path.relative_to(REPO_ROOT))
    return entries


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("names", nargs="*", metavar="NAME", help=f"default: all of {FIGURES}")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true", help="regenerate into build/ and compare")
    mode.add_argument("--measure", action="store_true", help="re-run the named figures' benchmarks")
    parser.add_argument("--preview", type=Path, help="also write PNG previews into this directory")
    args = parser.parse_args()
    unknown = sorted(set(args.names) - set(FIGURES))
    if unknown:
        parser.error(f"unknown figure(s) {unknown}; known: {list(FIGURES)}")
    names = args.names or list(FIGURES)

    if args.measure:
        for name in names:
            module = _module(name)
            if not hasattr(module, "measure"):
                parser.error(f"{name} has no measured data")
            print(module.measure().relative_to(REPO_ROOT))
        return

    if args.check:
        shutil.rmtree(CHECK_DIR, ignore_errors=True)
        same_platform, problems = lock.compare(
            regenerate(names, CHECK_DIR, args.preview), CHECK_DIR
        )
        if problems:
            print("\n".join(problems), file=sys.stderr)
            raise lock.FiguresLockError(
                f"{len(problems)} difference(s); run `pixi run docs-figures` and commit"
            )
        if same_platform:
            print(f"{len(names)} figure(s) reproduce bit for bit")
        else:
            print(
                f"{len(names)} figure(s): inputs match the lock and render to well-formed SVG; "
                f"bytes not compared, this is not the render platform {lock.read_lock()['render_platform']}"
            )
        return

    lock.write_lock(regenerate(names, FIGURES_DIR, args.preview))


if __name__ == "__main__":
    main()
