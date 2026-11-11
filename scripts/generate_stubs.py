"""Generate (or check) the committed .pyi stubs of every nanobind extension module.

The module list is discovered from the installed package, so a new extension
module cannot be left without a stub. Each stub is rendered by
`nanobind.stubgen` in a fresh interpreter: which cross-module types resolve
depends on what is imported, and a shared process would make one module's
stub depend on the modules rendered before it. Its id parameters are then
widened to the implicit `str` / `tuple` conversions (`widen_implicit_id_stubs.py`),
which stubgen does not record, cross references to a wrapped package's private
extension (`tesseract_robotics.common._common.Isometry3d`) are rewritten to the
public path (`tesseract_robotics.common.Isometry3d`), and nanobind's
`scipy.sparse.csr_matrix[float]` loses its subscript (not a valid type expression).

Usage:
    pixi run stubs         # rewrite the committed stubs
    pixi run stubs-check   # exit 1 if any committed stub differs from the build
"""

from __future__ import annotations

import argparse
import difflib
import importlib.machinery
import importlib.util
import os
import pkgutil
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from widen_implicit_id_stubs import widen

import tesseract_robotics

# Modules are discovered in the installed package (editable or wheel: an editable install
# splits the package between the source tree and site-packages, so every path entry is
# walked); stubs are compared against / written to the repo checkout.
PACKAGE_PATHS = list(tesseract_robotics.__path__)
REPO_ROOT = Path(__file__).resolve().parent.parent
STUB_ROOT = REPO_ROOT / "src" / "tesseract_robotics"

# Runs in the child interpreter: render one module's stub to stdout.
_RENDER = """
import importlib, sys
from nanobind.stubgen import StubGen
mod = importlib.import_module(sys.argv[1])
sg = StubGen(module=mod, quiet=True)
sg.put(mod)
sys.stdout.write(sg.get())
"""


# A wrapped package `tesseract_robotics.<pkg>` holds its extension as `_<pkg>` and star-imports
# it in `__init__.py`, so `tesseract_robotics.<pkg>.X` and `tesseract_robotics.<pkg>._<pkg>.X`
# name the same object. nanobind records the defining (private) module; the stubs show the
# public one.
_PRIVATE_EXTENSION_PATH = re.compile(r"\btesseract_robotics\.(\w+)\._\1\b")


def publish_paths(stub: str) -> str:
    """Rewrite references to a wrapped package's private extension module to the package."""
    return _PRIVATE_EXTENSION_PATH.sub(r"tesseract_robotics.\1", stub)


# nanobind's Eigen sparse caster names its Python type `scipy.sparse.csr_matrix[<scalar>]`
# (csc_matrix for column-major). scipy's csr_matrix is not a Generic class to a type checker
# (and scipy-stubs bounds the parameter to numpy scalar types, not `float`), so the subscript
# is an invalid type expression; the bare class is the exact static type.
_SPARSE_SUBSCRIPT = re.compile(r"\b(scipy\.sparse\.cs[rc]_matrix)\[[^\]]*\]")


def unparameterize_sparse(stub: str) -> str:
    """Drop the scalar subscript from `scipy.sparse.csr_matrix[...]` / `csc_matrix[...]`."""
    return _SPARSE_SUBSCRIPT.sub(r"\1", stub)


class StubRenderError(RuntimeError):
    """nanobind.stubgen failed for a module."""


def discover_modules() -> list[str]:
    """Return the dotted names of all compiled extension modules, sorted."""
    names = []
    for info in pkgutil.walk_packages(PACKAGE_PATHS, "tesseract_robotics."):
        if info.ispkg:
            continue
        spec = importlib.util.find_spec(info.name)
        if spec is not None and isinstance(spec.loader, importlib.machinery.ExtensionFileLoader):
            names.append(info.name)
    return sorted(names)


def stub_path(module: str) -> Path:
    """Committed stub location, beside the module.

    `tesseract_robotics.a` → `src/tesseract_robotics/a.pyi`;
    `tesseract_robotics.b._b` → `src/tesseract_robotics/b/_b.pyi`.
    """
    parts = module.split(".")[1:]
    return STUB_ROOT.joinpath(*parts).with_suffix(".pyi")


def render(module: str) -> str:
    """Render the stub of `module` in a fresh interpreter, id parameters widened.

    Raises:
        StubRenderError: stubgen exited non-zero.
    """
    # UTF-8 both ends: Windows would otherwise encode the child's stdout as cp1252.
    env = {**os.environ, "TRAJOPT_LOG_THRESH": "ERROR", "PYTHONIOENCODING": "utf-8"}
    proc = subprocess.run(
        [sys.executable, "-c", _RENDER, module],
        capture_output=True,
        text=True,
        encoding="utf-8",
        env=env,
    )
    if proc.returncode != 0:
        raise StubRenderError(f"stubgen failed for {module}:\n{proc.stderr}")
    return unparameterize_sparse(publish_paths(widen(proc.stdout)))


def render_all(modules: list[str]) -> dict[str, str]:
    """Render all stubs concurrently (each in its own process)."""
    with ThreadPoolExecutor() as pool:
        return dict(zip(modules, pool.map(render, modules)))


def drift_report(path: Path, committed: str, rendered: str) -> str:
    """Unified diff of a stale stub; in CI the log is the only place a platform leak shows."""
    return "".join(
        difflib.unified_diff(
            committed.splitlines(keepends=True),
            rendered.splitlines(keepends=True),
            fromfile=f"committed/{path.as_posix()}",
            tofile=f"rendered/{path.as_posix()}",
        )
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="report drift instead of writing")
    args = parser.parse_args(argv)

    rendered = render_all(discover_modules())
    stale = []
    for module, text in rendered.items():
        path = stub_path(module)
        committed = path.read_text(encoding="utf-8") if path.is_file() else ""
        if path.is_file() and committed == text:
            continue
        stale.append(path.relative_to(REPO_ROOT))
        if args.check:
            print(drift_report(stale[-1], committed, text))
        else:
            path.write_text(text, encoding="utf-8")
    (STUB_ROOT / "py.typed").touch()

    if args.check and stale:
        print("Stale stubs (run `pixi run stubs`):", *stale, sep="\n  ")
        return 1
    verb = "would rewrite" if args.check else "rewrote"
    print(f"{len(rendered)} modules, {verb} {len(stale)} stubs")
    return 0


if __name__ == "__main__":
    sys.exit(main())
