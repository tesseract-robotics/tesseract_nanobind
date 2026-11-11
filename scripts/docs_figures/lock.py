"""Content-addressed lock of the docs figures: `docs/assets/figures/figures.lock.json`.

Per figure the lock holds the sha256 of every input (its generator module, the shared drawing
modules, the robot model files it loads, its measured-data JSON) and of every output SVG.
The upstream C++ SHAs the bindings were built from are recorded once, as an input of every
figure, and so is the render platform (OS, CPU architecture, matplotlib, FreeType, numpy and
trimesh versions) the committed SVGs were rendered on.

`compare` checks a fresh regeneration: the inputs must hash as the lock says on any platform;
the output bytes must match the committed SVGs and the lock on the render platform, and
elsewhere each output must be a well-formed SVG (the bytes are platform-dependent there, see
`render_platform`).
"""

from __future__ import annotations

import hashlib
import json
import platform
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from importlib.metadata import version
from pathlib import Path
from typing import Any, Final
from xml.etree import ElementTree

from matplotlib import ft2font

from scripts.docs_figures.style import FIGURES_DIR, REPO_ROOT
from tesseract_robotics import get_task_composer_config_path, get_tesseract_resource_path
from tesseract_robotics.build_identity import UPSTREAM_SOURCES

LOCK_PATH: Final[Path] = FIGURES_DIR / "figures.lock.json"
PACKAGE_DIR: Final[Path] = Path(__file__).resolve().parent
# Modules every figure is drawn through: a change in any of them can change every output.
SHARED_MODULES: Final[tuple[str, ...]] = ("style.py", "scene3d.py", "lock.py", "__main__.py")
# Prefixes of lock keys that name files outside the repository: the tesseract support data
# and the task composer config the installed package resolves.
SUPPORT_PREFIX: Final[str] = "tesseract_support:"
COMPOSER_CONFIG_PREFIX: Final[str] = "task_composer_config:"
LOCK_FORMAT: Final[int] = 2
SVG_NAMESPACE: Final[str] = "{http://www.w3.org/2000/svg}"


class FiguresLockError(RuntimeError):
    """The committed figures, their lock and a fresh regeneration disagree."""


def sha256(path: Path) -> str:
    """Hex sha256 of a file's content, independent of the checkout's line endings.

    Git on Windows checks text out with CRLF, so the bytes of a `.py`, URDF or YAML input differ
    from the LF commit that the lock was written from. UTF-8 text is hashed with CRLF folded to
    LF; anything else (a binary mesh) is hashed byte for byte. An LF file hashes as its bytes.
    """
    data = path.read_bytes()
    try:
        data.decode("utf-8")
    except UnicodeDecodeError:
        return hashlib.sha256(data).hexdigest()
    return hashlib.sha256(data.replace(b"\r\n", b"\n")).hexdigest()


def support_root() -> Path:
    """Root of the tesseract support data (`package://tesseract/...` resolves under it)."""
    return Path(get_tesseract_resource_path()) / "tesseract"


def support_files(patterns: Iterable[str]) -> list[Path]:
    """Files under the tesseract support data matching glob `patterns`; fails on an empty match."""
    root = support_root()
    files = []
    for pattern in patterns:
        matched = sorted(p for p in root.glob(pattern) if p.is_file())
        if not matched:
            raise FiguresLockError(f"no support file matches {pattern!r} under {root}")
        files.extend(matched)
    return files


def composer_config_dir() -> Path:
    """Directory of the task composer config `TaskComposer.from_config()` reads."""
    return Path(get_task_composer_config_path()).parent


def composer_config_files() -> list[Path]:
    """Every YAML of the task composer config (the root file `!include`s the others)."""
    files = sorted(composer_config_dir().rglob("*.yaml"))
    if not files:
        raise FiguresLockError(f"no task composer config under {composer_config_dir()}")
    return files


def _key(path: Path) -> str:
    path = path.resolve()
    if path.is_relative_to(REPO_ROOT):
        return path.relative_to(REPO_ROOT).as_posix()
    for prefix, root in (
        (SUPPORT_PREFIX, support_root().resolve()),
        (COMPOSER_CONFIG_PREFIX, composer_config_dir().resolve()),
    ):
        if path.is_relative_to(root):
            return prefix + path.relative_to(root).as_posix()
    raise FiguresLockError(
        f"{path} is not in the repository, the support data or the composer config"
    )


def upstream_identity() -> dict[str, str]:
    """The upstream C++ SHAs the bindings were built from: an input of every figure."""
    return {source.repository: source.sha for source in UPSTREAM_SOURCES}


def render_platform() -> dict[str, str]:
    """What turns identical inputs into identical bytes: OS, CPU architecture, numerics.

    Two runs on the same render platform produce the same SVG bytes. Across platforms, libm,
    BLAS and solver round-off can move a coordinate in the last digit the SVG keeps, so the
    bytes are only compared where this matches the lock's.
    """
    return {
        "system": platform.system(),
        "machine": platform.machine(),
        "matplotlib": version("matplotlib"),
        "freetype": ft2font.__freetype_version__,
        "numpy": version("numpy"),
        "trimesh": version("trimesh"),
    }


@dataclass(frozen=True)
class FigureEntry:
    """One figure's input and output hashes, keyed by repo-relative (or support) path."""

    inputs: dict[str, str]
    outputs: dict[str, str]


def input_hashes(module_name: str, extra_inputs: Iterable[Path]) -> dict[str, str]:
    """sha256 of a figure's generator module, the shared modules and its other inputs."""
    paths = [PACKAGE_DIR / f"{module_name}.py", *(PACKAGE_DIR / m for m in SHARED_MODULES)]
    paths.extend(extra_inputs)
    return {_key(p): sha256(p) for p in sorted(set(paths))}


def output_hashes(outputs: Iterable[Path], relative_to: Path) -> dict[str, str]:
    """sha256 of outputs, keyed as if they lay in `FIGURES_DIR` (so a scratch copy compares)."""
    return {_key(FIGURES_DIR / p.relative_to(relative_to)): sha256(p) for p in sorted(outputs)}


def read_lock() -> dict[str, Any]:
    """The committed lock; an empty one when there is none yet."""
    if not LOCK_PATH.exists():
        return {"format": LOCK_FORMAT, "upstream": {}, "render_platform": {}, "figures": {}}
    return json.loads(LOCK_PATH.read_text())


def write_lock(entries: Mapping[str, FigureEntry]) -> None:
    """Merge `entries` into the lock and write it (sorted keys, trailing newline)."""
    lock = read_lock()
    figures = dict(lock["figures"])
    for name, entry in entries.items():
        figures[name] = {"inputs": entry.inputs, "outputs": entry.outputs}
    written = {
        "format": LOCK_FORMAT,
        "upstream": upstream_identity(),
        # The platform the committed outputs were rendered on: their bytes reproduce there.
        "render_platform": render_platform(),
        "figures": dict(sorted(figures.items())),
    }
    LOCK_PATH.write_text(json.dumps(written, indent=2, sort_keys=True) + "\n")


def svg_problems(path: Path) -> list[str]:
    """Why `path` is not a well-formed, non-empty matplotlib SVG (empty when it is)."""
    try:
        root = ElementTree.parse(path).getroot()
    except ElementTree.ParseError as error:
        return [f"{path.name}: not well-formed XML ({error})"]
    problems = []
    if root.tag != f"{SVG_NAMESPACE}svg":
        problems.append(f"{path.name}: root element is {root.tag}, not svg")
    for attribute in ("width", "height", "viewBox"):
        if attribute not in root.attrib:
            problems.append(f"{path.name}: svg element has no {attribute}")
    if root.find(f".//{SVG_NAMESPACE}path") is None:
        problems.append(f"{path.name}: draws no path")
    return problems


def compare(entries: Mapping[str, FigureEntry], render_dir: Path) -> tuple[bool, list[str]]:
    """Every disagreement between a fresh regeneration in `render_dir` and the commit.

    Inputs (every file hash and the upstream SHAs) must match the lock on any platform. On the
    lock's render platform the outputs must match the committed SVGs and the lock bit for bit;
    on any other platform each output must instead be a well-formed SVG that draws something.

    Returns:
        Whether the bytes were compared (same render platform), and the problems found.
    """
    lock = read_lock()
    problems = []
    if lock["upstream"] != upstream_identity():
        problems.append(
            f"upstream SHAs changed: lock {lock['upstream']} != now {upstream_identity()}"
        )
    same_platform = lock["render_platform"] == render_platform()
    locked: dict[str, dict[str, dict[str, str]]] = lock["figures"]
    for name, entry in entries.items():
        if name not in locked:
            problems.append(f"{name}: not in {_key(LOCK_PATH)}")
            continue
        for key, digest in entry.inputs.items():
            if locked[name]["inputs"].get(key) != digest:
                problems.append(f"{name}: input {key} changed since the lock")
        for key in set(locked[name]["inputs"]) - set(entry.inputs):
            problems.append(f"{name}: input {key} is in the lock but no longer used")
        if set(entry.outputs) != set(locked[name]["outputs"]):
            problems.append(
                f"{name}: outputs {sorted(entry.outputs)} != lock {sorted(locked[name]['outputs'])}"
            )
        for key, digest in entry.outputs.items():
            if not same_platform:
                problems.extend(svg_problems(render_dir / (REPO_ROOT / key).name))
                continue
            committed = REPO_ROOT / key
            if not committed.exists():
                problems.append(f"{name}: {key} is not committed")
                continue
            if sha256(committed) != digest:
                problems.append(f"{name}: {key} differs from a fresh render")
            if locked[name]["outputs"].get(key) != digest:
                problems.append(f"{name}: lock hash of {key} differs from a fresh render")
    return same_platform, problems
