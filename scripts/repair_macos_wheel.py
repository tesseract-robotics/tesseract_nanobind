"""Turn the wheel `pip wheel` built on macOS into one that runs outside the build env.

Copies the dylib closure of the extension modules and the `dlopen`'d plugin factories into
`tesseract_robotics/.dylibs/`, points every reference at it, deletes the build's rpaths, patches
the packaged YAML, and rewrites RECORD. It fails on anything it can't place.

OpenMP is the one library left to the dynamic linker: images load `@rpath/libomp.dylib`, looking
in the env's `lib` first and at the bundled copy second. conda and pixi envs then share the
runtime numpy already loaded (#136), and an env with no `libomp` (uv) gets the bundled one.

Usage:
    python scripts/repair_macos_wheel.py dist/<wheel>.whl --env-lib "$CONDA_PREFIX/lib" --out wheelhouse
"""

import argparse
import base64
import hashlib
import os
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile
from dataclasses import dataclass
from pathlib import Path

PACKAGE = "tesseract_robotics"
BUNDLE = f"{PACKAGE}/.dylibs"
OPENMP = "libomp.dylib"
SYSTEM_PREFIXES = ("/usr/lib/", "/System/")
RPATH_PREFIX = "@rpath/"
LOADER_PREFIX = "@loader_path"
# From <env>/lib/python3.X/site-packages to <env>/lib.
SITE_PACKAGES_TO_ENV_LIB = 2
# Loaded with dlopen, so no load command leads to them.
PLUGINS = (
    "libtesseract_collision_bullet_factories.dylib",
    "libtesseract_collision_fcl_factories.dylib",
    "libtesseract_kinematics_factories.dylib",
    "libtesseract_kinematics_kdl_factories.dylib",
    "libtesseract_kinematics_opw_factory.dylib",
    "libtesseract_kinematics_ur_factory.dylib",
    "libtesseract_task_composer_factories.dylib",
    "libtesseract_task_composer_planning_factories.dylib",
    "libtesseract_task_composer_taskflow_factories.dylib",
)
_LOAD_COMMANDS = (
    "LC_LOAD_DYLIB",
    "LC_LOAD_WEAK_DYLIB",
    "LC_REEXPORT_DYLIB",
    "LC_LOAD_UPWARD_DYLIB",
)


class MissingDylibError(RuntimeError):
    """A dependency isn't in the env's lib directory."""


class ForeignDylibError(RuntimeError):
    """A dependency sits at an absolute path outside the env and the system."""


class MissingPluginError(RuntimeError):
    """A plugin factory the package loads isn't in the env's lib directory."""


class UnrepairedWheelError(RuntimeError):
    """The repaired wheel still reaches outside itself."""


@dataclass(frozen=True)
class MachO:
    """The load commands the repair reads from one image."""

    install_id: str | None
    deps: tuple[str, ...]
    rpaths: tuple[str, ...]


def parse_load_commands(otool_l: str) -> MachO:
    """Parse `otool -l` output."""
    install_id = None
    deps: list[str] = []
    rpaths: list[str] = []
    command = None
    for line in otool_l.splitlines():
        words = line.split()
        if len(words) >= 2 and words[0] == "cmd":
            command = words[1]
        elif len(words) >= 2 and words[0] == "name":
            if command == "LC_ID_DYLIB":
                install_id = words[1]
            elif command in _LOAD_COMMANDS:
                deps.append(words[1])
        elif len(words) >= 2 and words[0] == "path" and command == "LC_RPATH":
            rpaths.append(words[1])
    return MachO(install_id, tuple(deps), tuple(rpaths))


def is_system(dep: str) -> bool:
    """Whether macOS itself provides `dep`."""
    return dep.startswith(SYSTEM_PREFIXES)


def _relative(target: Path, image: Path) -> str:
    return Path(os.path.relpath(target, image.parent)).as_posix()


def reference_to(target: Path, image: Path) -> str:
    """The load path from `image` to `target`, both inside the wheel."""
    return f"{LOADER_PREFIX}/{_relative(target, image)}"


def resolve_source(dep: str, env_lib: Path) -> Path:
    """The real file in the env behind `dep`.

    SONAME symlinks resolve to their target: two names for one library load it twice, and
    anything with singleton state crashes on first use (#48).
    """
    if dep.startswith(RPATH_PREFIX):
        candidate = env_lib / dep.removeprefix(RPATH_PREFIX)
    elif Path(dep).is_absolute():
        candidate = Path(dep)
        if not candidate.is_relative_to(env_lib):
            raise ForeignDylibError(f"{dep} is outside the env ({env_lib}) and the system")
    else:
        raise ForeignDylibError(f"{dep} has no location the repair can resolve")
    if not candidate.exists():
        raise MissingDylibError(f"{dep} not found in {env_lib}")
    return candidate.resolve()


def openmp_rpaths(image: Path, root: Path, bundle: Path) -> tuple[str, str]:
    """The rpaths of an OpenMP user: the env's `lib` as installed, then the bundled copy."""
    depth = len(image.parent.relative_to(root).parts) + SITE_PACKAGES_TO_ENV_LIB
    env_lib = "/".join([LOADER_PREFIX, *[".."] * depth])
    to_bundle = _relative(bundle, image)
    bundled = LOADER_PREFIX if to_bundle == "." else f"{LOADER_PREFIX}/{to_bundle}"
    return env_lib, bundled


def read_macho(image: Path) -> MachO:
    """Read one image's load commands."""
    return parse_load_commands(
        subprocess.run(
            ["otool", "-l", str(image)], check=True, capture_output=True, text=True
        ).stdout
    )


def _copy_in(source: Path, bundle: Path) -> Path:
    target = bundle / source.name
    if not target.exists():
        shutil.copyfile(source, target)
        target.chmod(0o755)
    return target


def _images(root: Path) -> list[Path]:
    return sorted(p for p in root.rglob("*") if p.is_file() and p.suffix in (".so", ".dylib"))


def bundle_closure(root: Path, env_lib: Path) -> None:
    """Copy the closure into the bundle and rewrite every image's load commands."""
    bundle = root / BUNDLE
    bundle.mkdir(parents=True, exist_ok=True)
    for plugin in PLUGINS:
        source = env_lib / plugin
        if not source.exists():
            raise MissingPluginError(f"{plugin} not found in {env_lib}")
        _copy_in(source.resolve(), bundle)

    queue = _images(root)
    seen: set[Path] = set()
    while queue:
        image = queue.pop()
        if image in seen:
            continue
        seen.add(image)
        macho = read_macho(image)
        edits: list[str] = []
        uses_openmp = False
        for dep in macho.deps:
            if is_system(dep) or dep.startswith(LOADER_PREFIX):
                continue
            source = resolve_source(dep, env_lib)
            copied = source.name not in {p.name for p in bundle.iterdir()}
            target = _copy_in(source, bundle)
            if copied:
                queue.append(target)
            if source.name == OPENMP:
                uses_openmp = True
                reference = f"{RPATH_PREFIX}{OPENMP}"
            else:
                reference = reference_to(target, image)
            if dep != reference:
                edits += ["-change", dep, reference]
        for rpath in dict.fromkeys(macho.rpaths):
            edits += ["-delete_rpath", rpath]
        if uses_openmp:
            for rpath in openmp_rpaths(image, root, bundle):
                edits += ["-add_rpath", rpath]
        if macho.install_id is not None:
            wanted = (
                f"{RPATH_PREFIX}{OPENMP}"
                if image.name == OPENMP
                else f"{LOADER_PREFIX}/{image.name}"
            )
            if macho.install_id != wanted:
                edits += ["-id", wanted]
        if edits:
            subprocess.run(
                ["install_name_tool", *edits, str(image)], check=True, capture_output=True
            )
            # A rewrite invalidates the linker's signature, and dyld kills the process at load.
            subprocess.run(
                ["codesign", "--force", "--sign", "-", str(image)], check=True, capture_output=True
            )


def problems_in(image: Path, macho: MachO, root: Path, bundle: Path) -> list[str]:
    """Everything in `image` that reaches outside the wheel."""
    problems = []
    uses_openmp = False
    for dep in macho.deps:
        if is_system(dep):
            continue
        if dep == f"{RPATH_PREFIX}{OPENMP}":
            uses_openmp = True
            if not (bundle / OPENMP).exists():
                problems.append(f"{image.name}: {OPENMP} isn't bundled")
        elif dep.startswith(f"{LOADER_PREFIX}/"):
            if not (image.parent / dep.removeprefix(f"{LOADER_PREFIX}/")).exists():
                problems.append(f"{image.name}: {dep} isn't in the wheel")
        else:
            problems.append(f"{image.name}: stray reference {dep}")
    allowed = openmp_rpaths(image, root, bundle) if uses_openmp else ()
    problems += [f"{image.name}: stray rpath {r}" for r in macho.rpaths if r not in allowed]
    return problems


def verify(root: Path) -> None:
    """Refuse a wheel that reaches outside itself or carries one library under two names."""
    bundle = root / BUNDLE
    problems = []
    names: dict[str, Path] = {}
    for image in _images(root):
        problems += problems_in(image, read_macho(image), root, bundle)
        if image.name in names:
            problems.append(f"{image.name} is in the wheel twice: {names[image.name]}, {image}")
        names[image.name] = image
    links = [p for p in bundle.iterdir() if p.is_symlink()]
    problems += [f"{p.name} is a symlink" for p in links]
    if problems:
        raise UnrepairedWheelError("\n".join(problems))


def patch_plugin_path(source: str) -> str:
    """Replace the hardcoded plugin directory with the placeholder `__init__.py` resolves."""
    return source.replace("/usr/local/lib", '"@PLUGIN_PATH@"')


def strip_search_paths(source: str) -> str:
    """Drop `search_paths:` and absolute-path list items, so the env vars from `__init__.py` apply."""
    kept = [
        line
        for line in source.splitlines(keepends=True)
        if "search_paths:" not in line and not re.match(r"^\s*- /.*$", line)
    ]
    return "".join(kept)


def patch_yaml(root: Path) -> None:
    """Patch the task composer configs and robot YAMLs for the installed layout."""
    for path in (root / PACKAGE / "data" / "task_composer_config").glob("*.yaml"):
        path.write_text(patch_plugin_path(path.read_text()))
    for path in (root / PACKAGE / "data" / "tesseract" / "support").rglob("*.yaml"):
        path.write_text(strip_search_paths(path.read_text()))


def write_record(root: Path) -> None:
    """Rewrite RECORD from the files actually in the wheel."""
    record = next(root.glob("*.dist-info")) / "RECORD"
    rows = []
    for path in sorted(p for p in root.rglob("*") if p.is_file() and p != record):
        data = path.read_bytes()
        digest = base64.urlsafe_b64encode(hashlib.sha256(data).digest()).rstrip(b"=").decode()
        rows.append(f"{path.relative_to(root).as_posix()},sha256={digest},{len(data)}")
    rows.append(f"{record.relative_to(root).as_posix()},,")
    record.write_text("\n".join(rows) + "\n")


def pack(root: Path, wheel: Path) -> None:
    """Zip the tree into `wheel`, dist-info last as the wheel spec recommends."""
    files = sorted(
        (p for p in root.rglob("*") if p.is_file()),
        key=lambda p: (p.relative_to(root).parts[0].endswith(".dist-info"), p.as_posix()),
    )
    with zipfile.ZipFile(wheel, "w", zipfile.ZIP_DEFLATED) as archive:
        for path in files:
            archive.write(path, path.relative_to(root).as_posix())


def repair(wheel: Path, env_lib: Path, out: Path) -> Path:
    """Repair `wheel` into `out` and return the new wheel's path."""
    out.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=out) as scratch:
        root = Path(scratch)
        with zipfile.ZipFile(wheel) as archive:
            archive.extractall(root)
        bundle_closure(root, env_lib)
        patch_yaml(root)
        verify(root)
        write_record(root)
        repaired = out / wheel.name
        pack(root, repaired)
    return repaired


def main() -> int:
    """Command-line entry point."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("wheel", type=Path)
    parser.add_argument("--env-lib", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    print(repair(args.wheel, args.env_lib.resolve(), args.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
