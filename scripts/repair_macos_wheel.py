"""Turn the wheel `pip wheel` built on macOS into one that runs outside the build env.

Copies the dylib closure of the extension modules and the `dlopen`'d plugin factories into
`tesseract_robotics/.dylibs/`, points every reference at it, deletes the build's rpaths, patches
the packaged YAML, and rewrites RECORD. It fails on anything it can't place.

OpenMP is the one library left to the dynamic linker: images load `@rpath/libomp.dylib`, looking
in the env's `lib` first and at the bundled copy second. conda and pixi envs then share the
runtime numpy already loaded (#136), and an env with no `libomp` (uv) gets the bundled one.

The vcpkg/anchor mode takes repeated --lib-dir arguments instead of --env-lib.
It follows linked plugin factories without a manual list, keeps YAML unchanged,
and tags the wheel for the native stack's macOS deployment target
(native_stack.OSX_DEPLOYMENT_TARGET), refusing a bundled image built for a newer macOS
(DeploymentTargetExceededError). The original --env-lib mode remains available for older
conda-built wheels.

Usage:
    python scripts/repair_macos_wheel.py dist/<wheel>.whl --lib-dir ws/install/lib --out wheelhouse
"""

import argparse
import base64
import hashlib
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

from native_stack import OSX_DEPLOYMENT_TARGET

# A macOS version as (major, minor, patch), from LC_BUILD_VERSION's minos or a deployment target.
OsVersion = tuple[int, int, int]

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
# pip matches only macosx_<major>_0 from macOS 11 (Big Sur) on: a wheel tagged with a nonzero
# minor there installs on no Mac.
MACOS_MAJOR_ONLY_TAGS_FROM = 11
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


class DeploymentTargetExceededError(RuntimeError):
    """A bundled image needs a newer macOS than the declared deployment target."""


class UninstallableTagError(RuntimeError):
    """The deployment target makes a macOS wheel tag pip matches on no Mac."""


@dataclass(frozen=True)
class MachO:
    """The load commands the repair reads from one image."""

    install_id: str | None
    deps: tuple[str, ...]
    rpaths: tuple[str, ...]
    minimum_os: OsVersion = (0, 0, 0)


def parse_os_version(text: str) -> OsVersion:
    """Parse a dotted macOS version ("12", "12.0", "15.3.1") into (major, minor, patch)."""
    parts = [int(part) for part in text.split(".")]
    return (*(parts + [0, 0])[:3],)


# The deployment target the wheel is tagged with: the one the native stack is built for.
DEPLOYMENT_TARGET = parse_os_version(OSX_DEPLOYMENT_TARGET)


def format_os_version(version: OsVersion) -> str:
    """The version as major.minor, with .patch only when nonzero."""
    major, minor, patch = version
    return f"{major}.{minor}.{patch}" if patch else f"{major}.{minor}"


def parse_load_commands(otool_l: str) -> MachO:
    """Parse `otool -l` output."""
    install_id = None
    deps: list[str] = []
    rpaths: list[str] = []
    command = None
    minimum_os = (0, 0, 0)
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
        elif len(words) >= 2 and (
            (words[0] == "minos" and command == "LC_BUILD_VERSION")
            or (words[0] == "version" and command == "LC_VERSION_MIN_MACOSX")
        ):
            minimum_os = parse_os_version(words[1])
    return MachO(install_id, tuple(deps), tuple(rpaths), minimum_os)


def is_system(dep: str) -> bool:
    """Whether macOS itself provides `dep`."""
    return dep.startswith(SYSTEM_PREFIXES)


def _relative(target: Path, image: Path) -> str:
    return target.relative_to(image.parent, walk_up=True).as_posix()


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
                ["codesign", "--remove-signature", str(image)], check=True, capture_output=True
            )
            subprocess.run(["codesign", "--sign", "-", str(image)], check=True, capture_output=True)


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
    if "search_paths:" not in source:
        return source
    kept = [
        line
        for line in source.splitlines(keepends=True)
        if "search_paths:" not in line and not re.match(r"^\s*- /.*$", line)
    ]
    return "".join(kept)


def _rewrite_text(path: Path, edit: Callable[[str], str]) -> None:
    """Apply `edit` to a UTF-8 file in place, keeping its line endings on every platform.

    `newline=""` turns off newline translation both ways: Windows text mode would otherwise
    write each "\\n" as "\\r\\n", and the locale codepage would stand in for UTF-8.
    """
    with path.open(encoding="utf-8", newline="") as source:
        text = source.read()
    path.write_text(edit(text), encoding="utf-8", newline="")


def patch_yaml(root: Path) -> None:
    """Patch the task composer configs and robot YAMLs for the installed layout."""
    for path in (root / PACKAGE / "data" / "task_composer_config").glob("*.yaml"):
        _rewrite_text(path, patch_plugin_path)
    for path in (root / PACKAGE / "data" / "tesseract" / "support").rglob("*.yaml"):
        _rewrite_text(path, strip_search_paths)


def write_record(root: Path) -> None:
    """Rewrite RECORD from the files actually in the wheel."""
    record = next(root.glob("*.dist-info")) / "RECORD"
    rows = []
    for path in sorted(p for p in root.rglob("*") if p.is_file() and p != record):
        data = path.read_bytes()
        digest = base64.urlsafe_b64encode(hashlib.sha256(data).digest()).rstrip(b"=").decode()
        rows.append(f"{path.relative_to(root).as_posix()},sha256={digest},{len(data)}")
    rows.append(f"{record.relative_to(root).as_posix()},,")
    # LF on every platform, as the `wheel` package writes RECORD.
    record.write_text("\n".join(rows) + "\n", encoding="utf-8", newline="\n")


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


class DuplicateDylibError(RuntimeError):
    """Distinct source libraries would occupy the same bundled filename."""


class InvalidWheelTagError(RuntimeError):
    """The input is not a single-platform macOS wheel."""


def resolve_anchored_source(
    dep: str, image: Path, rpaths: tuple[str, ...], lib_dirs: tuple[Path, ...]
) -> Path:
    """Resolve a load command within explicitly allowed library directories.

    Explicit library directories have the same precedence as DYLD_LIBRARY_PATH;
    image rpaths follow them. Absolute dependencies must exist at their named path.
    """
    if Path(dep).is_absolute():
        candidates = [Path(dep)]
    elif dep.startswith(f"{LOADER_PREFIX}/"):
        candidates = [image.parent / dep.removeprefix(f"{LOADER_PREFIX}/")]
    elif dep.startswith(RPATH_PREFIX):
        name = dep.removeprefix(RPATH_PREFIX)
        search = list(lib_dirs)
        for rpath in rpaths:
            if rpath == LOADER_PREFIX:
                search.append(image.parent)
            elif rpath.startswith(f"{LOADER_PREFIX}/"):
                search.append(image.parent / rpath.removeprefix(f"{LOADER_PREFIX}/"))
            elif Path(rpath).is_absolute():
                search.append(Path(rpath))
        candidates = [directory / name for directory in search]
    else:
        raise ForeignDylibError(f"Unsupported load command {dep} in {image}")
    for candidate in candidates:
        if candidate.exists():
            resolved = candidate.resolve()
            if not any(resolved.is_relative_to(directory) for directory in lib_dirs):
                raise ForeignDylibError(f"{dep} resolves outside allowed libraries: {resolved}")
            return resolved
    raise MissingDylibError(f"{dep} from {image} not found in {lib_dirs}")


def bundle_anchored_closure(root: Path, lib_dirs: tuple[Path, ...]) -> dict[str, OsVersion]:
    """Bundle the linked plugin closure, editing and signing each image once.

    The original source path is retained while traversing copied libraries, so
    loader-relative dependencies resolve against their original directory.
    Plugin anchors supply ordinary load commands; no plugin list or YAML rewrite
    is needed.

    Returns:
        The minimum macOS of every image in the wheel, by its path relative to `root`.
    """
    bundle = root / BUNDLE
    bundle.mkdir(parents=True, exist_ok=True)
    allowed = (*lib_dirs, root)
    queue = [(image, image) for image in _images(root)]
    sources = {image: image for image, _ in queue if image.parent == bundle}
    seen: set[Path] = set()
    minimum_os: dict[str, OsVersion] = {}
    while queue:
        image, original = queue.pop()
        if image in seen:
            continue
        seen.add(image)
        macho = read_macho(original)
        minimum_os[image.relative_to(root).as_posix()] = macho.minimum_os
        edits: list[str] = []
        uses_openmp = False
        for dep in macho.deps:
            if is_system(dep):
                continue
            source = resolve_anchored_source(dep, original, macho.rpaths, allowed)
            target = bundle / source.name
            previous = sources.get(target)
            if previous is not None and previous != source:
                raise DuplicateDylibError(f"{previous} and {source} both map to {target}")
            sources[target] = source
            if target != source:
                _copy_in(source, bundle)
            queue.append((target, source))
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
            image.chmod(image.stat().st_mode | 0o200)
            subprocess.run(
                ["install_name_tool", *edits, str(image)], check=True, capture_output=True
            )
            subprocess.run(
                ["codesign", "--remove-signature", str(image)], check=True, capture_output=True
            )
            subprocess.run(["codesign", "--sign", "-", str(image)], check=True, capture_output=True)
    return minimum_os


def check_deployment_target(minimum_os: dict[str, OsVersion], target: OsVersion) -> None:
    """Refuse a wheel whose images need a newer macOS than `target`.

    The tag is the deployment target, never raised to fit the closure: an image above it was
    built without the target (a sub-build that did not inherit it), and is fixed at its build.

    Args:
        minimum_os: From `bundle_anchored_closure`: each image's minimum macOS.
        target: The declared deployment target.

    Raises:
        DeploymentTargetExceededError: Some image's minimum macOS is above `target`.
    """
    above = sorted((name, version) for name, version in minimum_os.items() if version > target)
    if above:
        found = "\n".join(f"{name}: {format_os_version(version)}" for name, version in above)
        raise DeploymentTargetExceededError(
            f"{len(above)} bundled image(s) need a newer macOS than the deployment target "
            f"{format_os_version(target)} (native_stack.OSX_DEPLOYMENT_TARGET):\n{found}\n"
            "Rebuild them with MACOSX_DEPLOYMENT_TARGET set (native_stack.py sets it for colcon "
            "and the bindings); a library from outside the stack needs a build for that target."
        )


def retag_macos(root: Path, name: str, target: OsVersion) -> str:
    """Tag the wheel filename and its WHEEL metadata for the deployment target.

    Args:
        root: The unpacked wheel.
        name: The wheel's filename as built.
        target: The declared deployment target.

    Returns:
        The filename with the platform tag `macosx_<major>_<minor>_<arch>` of `target`.

    Raises:
        InvalidWheelTagError: `name` is not a single-platform macOS wheel.
        UninstallableTagError: `target` has a nonzero minor from macOS 11 on.
    """
    prefix, platform = name.removesuffix(".whl").rsplit("-", 1)
    match = re.fullmatch(r"macosx_(\d+)_(\d+)_(arm64|x86_64)", platform)
    if match is None:
        raise InvalidWheelTagError(name)
    major, minor, _patch = target
    if major >= MACOS_MAJOR_ONLY_TAGS_FROM and minor:
        raise UninstallableTagError(
            f"deployment target {format_os_version(target)}: pip matches only "
            f"macosx_{major}_0 from macOS {MACOS_MAJOR_ONLY_TAGS_FROM} on"
        )
    replacement = f"macosx_{major}_{minor}_{match[3]}"
    _rewrite_text(
        next(root.glob("*.dist-info/WHEEL")),
        lambda text: "".join(
            line.replace(platform, replacement) if line.startswith("Tag: ") else line
            for line in text.splitlines(keepends=True)
        ),
    )
    return f"{prefix}-{replacement}.whl"


def repair_anchored(
    wheel: Path,
    lib_dirs: tuple[Path, ...],
    out: Path,
    *,
    deployment_target: OsVersion = DEPLOYMENT_TARGET,
) -> Path:
    """Repair an anchored vcpkg wheel without changing its packaged YAML.

    Args:
        wheel: The wheel `pip wheel` built.
        lib_dirs: The library directories the closure may come from.
        out: Where the repaired wheel goes.
        deployment_target: The macOS the wheel is tagged for; no bundled image may need newer.

    Returns:
        The repaired wheel's path.

    Raises:
        DeploymentTargetExceededError: A bundled image needs a newer macOS.
    """
    out.mkdir(parents=True, exist_ok=True)
    lib_dirs = tuple(directory.resolve(strict=True) for directory in lib_dirs)
    with tempfile.TemporaryDirectory(dir=out) as scratch:
        root = Path(scratch).resolve()
        with zipfile.ZipFile(wheel) as archive:
            archive.extractall(root)
        minimum_os = bundle_anchored_closure(root, lib_dirs)
        check_deployment_target(minimum_os, deployment_target)
        verify(root)
        name = retag_macos(root, wheel.name, deployment_target)
        # RECORD checksums are mandated by the wheel format, not an application identity.
        write_record(root)
        repaired = out / name
        pack(root, repaired)
    return repaired


def main() -> int:
    """Command-line entry point."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("wheel", type=Path)
    sources = parser.add_mutually_exclusive_group(required=True)
    sources.add_argument("--env-lib", type=Path)
    sources.add_argument("--lib-dir", type=Path, action="append")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    if args.lib_dir:
        print(repair_anchored(args.wheel, tuple(args.lib_dir), args.out))
    else:
        print(repair(args.wheel, args.env_lib.resolve(), args.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
