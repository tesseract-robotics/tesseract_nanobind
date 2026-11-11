"""Build the native C++ stack the bindings link against: the one path CI and the dev env share.

Layout under `--root` (CI: the workspace; default: `.native/<triplet>` of the host, gitignored):

    ws/src/      tesseract, trajopt, tesseract_planning, ... from dependencies.rosinstall
    ws/install/  the colcon install; this is TESSERACT_CPP_PREFIX
    vcpkg/       vcpkg checkout; vcpkg/installed/<triplet> holds the third-party libraries

Stages (`sources`, `vcpkg`, `colcon`, or `all`) each record a content-addressed identity next to
their output: the hash of every input (rosinstall, declarations, overlays, triplet, this script),
the tool versions and the identity of the stage before. A stage whose recorded identity equals
the current one is skipped.

Example:
    ```bash
    python scripts/native_stack.py all --libomp-prefix "$CONDA_PREFIX"
    python scripts/native_stack.py exec --libomp-prefix "$CONDA_PREFIX" -- pip install -e . --no-build-isolation
    ```
"""

import argparse
import hashlib
import json
import os
import platform
import shutil
import subprocess
import sys
import tomllib
from dataclasses import dataclass
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
ROSINSTALL = REPO / "dependencies.rosinstall"
DECLARATIONS = REPO / "scripts" / "native_stack.toml"
WORKFLOWS = REPO / ".github" / "workflows"
TRIPLET_OVERLAYS = WORKFLOWS / "vcpkg_triplets"
PORT_OVERLAYS = WORKFLOWS / "vcpkg_overlays"
PORT_OVERLAYS_WINDOWS = WORKFLOWS / "vcpkg_overlays_windows"
IDENTITY_FILE = ".native-stack-identity.json"
# Deployment target of the vcpkg_triplets osx triplets and the colcon build (macOS release).
OSX_DEPLOYMENT_TARGET = "12.0"
# Linux builds two colcon packages at a time, each with every core: four at once peak past a
# 16 GB runner.
LINUX_COLCON_WORKERS = "2"
# Upstream headers use std::runtime_error without <stdexcept>; the distro Boost pulled it in
# transitively, vcpkg's does not (tesseract-robotics/tesseract_planning#802).
LINUX_CXXFLAGS = "-include stdexcept"


class NativeStackError(RuntimeError):
    """Base of every failure of this script."""


class UnsupportedTripletError(NativeStackError):
    """The vcpkg triplet is not one of the platforms the stack is built for."""


class MissingToolError(NativeStackError):
    """A required build tool is not on PATH."""


class MissingInputError(NativeStackError):
    """A file or directory a stage needs does not exist (an earlier stage has not run)."""


class VcpkgCheckoutMismatchError(NativeStackError):
    """The vcpkg checkout under the root is not at the declared revision."""


class CommandFailedError(NativeStackError):
    """A subprocess exited non-zero."""


@dataclass(frozen=True)
class Host:
    """The operating system a triplet builds for."""

    name: str  # "linux", "osx" or "windows"

    @staticmethod
    def of_triplet(triplet: str) -> "Host":
        for name in ("linux", "osx", "windows"):
            if f"-{name}" in triplet:
                return Host(name)
        raise UnsupportedTripletError(f"unsupported triplet {triplet!r}")


def host_triplet() -> str:
    """The triplet of the machine running this script."""
    arch = {"arm64": "arm64", "aarch64": "arm64", "x86_64": "x64", "amd64": "x64"}[
        platform.machine().lower()
    ]
    if sys.platform == "darwin":
        return f"{arch}-osx-dynamic-release"
    if sys.platform.startswith("linux"):
        return f"{arch}-linux-dynamic-release"
    if sys.platform == "win32":
        return f"{arch}-windows-release"
    raise UnsupportedTripletError(f"unsupported platform {sys.platform!r}")


@dataclass(frozen=True)
class Stack:
    """One build of the native stack: a triplet built under a root."""

    triplet: str
    root: Path
    binary_cache: Path
    libomp_prefix: Path | None

    @property
    def host(self) -> Host:
        return Host.of_triplet(self.triplet)

    @property
    def src(self) -> Path:
        return self.root / "ws" / "src"

    @property
    def install(self) -> Path:
        """TESSERACT_CPP_PREFIX."""
        return self.root / "ws" / "install"

    @property
    def vcpkg_root(self) -> Path:
        return self.root / "vcpkg"

    @property
    def vcpkg_prefix(self) -> Path:
        return self.vcpkg_root / "installed" / self.triplet

    def vcpkg_exe(self) -> Path:
        return self.vcpkg_root / ("vcpkg.exe" if self.host.name == "windows" else "vcpkg")

    def planning_port_overlays(self) -> Path:
        return self.src / "tesseract_planning" / ".github" / "workflows" / "vcpkg_overlays"

    def runtime_library_env(self) -> dict[str, str]:
        """Library search path entries for a process that loads the stack's shared libraries."""
        libs = os.pathsep.join([str(self.vcpkg_prefix / "lib"), str(self.install / "lib")])
        if self.host.name == "linux":
            return {"LD_LIBRARY_PATH": prepend(libs, os.environ.get("LD_LIBRARY_PATH"))}
        if self.host.name == "osx":
            return {"DYLD_LIBRARY_PATH": prepend(libs, os.environ.get("DYLD_LIBRARY_PATH"))}
        bins = os.pathsep.join([str(self.vcpkg_prefix / "bin"), str(self.install / "bin")])
        return {"PATH": os.environ["PATH"] + os.pathsep + bins}

    def binding_env(self) -> dict[str, str]:
        """Environment `pip install .` needs to build the bindings against this stack."""
        env = {
            "TESSERACT_CPP_PREFIX": str(self.install),
            "CMAKE_PREFIX_PATH": os.pathsep.join([str(self.install), str(self.vcpkg_prefix)]),
            "CMAKE_POLICY_VERSION_MINIMUM": "3.5",
            **self.runtime_library_env(),
        }
        if self.host.name == "osx":
            # descartes_light's exported config needs OpenMP; point it at libomp as colcon does.
            omp = self.openmp_args(prefix="-D")
            env["CMAKE_ARGS"] = " ".join(
                f'"{arg}"' if " " in arg else arg
                for arg in [f"-DCMAKE_OSX_DEPLOYMENT_TARGET={OSX_DEPLOYMENT_TARGET}", *omp]
            )
        return env

    def openmp_args(self, prefix: str) -> list[str]:
        """CMake OpenMP definitions for Apple clang with an external libomp."""
        if self.libomp_prefix is None:
            raise MissingInputError("--libomp-prefix is required to build for macOS")
        include = self.libomp_prefix / "include"
        library = self.libomp_prefix / "lib" / "libomp.dylib"
        for path in (include / "omp.h", library):
            if not path.exists():
                raise MissingInputError(f"{path} does not exist (libomp prefix)")
        return [
            f"{prefix}OpenMP_CXX_INCLUDE_DIR={include}",
            f"{prefix}OpenMP_C_INCLUDE_DIR={include}",
            f"{prefix}OpenMP_CXX_LIB_NAMES=libomp",
            f"{prefix}OpenMP_C_LIB_NAMES=libomp",
            f"{prefix}OpenMP_CXX_FLAGS=-Xpreprocessor -fopenmp",
            f"{prefix}OpenMP_C_FLAGS=-Xpreprocessor -fopenmp",
            f"{prefix}OpenMP_libomp_LIBRARY={library}",
        ]


def prepend(entries: str, existing: str | None) -> str:
    return entries if not existing else entries + os.pathsep + existing


def require_tool(name: str) -> str:
    found = shutil.which(name)
    if found is None:
        raise MissingToolError(f"{name} is not on PATH")
    return found


def run(args: list[str], env: dict[str, str] | None = None, cwd: Path | None = None) -> None:
    """Run a command with `env` merged over the process environment, failing loudly."""
    print("+", " ".join(args), flush=True)
    merged = {**os.environ, **(env or {})}
    result = subprocess.run(args, env=merged, cwd=cwd)
    if result.returncode:
        raise CommandFailedError(f"{args[0]} exited with {result.returncode}")


def tool_version(*args: str) -> str:
    """First line a tool prints for its version (stdout and stderr; cl prints on stderr)."""
    require_tool(args[0])
    out = subprocess.run(args, capture_output=True, text=True, check=False)
    lines = (out.stdout + out.stderr).strip().splitlines()
    return lines[0] if lines else ""


def digest_paths(*paths: Path) -> str:
    """SHA-256 over the relative names and bytes of files and directories (recursive, sorted)."""
    sha = hashlib.sha256()
    for path in paths:
        if not path.exists():
            raise MissingInputError(f"{path} does not exist")
        files = sorted(p for p in path.rglob("*") if p.is_file()) if path.is_dir() else [path]
        for file in files:
            sha.update(file.relative_to(REPO).as_posix().encode())
            sha.update(file.read_bytes())
    return sha.hexdigest()


def declarations() -> dict:
    return tomllib.loads(DECLARATIONS.read_text())


def compiler_version(host: Host) -> str:
    return tool_version("cl") if host.name == "windows" else tool_version("c++", "--version")


def sources_identity() -> dict[str, str]:
    return {"rosinstall": digest_paths(ROSINSTALL)}


def vcpkg_identity(stack: Stack) -> dict[str, str]:
    overlays = [PORT_OVERLAYS_WINDOWS if stack.host.name == "windows" else PORT_OVERLAYS]
    if stack.host.name != "windows":
        overlays.append(TRIPLET_OVERLAYS)
    return {
        **sources_identity(),
        "triplet": stack.triplet,
        "declarations": digest_paths(DECLARATIONS),
        "overlays": digest_paths(*overlays),
        "script": digest_paths(Path(__file__).resolve()),
        "compiler": compiler_version(stack.host),
        "cmake": tool_version("cmake", "--version"),
    }


def colcon_identity(stack: Stack) -> dict[str, str]:
    return {
        **vcpkg_identity(stack),
        "ninja": tool_version("ninja", "--version"),
        "vcpkg_installed": json.dumps(read_identity(stack.vcpkg_prefix.parent), sort_keys=True),
    }


def read_identity(directory: Path) -> dict[str, str] | None:
    file = directory / IDENTITY_FILE
    return json.loads(file.read_text()) if file.is_file() else None


def write_identity(directory: Path, identity: dict[str, str]) -> None:
    (directory / IDENTITY_FILE).write_text(json.dumps(identity, indent=2, sort_keys=True) + "\n")


def fetch_sources(stack: Stack) -> None:
    """`vcs import` dependencies.rosinstall into ws/src."""
    stack.src.mkdir(parents=True, exist_ok=True)
    run([require_tool("vcs"), "import", "--input", str(ROSINSTALL), str(stack.src)])


def ensure_vcpkg(stack: Stack, revision: str, repository: str) -> None:
    """Clone vcpkg at the declared release tag and bootstrap it."""
    root = stack.vcpkg_root
    if not root.exists():
        run(
            [
                require_tool("git"),
                "clone",
                "--depth",
                "1",
                "--branch",
                revision,
                repository,
                str(root),
            ]
        )
    else:
        described = subprocess.run(
            ["git", "-C", str(root), "describe", "--tags", "--exact-match"],
            capture_output=True,
            text=True,
        ).stdout.strip()
        if described != revision:
            raise VcpkgCheckoutMismatchError(f"{root} is at {described!r}, declared {revision!r}")
    if not stack.vcpkg_exe().exists():
        script = "bootstrap-vcpkg.bat" if stack.host.name == "windows" else "bootstrap-vcpkg.sh"
        run([str(root / script), "-disableMetrics"], cwd=root)


def build_vcpkg(stack: Stack) -> None:
    """Install the declared vcpkg packages for the triplet, with the repo's overlays."""
    config = declarations()["vcpkg"]
    if not stack.planning_port_overlays().is_dir():
        raise MissingInputError(f"{stack.planning_port_overlays()}: run the sources stage first")
    ensure_vcpkg(stack, config["revision"], config["repository"])
    stack.binary_cache.mkdir(parents=True, exist_ok=True)
    args = [str(stack.vcpkg_exe()), "install", "--triplet", stack.triplet, "--clean-after-build"]
    if stack.host.name == "windows":
        args.append(f"--overlay-ports={PORT_OVERLAYS_WINDOWS}")
    else:
        args += [f"--overlay-triplets={TRIPLET_OVERLAYS}", f"--overlay-ports={PORT_OVERLAYS}"]
    args.append(f"--overlay-ports={stack.planning_port_overlays()}")
    args += config["packages"]
    env = {
        "VCPKG_ROOT": str(stack.vcpkg_root),
        "VCPKG_DEFAULT_BINARY_CACHE": str(stack.binary_cache),
        "VCPKG_DISABLE_METRICS": "1",
    }
    if stack.host.name == "linux":
        # vcpkg ships no prebuilt tools for arm64 Linux; use the system cmake and ninja.
        env["VCPKG_FORCE_SYSTEM_BINARIES"] = "1"
    run(args, env=env, cwd=stack.vcpkg_root)


def colcon_command(stack: Stack) -> tuple[list[str], dict[str, str]]:
    """The colcon invocation and environment of the stack's platform."""
    host = stack.host.name
    env = {"CMAKE_PREFIX_PATH": str(stack.vcpkg_prefix), **stack.runtime_library_env()}
    ignore = ["tesseract_examples", "gtest"]
    cmake_args = [
        "-GNinja",
        "-DCMAKE_BUILD_TYPE=Release",
        "-DCMAKE_C_COMPILER_LAUNCHER=ccache",
        "-DCMAKE_CXX_COMPILER_LAUNCHER=ccache",
        "-DTESSERACT_ENABLE_EXAMPLES=OFF",
        "-DTESSERACT_PLUGIN_FACTORY_CALLBACKS=ON",
        "-DTESSERACT_BUILD_TRAJOPT_IFOPT=ON",
        "-DCMAKE_POLICY_VERSION_MINIMUM=3.5",
    ]
    extra: list[str] = []
    if host == "linux":
        env["CXXFLAGS"] = LINUX_CXXFLAGS
        extra += ["--parallel-workers", LINUX_COLCON_WORKERS]
        cmake_args += ["-DBUILD_IPOPT=OFF", "-DBUILD_SNOPT=OFF", "-DTESSERACT_ENABLE_TESTING=OFF"]
    elif host == "osx":
        ignore = ["tesseract_examples", "ifopt", "vhacd"]
        cmake_args += [
            "-DINSTALL_OMPL=OFF",
            "-DINSTALL_OMPL_TAG=master",
            "-DBUILD_IPOPT=OFF",
            "-DBUILD_SNOPT=OFF",
            "-DBUILD_SHARED_LIBS=ON",
            "-DVCPKG_APPLOCAL_DEPS=OFF",
            "-DTESSERACT_ENABLE_TESTING=OFF",
            f"-DCMAKE_OSX_DEPLOYMENT_TARGET={OSX_DEPLOYMENT_TARGET}",
            *stack.openmp_args(prefix="-D"),
            f"-Dtcmalloc_minimal_LIBRARY={stack.vcpkg_prefix / 'lib' / 'libtcmalloc_minimal.dylib'}",
        ]
    else:
        env["CC"] = "cl"
        env["CXX"] = "cl"
        cmake_args += ["-DVCPKG_APPLOCAL_DEPS=OFF"]
    args = [
        require_tool("colcon"),
        "build",
        "--merge-install",
        *extra,
        "--cmake-force-configure",
        "--packages-ignore",
        *ignore,
        "--event-handlers",
        "console_cohesion+",
        "--cmake-args",
        *cmake_args,
    ]
    return args, env


def build_colcon(stack: Stack) -> None:
    if not stack.vcpkg_prefix.is_dir():
        raise MissingInputError(f"{stack.vcpkg_prefix}: run the vcpkg stage first")
    args, env = colcon_command(stack)
    run(args, env=env, cwd=stack.root / "ws")


def stage(name: str, stack: Stack) -> None:
    """Run one stage unless its recorded identity equals the current one."""
    if name == "sources":
        fetch_sources(stack)
        return
    if name == "vcpkg":
        directory, identity, build = stack.vcpkg_prefix.parent, vcpkg_identity(stack), build_vcpkg
    else:
        directory, identity, build = stack.install, colcon_identity(stack), build_colcon
    if read_identity(directory) == identity:
        print(f"{name}: up to date ({directory})")
        return
    build(stack)
    write_identity(directory, identity)


def build_stack(args: argparse.Namespace) -> Stack:
    triplet = args.triplet or host_triplet()
    root = (args.root or REPO / ".native" / triplet).resolve()
    return Stack(
        triplet=triplet,
        root=root,
        binary_cache=(args.binary_cache or root / "vcpkg_cache").resolve(),
        libomp_prefix=args.libomp_prefix,
    )


def main(argv: list[str]) -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument(
        "command", choices=("sources", "vcpkg", "colcon", "all", "github-env", "exec")
    )
    parser.add_argument("--triplet", help="vcpkg triplet (default: this machine's)")
    parser.add_argument("--root", type=Path, help="build root (default: <repo>/.native/<triplet>)")
    parser.add_argument(
        "--binary-cache", type=Path, help="vcpkg binary cache (default: <root>/vcpkg_cache)"
    )
    parser.add_argument(
        "--libomp-prefix",
        type=Path,
        help="macOS: prefix holding include/omp.h and lib/libomp.dylib",
    )
    parser.add_argument("rest", nargs="*", help="exec: the command, after --")
    args = parser.parse_args(argv)
    if args.command == "github-env":
        # Newline-terminated with LF on every OS: a CR would end up in the variable's value.
        with Path(os.environ["GITHUB_ENV"]).open("a", newline="\n") as env_file:
            env_file.write(f"VCPKG_REVISION={declarations()['vcpkg']['revision']}\n")
        return
    stack = build_stack(args)
    if args.command == "exec":
        run(args.rest, env=stack.binding_env())
        return
    for name in ("sources", "vcpkg", "colcon") if args.command == "all" else (args.command,):
        stage(name, stack)


if __name__ == "__main__":
    try:
        main(sys.argv[1:])
    except NativeStackError as error:
        sys.exit(f"native_stack: {type(error).__name__}: {error}")
