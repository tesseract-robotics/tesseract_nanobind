"""Preserve native installs and vcpkg binary archives across hosted runners."""

import argparse
import json
import tarfile
from pathlib import Path


class MissingNativeInstallError(RuntimeError):
    """A producer did not create the native install it is trying to publish."""


class NativeEnvironmentMismatchError(RuntimeError):
    """A native bundle targets a different prefix, runner or toolchain."""


def extract_bundle(bundle: Path, target: Path) -> None:
    """Restore permissions and internal symlinks, rejecting paths outside target."""
    target.mkdir(parents=True, exist_ok=True)
    with tarfile.open(bundle) as stream:
        stream.extractall(target, filter="data")


def pack_native(workspace: Path, output: Path, environment: dict[str, str]) -> None:
    """Archive both CMake install trees; their absolute references keep the same layout."""
    paths = ("ws/install", "vcpkg/installed")
    for relative in paths:
        if not (workspace / relative).is_dir():
            raise MissingNativeInstallError(f"Missing {workspace / relative}")
    output.mkdir(parents=True, exist_ok=True)
    (output / "native-environment.json").write_text(json.dumps(environment, indent=2) + "\n")
    with tarfile.open(output / "native-stack.tar", "w") as stream:
        for relative in paths:
            stream.add(workspace / relative, arcname=relative)


def unpack_native(artifact: Path, workspace: Path, expected: dict[str, str]) -> None:
    """Refuse an incompatible bundle before extracting any installed files."""
    actual = json.loads((artifact / "native-environment.json").read_text())
    if actual != expected:
        raise NativeEnvironmentMismatchError(
            f"Native build environment differs: {actual} != {expected}"
        )
    extract_bundle(artifact / "native-stack.tar", workspace)
    for relative in ("ws/install", "vcpkg/installed"):
        if not (workspace / relative).is_dir():
            raise MissingNativeInstallError(f"Restored bundle lacks {relative}")


def pack_vcpkg(cache: Path, output: Path) -> None:
    """vcpkg validates the binary packages when it consumes this archive."""
    if not cache.is_dir():
        raise MissingNativeInstallError(f"Missing vcpkg binary cache: {cache}")
    output.mkdir(parents=True, exist_ok=True)
    with tarfile.open(output / "vcpkg-binaries.tar", "w") as stream:
        stream.add(cache, arcname="vcpkg_cache")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "operation", choices=("pack-native", "unpack-native", "pack-vcpkg", "unpack-vcpkg")
    )
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--artifact", type=Path, required=True)
    parser.add_argument("--environment", type=Path)
    args = parser.parse_args()
    if args.operation in ("pack-native", "unpack-native"):
        if args.environment is None:
            parser.error("--environment is required for native installs")
        environment = json.loads(args.environment.read_text())
        if args.operation == "pack-native":
            pack_native(args.workspace, args.artifact, environment)
        else:
            unpack_native(args.artifact, args.workspace, environment)
    elif args.operation == "pack-vcpkg":
        pack_vcpkg(args.workspace / "vcpkg_cache", args.artifact)
    else:
        extract_bundle(args.artifact / "vcpkg-binaries.tar", args.workspace)
