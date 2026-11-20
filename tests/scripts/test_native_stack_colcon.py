"""scripts/native_stack.py: the macOS deployment target reaches every build it starts."""

import importlib.util
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
_spec = importlib.util.spec_from_file_location("native_stack", REPO / "scripts" / "native_stack.py")
native_stack = importlib.util.module_from_spec(_spec)
sys.modules["native_stack"] = native_stack
_spec.loader.exec_module(native_stack)


def _stack(tmp_path: Path, triplet: str, monkeypatch) -> "native_stack.Stack":
    monkeypatch.setattr(native_stack, "require_tool", lambda name: name)
    libomp = tmp_path / "libomp"
    (libomp / "include").mkdir(parents=True, exist_ok=True)
    (libomp / "lib").mkdir(exist_ok=True)
    (libomp / "include" / "omp.h").touch()
    (libomp / "lib" / "libomp.dylib").touch()
    return native_stack.Stack(
        triplet=triplet,
        root=tmp_path / "root",
        binary_cache=tmp_path / "cache",
        libomp_prefix=libomp,
    )


@pytest.mark.parametrize("triplet", ["arm64-osx-dynamic-release", "x64-osx-dynamic-release"])
def test_macos_colcon_environment_carries_the_deployment_target(tmp_path, monkeypatch, triplet):
    """Every compiler and nested CMake configure colcon starts inherits the target.

    trajopt's osqp, osqp_eigen_ext, piqp and qpoases build through ExternalProject_Add, which
    forwards no CMAKE_OSX_DEPLOYMENT_TARGET: without the environment variable those libraries
    were built for the host's macOS and the repaired wheel's tag rose with them.
    """
    _args, env = native_stack.colcon_command(_stack(tmp_path, triplet, monkeypatch))
    assert env["MACOSX_DEPLOYMENT_TARGET"] == native_stack.OSX_DEPLOYMENT_TARGET


@pytest.mark.parametrize("triplet", ["arm64-osx-dynamic-release", "x64-osx-dynamic-release"])
def test_macos_binding_build_carries_the_deployment_target(tmp_path, monkeypatch, triplet):
    """The binding's pip build gets the same target (scikit-build-core tags the wheel from it)."""
    env = _stack(tmp_path, triplet, monkeypatch).binding_env()
    assert env["MACOSX_DEPLOYMENT_TARGET"] == native_stack.OSX_DEPLOYMENT_TARGET


@pytest.mark.parametrize("triplet", ["x64-linux-dynamic-release", "x64-windows-release"])
def test_other_platforms_get_no_macos_deployment_target(tmp_path, monkeypatch, triplet):
    stack = _stack(tmp_path, triplet, monkeypatch)
    _args, env = native_stack.colcon_command(stack)
    assert "MACOSX_DEPLOYMENT_TARGET" not in env
    assert "MACOSX_DEPLOYMENT_TARGET" not in stack.binding_env()


@pytest.mark.parametrize("triplet", ["arm64-osx-dynamic-release", "x64-osx-dynamic-release"])
def test_vcpkg_osx_triplets_declare_the_same_deployment_target(triplet):
    """vcpkg reads its target from the triplet overlay, a CMake file; it must agree."""
    text = (native_stack.TRIPLET_OVERLAYS / f"{triplet}.cmake").read_text()
    assert f"set(VCPKG_OSX_DEPLOYMENT_TARGET {native_stack.OSX_DEPLOYMENT_TARGET})" in text
