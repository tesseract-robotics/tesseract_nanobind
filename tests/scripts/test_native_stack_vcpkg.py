"""scripts/native_stack.py: every platform's vcpkg install gets the shared port overlays."""

import importlib.util
import json
import re
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
_spec = importlib.util.spec_from_file_location("native_stack", REPO / "scripts" / "native_stack.py")
native_stack = importlib.util.module_from_spec(_spec)
sys.modules["native_stack"] = native_stack
_spec.loader.exec_module(native_stack)

TRIPLETS = ["arm64-osx-dynamic-release", "x64-linux-dynamic-release", "x64-windows-release"]


def _stack(tmp_path: Path, triplet: str, monkeypatch) -> "native_stack.Stack":
    monkeypatch.setattr(native_stack, "require_tool", lambda name: name)
    libomp = tmp_path / "libomp"
    (libomp / "include").mkdir(parents=True, exist_ok=True)
    (libomp / "lib").mkdir(exist_ok=True)
    (libomp / "include" / "omp.h").touch()
    (libomp / "lib" / "libomp.dylib").touch()
    stack = native_stack.Stack(
        triplet=triplet,
        root=tmp_path / "root",
        binary_cache=tmp_path / "cache",
        libomp_prefix=libomp,
    )
    stack.planning_port_overlays().mkdir(parents=True)
    return stack


@pytest.mark.parametrize("triplet", TRIPLETS)
def test_vcpkg_install_gets_the_common_port_overlays(tmp_path, monkeypatch, triplet):
    """The common overlays (kubazip's pinned archive) reach the install on every host."""
    stack = _stack(tmp_path, triplet, monkeypatch)
    calls = []
    monkeypatch.setattr(native_stack, "ensure_vcpkg", lambda stack, revision, repository: None)
    monkeypatch.setattr(native_stack, "run", lambda args, env=None, cwd=None: calls.append(args))
    native_stack.build_vcpkg(stack)
    (args,) = calls
    assert f"--overlay-ports={native_stack.PORT_OVERLAYS_COMMON}" in args


@pytest.mark.parametrize("triplet", TRIPLETS)
def test_vcpkg_identity_covers_the_common_port_overlays(tmp_path, monkeypatch, triplet):
    """Editing a common overlay changes the vcpkg stage's identity on every host."""
    stack = _stack(tmp_path, triplet, monkeypatch)
    monkeypatch.setattr(native_stack, "tool_version", lambda *args: args[0])
    digested = []
    monkeypatch.setattr(native_stack, "digest_paths", lambda *paths: digested.append(paths) or "")
    native_stack.vcpkg_identity(stack)
    assert any(native_stack.PORT_OVERLAYS_COMMON in paths for paths in digested)


def test_kubazip_overlay_pins_its_source_by_commit():
    """kubazip's archive is fetched by commit, not by a tag upstream can move.

    kuba--/zip rewrote its history on 2026-11-20 and re-created every tag on the rewritten
    commits; GitHub's archive of `v0.3.15` changed bytes, and the SHA512 in vcpkg 2026.10.16's
    port no longer matched it (microsoft/vcpkg#55200).
    """
    port = native_stack.PORT_OVERLAYS_COMMON / "kubazip"
    portfile = (port / "portfile.cmake").read_text()
    (ref,) = re.findall(r"^\s*REF\s+(\S+)\s*$", portfile, re.MULTILINE)
    assert re.fullmatch(r"[0-9a-f]{40}", ref), ref
    (sha512,) = re.findall(r"^\s*SHA512\s+(\S+)\s*$", portfile, re.MULTILINE)
    assert re.fullmatch(r"[0-9a-f]{128}", sha512), sha512
    manifest = json.loads((port / "vcpkg.json").read_text())
    assert manifest["name"] == "kubazip"
