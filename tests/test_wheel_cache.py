"""The native-cache key's libomp field: the conda record in the libomp prefix, on plain files."""

import importlib.util
import json
import sys
from pathlib import Path

import pytest

_SCRIPT = Path(__file__).parent.parent / "scripts" / "wheel_cache.py"
_spec = importlib.util.spec_from_file_location("wheel_cache", _SCRIPT)
assert _spec is not None and _spec.loader is not None
wheel_cache = importlib.util.module_from_spec(_spec)
sys.modules["wheel_cache"] = wheel_cache
_spec.loader.exec_module(wheel_cache)


def _record(prefix: Path, version: str, build: str) -> None:
    meta = prefix / "conda-meta"
    meta.mkdir(parents=True, exist_ok=True)
    record = {"name": "llvm-openmp", "version": version, "build": build}
    (meta / f"llvm-openmp-{version}-{build}.json").write_text(json.dumps(record))


def test_libomp_identity_names_the_conda_record(tmp_path: Path) -> None:
    _record(tmp_path, "23.1.5", "hdb3d66b_0")
    assert wheel_cache.libomp_identity(tmp_path) == "llvm-openmp 23.1.5 hdb3d66b_0"


def test_libomp_identity_refuses_a_prefix_without_the_record(tmp_path: Path) -> None:
    (tmp_path / "conda-meta").mkdir()
    with pytest.raises(wheel_cache.LibompRecordError, match="no llvm-openmp record"):
        wheel_cache.libomp_identity(tmp_path)


def test_libomp_identity_refuses_two_records(tmp_path: Path) -> None:
    _record(tmp_path, "23.1.5", "hdb3d66b_0")
    _record(tmp_path, "22.1.0", "h0000000_0")
    with pytest.raises(wheel_cache.LibompRecordError, match="2 llvm-openmp records"):
        wheel_cache.libomp_identity(tmp_path)


def test_native_artifact_name_carries_the_libomp(tmp_path: Path) -> None:
    environment = {
        "workspace": str(tmp_path),
        "platform": "macos-arm64",
        "image": "20261116.1",
        "compiler": "Apple clang version 17.0.0",
        "homebrew": "gcc 16.1.0",
        "libomp": "llvm-openmp 23.1.5 hdb3d66b_0",
    }
    name = wheel_cache.native_artifact_name(environment)
    assert name.endswith("gcc-16.1.0-llvm-openmp-23.1.5-hdb3d66b_0")
