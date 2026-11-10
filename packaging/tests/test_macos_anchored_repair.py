"""Contracts for repairing the vcpkg wheel without conda-era YAML rewriting."""

import importlib.util
import sys
from pathlib import Path

import pytest

_SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "repair_macos_wheel.py"
_spec = importlib.util.spec_from_file_location("anchored_repair", _SCRIPT)
assert _spec is not None and _spec.loader is not None
repair = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = repair
_spec.loader.exec_module(repair)


def test_vcpkg_and_native_libraries_resolve_in_explicit_roots(tmp_path):
    native, vcpkg = tmp_path / "native", tmp_path / "vcpkg"
    native.mkdir()
    vcpkg.mkdir()
    dependency = vcpkg / "libfoo.dylib"
    dependency.touch()
    image = native / "libconsumer.dylib"
    assert (
        repair.resolve_anchored_source("@rpath/libfoo.dylib", image, (), (native, vcpkg))
        == dependency
    )


def test_loader_relative_dependency_is_resolved_from_original_image(tmp_path):
    dependency = tmp_path / "libfoo.dylib"
    dependency.touch()
    assert (
        repair.resolve_anchored_source(
            "@loader_path/libfoo.dylib", tmp_path / "libbar.dylib", (), (tmp_path,)
        )
        == dependency
    )


def test_foreign_absolute_dependency_is_rejected(tmp_path):
    root = tmp_path / "allowed"
    root.mkdir()
    foreign = tmp_path / "foreign.dylib"
    foreign.touch()
    with pytest.raises(repair.ForeignDylibError):
        repair.resolve_anchored_source(str(foreign), root / "libx.dylib", (), (root,))


def test_parser_reads_minimum_os_not_sdk():
    macho = repair.parse_load_commands(
        "Load command 1\n cmd LC_BUILD_VERSION\n platform 1\n minos 15.0\n sdk 26.0\n"
    )
    assert macho.minimum_os == (15, 0, 0)


def test_parser_reads_legacy_minimum_os():
    macho = repair.parse_load_commands(
        "Load command 1\n cmd LC_VERSION_MIN_MACOSX\n version 10.15\n sdk 13.0\n"
    )
    assert macho.minimum_os == (10, 15, 0)


def test_retag_updates_filename_and_wheel_metadata(tmp_path):
    info = tmp_path / "example-1.dist-info"
    info.mkdir()
    (info / "WHEEL").write_text("Wheel-Version: 1.0\nTag: cp312-abi3-macosx_12_0_arm64\n")
    name = repair.retag_macos(tmp_path, "example-1-cp312-abi3-macosx_12_0_arm64.whl", (15, 0, 0))
    assert name == "example-1-cp312-abi3-macosx_15_0_arm64.whl"
    assert (info / "WHEEL").read_text().endswith("Tag: cp312-abi3-macosx_15_0_arm64\n")


def test_retag_never_lowers_input_floor(tmp_path):
    info = tmp_path / "example-1.dist-info"
    info.mkdir()
    (info / "WHEEL").write_text("Tag: cp312-abi3-macosx_15_0_x86_64\n")
    name = "example-1-cp312-abi3-macosx_15_0_x86_64.whl"
    assert repair.retag_macos(tmp_path, name, (12, 0, 0)) == name


def test_existing_bundle_cannot_mask_a_different_source_library(tmp_path, monkeypatch):
    root = tmp_path / "wheel"
    bundle = root / repair.BUNDLE
    bundle.mkdir(parents=True)
    (bundle / "libsame.dylib").write_bytes(b"existing")
    external = tmp_path / "native"
    external.mkdir()
    foreign = external / "libsame.dylib"
    foreign.write_bytes(b"different")
    extension = root / repair.PACKAGE / "_ext.so"
    extension.touch()

    def load_commands(image):
        return repair.MachO(None, (str(foreign),) if image == extension else (), ())

    monkeypatch.setattr(repair, "read_macho", load_commands)
    monkeypatch.setattr(repair.subprocess, "run", lambda *args, **kwargs: None)
    with pytest.raises(repair.DuplicateDylibError):
        repair.bundle_anchored_closure(root, (external,))
