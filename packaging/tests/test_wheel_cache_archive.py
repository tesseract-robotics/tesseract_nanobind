"""Native bundles preserve install layout and reject mismatched or unsafe inputs."""

import importlib.util
import io
import tarfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]


def archive_module():
    spec = importlib.util.spec_from_file_location(
        "wheel_cache_archive", ROOT / "scripts/wheel_cache_archive.py"
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_native_round_trip_preserves_library_symlink(tmp_path):
    archive = archive_module()
    source = tmp_path / "source"
    library = source / "ws/install/lib"
    library.mkdir(parents=True)
    (source / "vcpkg/installed/include").mkdir(parents=True)
    (library / "libexample.dylib.1").write_bytes(b"compiled library fixture")
    (library / "libexample.dylib").symlink_to("libexample.dylib.1")
    output = tmp_path / "artifact"
    environment = {
        "workspace": "fixed-ci-prefix",
        "image": "2026.11",
        "platform": "macos-arm64",
        "homebrew": "",
    }
    archive.pack_native(source, output, environment)
    target = tmp_path / "target"
    archive.unpack_native(output, target, environment)
    link = target / "ws/install/lib/libexample.dylib"
    assert link.is_symlink()
    assert link.read_bytes() == b"compiled library fixture"


def test_layout_mismatch_fails_before_extracting(tmp_path):
    archive = archive_module()
    (tmp_path / "native-environment.json").write_text('{"workspace": "old-prefix"}')
    with pytest.raises(archive.NativeEnvironmentMismatchError):
        archive.unpack_native(tmp_path, tmp_path / "target", {"workspace": "new-prefix"})
    assert not (tmp_path / "target").exists()


def test_missing_native_install_is_not_archived(tmp_path):
    archive = archive_module()
    with pytest.raises(archive.MissingNativeInstallError):
        archive.pack_native(tmp_path, tmp_path / "artifact", {})


def test_archive_cannot_write_outside_restore_directory(tmp_path):
    archive = archive_module()
    bundle = tmp_path / "bad.tar"
    with tarfile.open(bundle, "w") as stream:
        member = tarfile.TarInfo("../escaped")
        member.size = 1
        stream.addfile(member, io.BytesIO(b"x"))
    with pytest.raises(tarfile.OutsideDestinationError):
        archive.extract_bundle(bundle, tmp_path / "restore")
    assert not (tmp_path / "escaped").exists()
