"""The macOS wheel repair's decisions, on otool text and plain files: no Mach-O, runs anywhere."""

import base64
import hashlib
import importlib.util
import io
import sys
import zipfile
from pathlib import Path

import pytest

_SCRIPT = Path(__file__).parent.parent / "scripts" / "repair_macos_wheel.py"
# The repair reads the deployment target from scripts/native_stack.py, as it does run as a script.
sys.path.insert(0, str(_SCRIPT.parent))
_spec = importlib.util.spec_from_file_location("repair_macos_wheel", _SCRIPT)
assert _spec is not None and _spec.loader is not None
repair = importlib.util.module_from_spec(_spec)
sys.modules["repair_macos_wheel"] = repair
_spec.loader.exec_module(repair)

OTOOL_L = """\
pkg/_ext.abi3.so:
Load command 3
          cmd LC_ID_DYLIB
      cmdsize 48
         name @rpath/libself.dylib (offset 24)
Load command 4
          cmd LC_LOAD_DYLIB
      cmdsize 56
         name @rpath/libfoo.1.dylib (offset 24)
Load command 5
          cmd LC_LOAD_WEAK_DYLIB
      cmdsize 56
         name /usr/lib/libSystem.B.dylib (offset 24)
Load command 6
          cmd LC_RPATH
      cmdsize 32
         path /build/env/lib (offset 12)
Load command 7
          cmd LC_RPATH
      cmdsize 32
         path @loader_path/ (offset 12)
"""


def test_parse_reads_id_deps_and_rpaths():
    macho = repair.parse_load_commands(OTOOL_L)
    assert macho.install_id == "@rpath/libself.dylib"
    assert macho.deps == ("@rpath/libfoo.1.dylib", "/usr/lib/libSystem.B.dylib")
    assert macho.rpaths == ("/build/env/lib", "@loader_path/")


def test_parse_of_a_module_has_no_id():
    assert repair.parse_load_commands("x.so:\nLoad command 0\n").install_id is None


@pytest.mark.parametrize(
    ("dep", "system"),
    [
        ("/usr/lib/libc++.1.dylib", True),
        ("/System/Library/Frameworks/Accelerate.framework/Accelerate", True),
        ("@rpath/libfoo.dylib", False),
        ("/opt/homebrew/lib/libfoo.dylib", False),
    ],
)
def test_system_libraries_stay(dep, system):
    assert repair.is_system(dep) is system


def test_reference_from_an_extension_climbs_to_the_bundle(tmp_path):
    bundle = tmp_path / "pkg" / ".dylibs"
    image = tmp_path / "pkg" / "sub" / "_ext.abi3.so"
    assert repair.reference_to(bundle / "libx.dylib", image) == "@loader_path/../.dylibs/libx.dylib"


def test_reference_inside_the_bundle_is_a_sibling(tmp_path):
    bundle = tmp_path / "pkg" / ".dylibs"
    assert (
        repair.reference_to(bundle / "libx.dylib", bundle / "liby.dylib")
        == "@loader_path/libx.dylib"
    )


def test_soname_symlink_resolves_to_one_real_file(tmp_path):
    """Two names for one library load twice, and singleton state crashes (#48)."""
    real = tmp_path / "libompl.1.7.0.dylib"
    real.touch()
    (tmp_path / "libompl.18.dylib").symlink_to(real.name)
    assert repair.resolve_source("@rpath/libompl.18.dylib", tmp_path) == real


def test_a_dependency_missing_from_the_env_raises(tmp_path):
    with pytest.raises(repair.MissingDylibError, match="libgone.dylib"):
        repair.resolve_source("@rpath/libgone.dylib", tmp_path)


def test_an_absolute_path_outside_the_env_raises(tmp_path):
    with pytest.raises(repair.ForeignDylibError, match="/opt/homebrew"):
        repair.resolve_source("/opt/homebrew/lib/libfoo.dylib", tmp_path)


def test_an_absolute_path_inside_the_env_resolves(tmp_path):
    lib = tmp_path / "libfoo.dylib"
    lib.touch()
    assert repair.resolve_source(str(lib), tmp_path) == lib


def test_openmp_rpaths_try_the_env_first_then_the_bundled_copy(tmp_path):
    """conda and pixi then share the env's runtime (#136); a plain env loads the bundled copy."""
    bundle = tmp_path / "tesseract_robotics" / ".dylibs"
    image = tmp_path / "tesseract_robotics" / "sub" / "_ext.abi3.so"
    assert repair.openmp_rpaths(image, tmp_path, bundle) == (
        "@loader_path/../../../..",
        "@loader_path/../.dylibs",
    )


def test_openmp_rpaths_from_inside_the_bundle(tmp_path):
    bundle = tmp_path / "tesseract_robotics" / ".dylibs"
    assert repair.openmp_rpaths(bundle / "libx.dylib", tmp_path, bundle) == (
        "@loader_path/../../../..",
        "@loader_path",
    )


def test_record_lists_every_file_with_its_hash(tmp_path):
    info = tmp_path / "pkg-1.dist-info"
    info.mkdir()
    (info / "RECORD").write_text("stale,sha256=x,1\n")
    (tmp_path / "a.txt").write_bytes(b"hello")
    repair.write_record(tmp_path)
    digest = base64.urlsafe_b64encode(hashlib.sha256(b"hello").digest()).rstrip(b"=").decode()
    assert (info / "RECORD").read_text().splitlines() == [
        f"a.txt,sha256={digest},5",
        "pkg-1.dist-info/RECORD,,",
    ]


def test_plugin_directory_becomes_the_placeholder():
    source = "search_libraries:\n  - /usr/local/lib\n"
    assert repair.patch_plugin_path(source) == 'search_libraries:\n  - "@PLUGIN_PATH@"\n'


def test_hardcoded_search_paths_are_removed():
    source = "plugins:\n  search_paths:\n    - /usr/local/lib\n  search_libraries:\n    - libfoo\n"
    assert repair.strip_search_paths(source) == "plugins:\n  search_libraries:\n    - libfoo\n"


def test_a_yaml_without_search_paths_keeps_its_absolute_list_items():
    source = "points:\n  - /keep/this\n"
    assert repair.strip_search_paths(source) == source


def test_verify_reports_every_stray_reference(tmp_path):
    bundle = tmp_path / "pkg" / ".dylibs"
    bundle.mkdir(parents=True)
    image = bundle / "libx.dylib"
    image.touch()
    stray = repair.MachO(
        None, ("/opt/homebrew/lib/libfoo.dylib", "@rpath/libbar.dylib"), ("/build/env/lib",)
    )
    problems = repair.problems_in(image, stray, tmp_path, bundle)
    assert len(problems) == 3
    assert any("/opt/homebrew" in p for p in problems)
    assert any("libbar" in p for p in problems)
    assert any("/build/env/lib" in p for p in problems)


def test_verify_accepts_a_repaired_image(tmp_path):
    bundle = tmp_path / "pkg" / ".dylibs"
    bundle.mkdir(parents=True)
    image = bundle / "libx.dylib"
    (bundle / "liby.dylib").touch()
    (bundle / repair.OPENMP).touch()
    clean = repair.MachO(
        None,
        ("@loader_path/liby.dylib", "/usr/lib/libSystem.B.dylib", f"@rpath/{repair.OPENMP}"),
        ("@loader_path/../../../..", "@loader_path"),
    )
    assert repair.problems_in(image, clean, tmp_path, bundle) == []


def test_verify_catches_a_reference_to_a_file_not_in_the_wheel(tmp_path):
    bundle = tmp_path / "pkg" / ".dylibs"
    bundle.mkdir(parents=True)
    broken = repair.MachO(None, ("@loader_path/libgone.dylib",), ())
    problems = repair.problems_in(bundle / "libx.dylib", broken, tmp_path, bundle)
    assert len(problems) == 1
    assert "libgone" in problems[0]


def test_the_declared_target_is_the_native_stacks():
    import native_stack

    assert repair.DEPLOYMENT_TARGET == repair.parse_os_version(native_stack.OSX_DEPLOYMENT_TARGET)


def test_parse_reads_the_minimum_os():
    otool = "x.dylib:\nLoad command 9\n      cmd LC_BUILD_VERSION\n  platform 1\n    minos 15.3\n"
    assert repair.parse_load_commands(otool).minimum_os == (15, 3, 0)


def test_a_binary_above_the_deployment_target_is_refused_naming_it():
    minimums = {
        "tesseract_robotics/.dylibs/libosqp.dylib": (15, 3, 0),
        "tesseract_robotics/.dylibs/libpiqp.dylib": (15, 3, 0),
        "tesseract_robotics/.dylibs/libtesseract_common.dylib": (12, 0, 0),
    }
    with pytest.raises(repair.DeploymentTargetExceededError) as caught:
        repair.check_deployment_target(minimums, (12, 0, 0))
    message = str(caught.value)
    assert "libosqp.dylib: 15.3" in message
    assert "libpiqp.dylib: 15.3" in message
    assert "libtesseract_common" not in message
    assert "12.0" in message


def test_binaries_at_or_below_the_deployment_target_pass():
    repair.check_deployment_target({"a.dylib": (12, 0, 0), "b.so": (11, 0, 0)}, (12, 0, 0))


def _wheel_metadata(root: Path, tag: str) -> Path:
    info = root / "pkg-1.dist-info"
    info.mkdir(parents=True)
    wheel = info / "WHEEL"
    wheel.write_bytes(f"Wheel-Version: 1.0\nTag: {tag}\n".encode())
    return wheel


# What Windows text mode does to a file opened without newline= or encoding=.
_WINDOWS_LINESEP = "\r\n"
_WINDOWS_LOCALE_ENCODING = "cp1252"


@pytest.fixture
def windows_text_io(monkeypatch):
    """Text-mode `open` as on Windows: "\\n" written as "\\r\\n", the locale codepage by default.

    pathlib opens every file through `io.open`. The repair's output must not depend on the
    platform it runs on; this makes a test of that fail on macOS and Linux too, not only on
    the Windows CI leg.
    """
    native_open = io.open

    def windows_open(file, mode="r", buffering=-1, encoding=None, errors=None, newline=None):
        if "b" not in mode:
            if encoding in (None, "locale"):
                encoding = _WINDOWS_LOCALE_ENCODING
            if newline is None and any(flag in mode for flag in "wax+"):
                newline = _WINDOWS_LINESEP
        return native_open(file, mode, buffering, encoding, errors, newline)

    monkeypatch.setattr(io, "open", windows_open)


def test_retagged_metadata_is_byte_identical_on_every_platform(tmp_path, windows_text_io):
    metadata = _wheel_metadata(tmp_path, "cp312-abi3-macosx_15_0_arm64")
    repair.retag_macos(tmp_path, "pkg-1-cp312-abi3-macosx_15_0_arm64.whl", (12, 0, 0))
    assert metadata.read_bytes() == b"Wheel-Version: 1.0\nTag: cp312-abi3-macosx_12_0_arm64\n"


def test_retag_keeps_the_metadata_line_endings_it_was_given(tmp_path):
    info = tmp_path / "pkg-1.dist-info"
    info.mkdir()
    metadata = info / "WHEEL"
    metadata.write_bytes(b"Wheel-Version: 1.0\r\nTag: cp312-abi3-macosx_15_0_arm64\r\n")
    repair.retag_macos(tmp_path, "pkg-1-cp312-abi3-macosx_15_0_arm64.whl", (12, 0, 0))
    assert metadata.read_bytes() == b"Wheel-Version: 1.0\r\nTag: cp312-abi3-macosx_12_0_arm64\r\n"


def test_record_is_byte_identical_on_every_platform(tmp_path, windows_text_io):
    info = tmp_path / "pkg-1.dist-info"
    info.mkdir()
    (info / "RECORD").write_bytes(b"")
    (tmp_path / "a.txt").write_bytes(b"hello")
    repair.write_record(tmp_path)
    digest = base64.urlsafe_b64encode(hashlib.sha256(b"hello").digest()).rstrip(b"=").decode()
    assert (info / "RECORD").read_bytes() == (
        f"a.txt,sha256={digest},5\npkg-1.dist-info/RECORD,,\n".encode()
    )


def test_patched_yaml_keeps_its_bytes_on_every_platform(tmp_path, windows_text_io):
    """Line endings as given, and UTF-8 text that is no valid cp1252 ("Á" is 0xC3 0x81)."""
    config = tmp_path / repair.PACKAGE / "data" / "task_composer_config"
    support = tmp_path / repair.PACKAGE / "data" / "tesseract" / "support" / "robot"
    config.mkdir(parents=True)
    support.mkdir(parents=True)
    (config / "c.yaml").write_bytes("# Á\nsearch_libraries:\n  - /usr/local/lib\n".encode())
    (support / "r.yaml").write_bytes(
        "# Á\r\nplugins:\r\n  search_paths:\r\n    - /usr/local/lib\r\n  x: 1\r\n".encode()
    )
    repair.patch_yaml(tmp_path)
    assert (config / "c.yaml").read_bytes() == (
        '# Á\nsearch_libraries:\n  - "@PLUGIN_PATH@"\n'.encode()
    )
    assert (support / "r.yaml").read_bytes() == "# Á\r\nplugins:\r\n  x: 1\r\n".encode()


def test_repaired_wheel_record_hashes_the_metadata_bytes_it_packs(tmp_path, no_otool):
    lib = tmp_path / "lib"
    lib.mkdir()
    repaired = repair.repair_anchored(_zip_wheel(tmp_path), (lib,), tmp_path / "out")
    with zipfile.ZipFile(repaired) as archive:
        wheel = archive.read("tesseract_robotics-1.dist-info/WHEEL")
        record = archive.read("tesseract_robotics-1.dist-info/RECORD").decode()
    digest = base64.urlsafe_b64encode(hashlib.sha256(wheel).digest()).rstrip(b"=").decode()
    assert f"tesseract_robotics-1.dist-info/WHEEL,sha256={digest},{len(wheel)}\n" in record
    assert b"\r" not in wheel


def test_the_wheel_is_tagged_with_the_deployment_target(tmp_path):
    """Not the host's version scikit-build-core may have written, nor the closure's maximum."""
    metadata = _wheel_metadata(tmp_path, "cp312-abi3-macosx_15_0_arm64")
    name = repair.retag_macos(tmp_path, "pkg-1-cp312-abi3-macosx_15_0_arm64.whl", (12, 0, 0))
    assert name == "pkg-1-cp312-abi3-macosx_12_0_arm64.whl"
    assert "Tag: cp312-abi3-macosx_12_0_arm64\n" in metadata.read_text()


def test_a_tag_pip_never_matches_is_refused(tmp_path):
    """pip matches macosx_<major>_0 only, from macOS 11 on: macosx_15_3 installs nowhere."""
    _wheel_metadata(tmp_path, "cp312-abi3-macosx_12_0_arm64")
    with pytest.raises(repair.UninstallableTagError, match="15.3"):
        repair.retag_macos(tmp_path, "pkg-1-cp312-abi3-macosx_12_0_arm64.whl", (15, 3, 0))


_WHEEL_NAME = "tesseract_robotics-1-cp312-abi3-macosx_14_0_arm64.whl"


def _zip_wheel(tmp_path: Path) -> Path:
    """A wheel with only its metadata: the Mach-O stages are replaced in these tests."""
    wheel = tmp_path / _WHEEL_NAME
    with zipfile.ZipFile(wheel, "w") as archive:
        archive.writestr(
            "tesseract_robotics-1.dist-info/WHEEL",
            "Wheel-Version: 1.0\nTag: cp312-abi3-macosx_14_0_arm64\n",
        )
        archive.writestr("tesseract_robotics-1.dist-info/RECORD", "")
    return wheel


@pytest.fixture
def no_otool(monkeypatch):
    """Skip the Mach-O rewriting and verification; the tag decisions read no binary."""
    monkeypatch.setattr(repair, "bundle_anchored_closure", lambda root, lib_dirs: {})
    monkeypatch.setattr(repair, "verify", lambda root: None)


def test_anchored_repair_refuses_a_closure_above_the_deployment_target(
    tmp_path, no_otool, monkeypatch
):
    monkeypatch.setattr(
        repair,
        "bundle_anchored_closure",
        lambda root, lib_dirs: {"tesseract_robotics/.dylibs/libqpOASES.3.2.dylib": (15, 3, 0)},
    )
    lib = tmp_path / "lib"
    lib.mkdir()
    with pytest.raises(repair.DeploymentTargetExceededError, match="libqpOASES"):
        repair.repair_anchored(_zip_wheel(tmp_path), (lib,), tmp_path / "out")
    assert not list((tmp_path / "out").glob("*.whl"))


def test_anchored_repair_names_the_wheel_for_the_deployment_target(tmp_path, no_otool):
    lib = tmp_path / "lib"
    lib.mkdir()
    repaired = repair.repair_anchored(_zip_wheel(tmp_path), (lib,), tmp_path / "out")
    major, minor, _patch = repair.DEPLOYMENT_TARGET
    assert repaired.name == f"tesseract_robotics-1-cp312-abi3-macosx_{major}_{minor}_arm64.whl"
    with zipfile.ZipFile(repaired) as archive:
        metadata = archive.read("tesseract_robotics-1.dist-info/WHEEL").decode()
    assert f"Tag: cp312-abi3-macosx_{major}_{minor}_arm64\n" in metadata
