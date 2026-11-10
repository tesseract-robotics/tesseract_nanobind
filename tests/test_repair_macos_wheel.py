"""The macOS wheel repair's decisions, on otool text and plain files: no Mach-O, runs anywhere."""

import base64
import hashlib
import importlib.util
import sys
from pathlib import Path

import pytest

_SCRIPT = Path(__file__).parent.parent / "scripts" / "repair_macos_wheel.py"
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
