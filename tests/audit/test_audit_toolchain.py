"""Contract tests of the fact key's toolchain (`scripts/audit_toolchain.py`): one canonical
serialisation, and an include-dir listing that changes exactly when a file a parse could
include appears. Needs no libclang.
"""

import dataclasses
from pathlib import Path

import pytest

from ._free import load_free

tc = load_free("audit_toolchain")


def _toolchain(**changes) -> "tc.Toolchain":
    fields = {
        "args": ("-x", "c++", "-I/abs/include"),
        "options": 5,
        "env": tc.digest_of("env"),
        "search_env": {"CPATH": None, "C_INCLUDE_PATH": "/opt/include"},
        "repo_dirs": {Path("/repo/src"): tc.digest_of("src"), Path("/repo/a"): tc.digest_of("a")},
        "extractors": tc.digest_of("extractors"),
    }
    return tc.Toolchain.build(**{**fields, **changes})


def test_mapping_order_is_not_part_of_the_key():
    a = _toolchain()
    b = _toolchain(
        search_env={"C_INCLUDE_PATH": "/opt/include", "CPATH": None},
        repo_dirs={Path("/repo/a"): tc.digest_of("a"), Path("/repo/src"): tc.digest_of("src")},
    )
    assert a == b
    assert a.canonical() == b.canonical()


@pytest.mark.parametrize(
    "change",
    [
        {"args": ("-x", "c++")},
        {"args": ("c++", "-x", "-I/abs/include")},  # argument order is search order
        {"options": 4},
        {"env": tc.digest_of("another env")},
        {"search_env": {"CPATH": "/x", "C_INCLUDE_PATH": "/opt/include"}},
        {"search_env": {"CPATH": None}},
        {"repo_dirs": {Path("/repo/src"): tc.digest_of("src")}},
        {"repo_dirs": {Path("/repo/src"): tc.digest_of("x"), Path("/repo/a"): tc.digest_of("a")}},
        {"extractors": tc.digest_of("another extractor")},
    ],
)
def test_every_field_is_part_of_the_key(change):
    assert _toolchain(**change).canonical() != _toolchain().canonical()


def test_an_unset_variable_differs_from_an_empty_one():
    unset = _toolchain(search_env={"CPATH": None})
    empty = _toolchain(search_env={"CPATH": ""})
    assert unset.canonical() != empty.canonical()


def test_the_serialisation_names_every_field():
    """A field added to `Toolchain` without reaching `canonical` would not invalidate."""
    text = _toolchain().canonical()
    assert all(f'"{f.name}"' in text for f in dataclasses.fields(tc.Toolchain))


def test_a_relative_repo_dir_is_refused():
    """A relative path names another directory from another working directory."""
    with pytest.raises(tc.RelativeRepoDirError):
        _toolchain(repo_dirs={Path("src"): tc.digest_of("src")})


def test_a_toolchain_is_hashable():
    assert len({_toolchain(), _toolchain(), _toolchain(options=4)}) == 2


@pytest.fixture
def include_dir(tmp_path):
    d = tmp_path / "include"
    (d / "pkg").mkdir(parents=True)
    (d / "a.h").write_text("int a;\n")
    (d / "pkg" / "b.hpp").write_text("int b;\n")
    return d


def _listing(directory: Path) -> str:
    tc.repo_dir_listing.cache_clear()  # cached per process: the audit lists a tree once
    return tc.repo_dir_listing(directory)


@pytest.mark.parametrize(
    "added",
    ["x.py", "pkg/x.pyi", "notes.md", "py.typed", "pkg/x_bindings.cpp", "pkg/__pycache__/x.pyc"],
)
def test_a_file_no_parse_reads_keeps_the_listing(include_dir, added):
    before = _listing(include_dir)
    (include_dir / added).parent.mkdir(parents=True, exist_ok=True)
    (include_dir / added).write_text("")
    assert _listing(include_dir) == before


@pytest.mark.parametrize("added", ["x.h", "pkg/y.hpp", "vector", "Eigen/Core", "z.ipp", "t.tcc"])
def test_a_file_a_parse_could_include_changes_the_listing(include_dir, added):
    """Also without a header suffix: `<vector>` and `<Eigen/Core>` have none, and a file of
    that name in a dir searched earlier shadows them."""
    before = _listing(include_dir)
    (include_dir / added).parent.mkdir(parents=True, exist_ok=True)
    (include_dir / added).write_text("")
    assert _listing(include_dir) != before


def test_a_removed_header_changes_the_listing(include_dir):
    before = _listing(include_dir)
    (include_dir / "a.h").unlink()
    assert _listing(include_dir) != before


def test_an_edit_keeps_the_listing(include_dir):
    """Content is the entry's business (each file the parse read is hashed), not the key's."""
    before = _listing(include_dir)
    (include_dir / "a.h").write_text("int a2;\n")
    assert _listing(include_dir) == before


def test_headers_and_the_source_are_listed_files(tmp_path):
    source = tmp_path / "a_bindings.cpp"
    tc.require_listed(source, [source, tmp_path / "a.h", Path("/env/include/c++/v1/vector")])


@pytest.mark.parametrize(
    "included", ["helper.cpp", "generated.py", "table.md", "pkg/__pycache__/x.h"]
)
def test_a_parse_that_read_an_unlisted_file_is_refused(tmp_path, included):
    """The listing skips these suffixes and directories because no parse reads them. One that
    does must not be cached: a shadowing file of that kind would go unseen."""
    source = tmp_path / "a_bindings.cpp"
    with pytest.raises(tc.UnlistedIncludeError, match=included):
        tc.require_listed(source, [source, tmp_path / "a.h", tmp_path / included])
