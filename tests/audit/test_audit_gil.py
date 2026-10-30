"""Contract tests of scripts/audit_gil.py (#124).

Run in the `audit` env only (libclang): `pixi run -e audit audit-test`.
"""

import os
import shutil
import subprocess
import sysconfig
from pathlib import Path

import nanobind
import pytest

from tests.audit._load import load

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
FIXTURE = Path(__file__).resolve().parent / "fixtures" / "gil_bindings.cpp"


# `load` caches in sys.modules: a second exec of audit_ownership would give the audit
# scripts loaded before it other exception classes than the tests collected after it.
entry_points = load("gil_entry_points")  # audit_gil imports it
gil = load("audit_gil")

_own = load("audit_ownership")


def _resource_dir() -> str:
    """clang-22's builtin headers in the audit env (as audit_bindings uses); else the system clang's."""
    conda = os.environ.get("CONDA_PREFIX")
    if conda:
        found = sorted((Path(conda) / "lib" / "clang").glob("*"))
        if found:
            return str(found[-1])
    clang = shutil.which("clang")
    if clang is None:
        pytest.skip("no clang resource dir: run in the audit env")
    return subprocess.check_output([clang, "-print-resource-dir"], text=True).strip()


def _args():
    return [
        "-x",
        "c++",
        "-std=c++17",
        "-fsyntax-only",
        "-resource-dir",
        _resource_dir(),
        f"-I{nanobind.include_dir()}",
        f"-I{sysconfig.get_paths()['include']}",
    ]


@pytest.fixture(scope="module")
def fixture_releases():
    return gil.Scanner(gil.parse(FIXTURE, _args()), "fixture").scan()


def R(scope, name):
    return gil.Release("fixture", scope, name)


def test_fixture_finds_every_release_form(fixture_releases):
    assert set(fixture_releases) == {
        R("Planner", "solve"),  # call_guard
        R("Planner", "solveInBody"),  # gil_scoped_release in the bound lambda
        R("Planner", "solveTwice"),  # in a local lambda bound by name
        R("Planner", "bind_equality"),  # a binding helper given the call_guard
        R("Manager", "contactTest"),  # in a function template the lambda calls
        R("module", "segment"),  # in a free function bound by pointer
        R("module", "loadInt"),  # call_guard in a helper template, named at its call site
        R("module", "loadLong"),
    }


@pytest.mark.parametrize(
    "name",
    [
        "name",  # a getter
        "callback",  # gil_scoped_acquire is not a release
        "localStruct",  # a local struct's method is not the binding's body
        "__init__",
    ],
)
def test_fixture_non_releases_are_not_reported(fixture_releases, name):
    assert R("Planner", name) not in fixture_releases


def test_fixture_plain_helper_is_not_reported(fixture_releases):
    assert R("module", "loadPlain") not in fixture_releases


def test_fixture_location_is_the_def_line(fixture_releases):
    lines = FIXTURE.read_text().splitlines()
    file, line = fixture_releases[R("Planner", "solveInBody")].split(":")
    assert file == FIXTURE.name
    assert '"solveInBody"' in lines[int(line) - 1]


def test_compare_reports_both_directions():
    found = {R("Planner", "solve"): "x.cpp:1", R("Planner", "extra"): "x.cpp:2"}
    listed = {R("Planner", "solve"): "why", R("Planner", "gone"): "why"}
    problems = gil.compare(found, listed, ["fixture"])
    assert len(problems) == 2
    assert any("x.cpp:2" in p and "Planner.extra" in p and "not in" in p for p in problems)
    assert any("Planner.gone" in p and "no longer releases" in p for p in problems)


def test_compare_ignores_unscanned_modules():
    listed = {gil.Release("other", "C", "m"): "why"}
    assert gil.compare({}, listed, ["fixture"]) == []


def test_entry_needs_a_reason(tmp_path):
    path = tmp_path / "nogil.yaml"
    path.write_text("mod:\n  C:\n    m: ''\n")
    with pytest.raises(entry_points.EntryPointsError, match="needs a reason"):
        gil.load_entry_points(path)


def test_committed_list_loads_and_names_real_modules():
    listed = gil.load_entry_points(gil.ENTRY_POINTS)
    assert listed
    assert {r.module for r in listed} <= set(_own.binding_modules())


def test_a_release_whose_class_cannot_be_resolved_fails_loudly(tmp_path):
    # A literal name on a template-dependent receiver: the Python class is unknowable here.
    source = """
#include <nanobind/nanobind.h>
namespace nb = nanobind;
template <typename Class>
void def_solve(Class& cls) {
    cls.def("solve", [](int n) { return n; }, nb::call_guard<nb::gil_scoped_release>());
}
NB_MODULE(_unresolved, m) {}
"""
    cpp = tmp_path / "unresolved.cpp"
    cpp.write_text(source)
    with pytest.raises(gil.UnresolvedScopeError, match="solve"):
        gil.Scanner(gil.parse(cpp, _args()), "fixture").scan()
