"""The example-coverage gate names every example the test session did not execute (#445).

The verdict is tested as a pure function on synthetic modules; one pytester run checks the
plugin end to end under xdist, where the executed sets come from the workers.
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest

from .example_coverage import (
    ExampleKind,
    ExampleModule,
    UntestedExampleError,
    assert_all_executed,
    discover,
)

ENTRY = ExampleModule("pkg.entry_example", Path("/pkg/entry_example.py"), ExampleKind.ENTRY)
HELPER = ExampleModule("pkg.helper", Path("/pkg/helper.py"), ExampleKind.HELPER)


def test_entry_example_whose_main_ran_passes():
    assert_all_executed([ENTRY], {(ENTRY.path, "main")})


def test_entry_example_whose_run_ran_passes():
    assert_all_executed([ENTRY], {(ENTRY.path, "run")})


def test_entry_example_imported_only_is_named():
    with pytest.raises(UntestedExampleError, match="entry_example.py"):
        assert_all_executed([ENTRY], {(ENTRY.path, "<module>")})


def test_entry_example_with_only_a_helper_function_run_is_named():
    with pytest.raises(UntestedExampleError, match="entry_example.py"):
        assert_all_executed([ENTRY], {(ENTRY.path, "build_scene")})


def test_helper_with_any_function_run_passes():
    assert_all_executed([HELPER], {(HELPER.path, "make_obstacle")})


def test_helper_imported_only_is_named():
    with pytest.raises(UntestedExampleError, match="helper.py"):
        assert_all_executed([HELPER], {(HELPER.path, "<module>")})


def test_every_violation_is_named():
    with pytest.raises(UntestedExampleError) as info:
        assert_all_executed([ENTRY, HELPER], set())
    assert "entry_example.py" in str(info.value)
    assert "helper.py" in str(info.value)


def test_discovery_finds_examples_helpers_and_subpackages():
    found = {m.name: m.kind for m in discover("tesseract_robotics.examples")}
    assert found["tesseract_robotics.examples.freespace_ompl_example"] is ExampleKind.ENTRY
    assert found["tesseract_robotics.examples.online_planning_scene"] is ExampleKind.HELPER
    assert (
        found["tesseract_robotics.examples.lowlevel.scene_graph_c_api_example"] is ExampleKind.ENTRY
    )
    assert found["tesseract_robotics.examples.lowlevel.example_support"] is ExampleKind.HELPER
    # A package `__init__` holds no example; it is not enrolled.
    assert "tesseract_robotics.examples.lowlevel" not in found


FAKE_PACKAGE = {
    "fakeex/__init__.py": "",
    "fakeex/covered_example.py": "def main(argv=None):\n    return 0\n",
    "fakeex/orphan_example.py": "def main(argv=None):\n    return 0\n",
}
FAKE_TESTS = {
    "fake_tests/__init__.py": "",
    "fake_tests/test_covered.py": (
        "from fakeex import covered_example\n\n"
        "def test_covered():\n    assert covered_example.main() == 0\n"
    ),
    "fake_tests/test_other.py": "def test_other():\n    pass\n",
}
ORPHAN_TEST = {
    "fake_tests/test_orphan.py": (
        "from fakeex import orphan_example\n\n"
        "def test_orphan():\n    assert orphan_example.main() == 0\n"
    ),
}


REPO_ROOT = Path(__file__).resolve().parents[2]


def _fake_project(
    pytester: pytest.Pytester, monkeypatch: pytest.MonkeyPatch, files: dict[str, str]
) -> None:
    for name, text in files.items():
        path = pytester.path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
    pytester.makeini(
        "[pytest]\nexample_coverage_package = fakeex\nexample_coverage_testdir = fake_tests\n"
    )
    # The subprocess imports the plugin from this repository and `fakeex` from its own root.
    monkeypatch.setenv("PYTHONPATH", f"{REPO_ROOT}")


def _run(pytester: pytest.Pytester, *args: str) -> pytest.RunResult:
    return pytester.runpytest_subprocess("-p", "tests.examples.example_coverage", *args)


@pytest.mark.parametrize("workers", [[], ["-n", "2"]], ids=["serial", "xdist"])
def test_session_fails_naming_an_unexecuted_example(pytester, monkeypatch, workers):
    _fake_project(pytester, monkeypatch, FAKE_PACKAGE | FAKE_TESTS)
    result = _run(pytester, "fake_tests", *workers)
    assert result.ret == pytest.ExitCode.TESTS_FAILED
    result.stdout.fnmatch_lines(["*orphan_example.py*"])
    assert "covered_example.py" not in result.stdout.str()


@pytest.mark.parametrize("workers", [[], ["-n", "2"]], ids=["serial", "xdist"])
def test_session_passes_when_every_example_ran(pytester, monkeypatch, workers):
    _fake_project(pytester, monkeypatch, FAKE_PACKAGE | FAKE_TESTS | ORPHAN_TEST)
    result = _run(pytester, "fake_tests", *workers)
    assert result.ret == pytest.ExitCode.OK


@pytest.mark.parametrize(
    "selection",
    [["fake_tests/test_other.py"], ["fake_tests", "-k", "other"]],
    ids=["one-file", "deselected"],
)
def test_partial_session_does_not_judge(pytester, monkeypatch, selection):
    _fake_project(pytester, monkeypatch, FAKE_PACKAGE | FAKE_TESTS)
    result = _run(pytester, *selection)
    assert result.ret == pytest.ExitCode.OK


def test_examples_imported_through_a_symlink_are_recorded(pytester, monkeypatch):
    """A site-packages reached through a symlink: `co_filename` is the unresolved path."""
    _fake_project(pytester, monkeypatch, FAKE_TESTS | ORPHAN_TEST)
    real = pytester.path / "real_site"
    for name, text in FAKE_PACKAGE.items():
        path = real / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
    (pytester.path / "linked_site").symlink_to(real, target_is_directory=True)
    monkeypatch.setenv("PYTHONPATH", f"{pytester.path / 'linked_site'}{os.pathsep}{REPO_ROOT}")
    result = _run(pytester, "fake_tests")
    assert result.ret == pytest.ExitCode.OK, result.stdout.str()
