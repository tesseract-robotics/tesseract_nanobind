"""Contract tests of `tests/examples/api_recorder.py`, the pytest plugin that records example calls.

Each test names the mechanism it proves. The recorder wraps nanobind callables
in place; which entry points a plain `setattr` wrapper intercepts was probed on
2026-10-10, and only ordinary methods were confirmed. `__init__` (nanobind's
type vectorcall bypasses the class dict), the dunder slots, properties, static
methods and module functions bound by `from m import f` each needed their own
mechanism, and each test below was red before the recorder had it
(`.scratch/phaseD-f0-recorder-red.log`).

The fixture example (`fixtures/examples/fixture_example.py`) runs in an inner
pytest session under `-p tests.examples.api_recorder`, in a subprocess, so the
instrumentation never touches this process.
"""

import json
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
FIXTURES = Path(__file__).resolve().parent / "fixtures"
EXAMPLE_ROOT = FIXTURES / "examples"
DRIVER = FIXTURES / "driver" / "drive_fixture_example.py"
TRAMPOLINE_SUBCLASS = FIXTURES / "driver" / "trampoline_subclass.py"
# Inner sessions import the whole extension package set; a generous bound.
SUBPROCESS_TIMEOUT_S = 300


def run_recorded_session(
    record_dir: Path, example_root: Path = EXAMPLE_ROOT, *extra: str
) -> list[dict]:
    """Run the fixture driver under the plugin; return every recorded call of every worker."""
    cmd = [
        sys.executable,
        "-m",
        "pytest",
        "-p",
        "tests.examples.api_recorder",
        "--api-record-dir",
        str(record_dir),
        "--api-record-example-root",
        str(example_root),
        "-o",
        "python_files=drive_*.py",
        "-o",
        "addopts=",
        "-p",
        "no:cacheprovider",
        "-q",
        *extra,
        str(DRIVER),
    ]
    proc = subprocess.run(
        cmd, cwd=REPO_ROOT, capture_output=True, text=True, timeout=SUBPROCESS_TIMEOUT_S
    )
    assert proc.returncode == 0, proc.stdout + proc.stderr
    calls = []
    for path in sorted(record_dir.glob("*.json")):
        calls += json.loads(path.read_text())["calls"]
    return calls


@pytest.fixture(scope="module")
def calls(tmp_path_factory):
    return run_recorded_session(tmp_path_factory.mktemp("recordings"))


def _recorded(calls, qualname):
    return {(c["kind"], c["index"]) for c in calls if c["qualname"] == qualname}


def test_two_overloads_of_a_method_are_both_recorded(calls):
    """Ordinary methods (setattr wrapper; the probed mechanism), each with its own overload."""
    assert _recorded(calls, "AllowedCollisionMatrix.removeAllowedCollision") == {
        ("method", 0),
        ("method", 1),
    }


def test_init_overloads_are_recorded(calls):
    """`Cls(...)`: the type's vectorcall is cleared so construction goes through the dict `__init__`."""
    assert _recorded(calls, "AllowedCollisionMatrix.__init__") == {("method", 0), ("method", 1)}
    assert _recorded(calls, "JointTrajectory.__init__") == {("method", 1)}


@pytest.mark.parametrize(
    "qualname",
    [
        "JointTrajectory.__getitem__",
        "JointTrajectory.__setitem__",
        "AllowedCollisionMatrix.__eq__",
        "AllowedCollisionMatrix.__ne__",
        "ContactAllowedValidator.__call__",
    ],
)
def test_dunder_slots_are_recorded(calls, qualname):
    """Operators and protocols reach the wrapper through the type slot, not an attribute call."""
    assert _recorded(calls, qualname) == {("method", 0)}


def test_property_access_is_recorded(calls):
    """A property is replaced by one whose getter and setter are wrapped."""
    assert _recorded(calls, "JointState.joint_names") == {("property", None)}


def test_static_method_is_recorded(calls):
    """A static method is a bare `nb_func` in the class dict; its wrapper must stay static."""
    assert _recorded(calls, "Isometry3d.Identity") == {("staticmethod", 0)}


def test_module_function_bound_by_from_import_is_recorded(calls):
    """`from tesseract_robotics.x import f` binds the package attribute, which must be the wrapper."""
    assert _recorded(calls, "makeOrderedLinkPair") == {("function", 0)}


def test_call_from_a_non_example_frame_is_not_recorded(calls):
    """Only the immediate caller counts: library code an example calls into is not coverage."""
    assert _recorded(calls, "AllowedCollisionMatrix.clearAllowedCollisions") == set()


def test_recorded_calls_name_their_module_and_example_line(calls):
    (call,) = [c for c in calls if c["qualname"] == "makeOrderedLinkPair"]
    assert call["module"] == "tesseract_common"
    assert call["example"] == "fixture_example.py"
    source = (EXAMPLE_ROOT / "fixture_example.py").read_text().splitlines()
    assert "makeOrderedLinkPair(" in source[call["line"] - 1]


def test_xdist_workers_each_record(tmp_path):
    """Under `-n 2` every worker writes its own file, and the union equals the serial run."""
    calls = run_recorded_session(tmp_path, EXAMPLE_ROOT, "-n", "2")
    assert len(list(tmp_path.glob("gw*.json"))) == 2
    assert _recorded(calls, "AllowedCollisionMatrix.__init__") == {("method", 0), ("method", 1)}


PURE_VIRTUAL = (
    "nanobind::detail::get_trampoline('locateResource()'): tried to call a pure virtual function!"
)


def test_trampoline_subclass_without_override_keeps_the_pure_virtual_error():
    """Non-perturbation: a wrapper on a trampoline base looks like a Python override.

    nanobind's trampoline looks the method up on the instance and treats any
    non-nanobind callable as an override, so it calls the wrapper, which calls
    back into C++. The trampoline's re-entry guard then dispatches to the C++
    base (`.scratch/probe_trampoline.log`: the wrapper runs twice, no recursion),
    and for a pure virtual that is the same error as without the recorder.
    """
    proc = subprocess.run(
        [sys.executable, str(TRAMPOLINE_SUBCLASS)],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        timeout=SUBPROCESS_TIMEOUT_S,
    )
    assert proc.returncode == 0, proc.stderr[-2000:]
    lines = [ln for ln in proc.stdout.splitlines() if ln.startswith("Defined")]
    assert lines == [
        f"DefinedBefore: RuntimeError: {PURE_VIRTUAL}",
        f"DefinedAfter: RuntimeError: {PURE_VIRTUAL}",
    ], proc.stdout[-2000:]
