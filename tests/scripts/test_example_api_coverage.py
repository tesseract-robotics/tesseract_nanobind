"""Contract tests of `scripts/example_api_coverage.py`, the coverage report and gate.

The report joins three inputs: the inventory (`scripts/api_inventory.py`), the
exemption rules X1–X7, and the recorder's calls (`tests/examples/api_recorder.py`).
Each test names the rule it pins. The ablation at the end runs the fixture
example under the recorder, deletes one call from a copy of it, and checks that
exactly that entry turns uncovered: the oracle is wired end to end, and it
cannot be satisfied by anything but the call itself.
"""

import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import pytest

from tests.scripts.test_api_recorder import EXAMPLE_ROOT, run_recorded_session

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
SCRIPTS = REPO_ROOT / "scripts"
FIXTURE_STUB = Path(__file__).resolve().parent / "fixtures" / "stubs" / "head.pyi"


def _load(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod  # dataclasses resolve their module by name
    spec.loader.exec_module(mod)
    return mod


inv = _load("api_inventory")
cov = _load("example_api_coverage")

MOD = "fixture_mod"
# Overloads of the fixture stub (tests/scripts/fixtures/stubs/head.pyi), in stub order.
SET_STATE_FLOAT, SET_STATE_ISO = "(Mapping[str, float])", "(Mapping[str, Isometry3d])"


def row(qualname, kind="method", overload="", change="new_name", module=MOD):
    return {
        "module": module,
        "qualname": qualname,
        "kind": kind,
        "overload": overload,
        "arity": -1,
        "change": change,
        "commit": "0123abcd",
        "subject": "feat: fixture (#901)",
    }


def call(qualname, kind="method", index=0, candidates=None, fully_bound=True, module=MOD, line=1):
    if candidates is None:
        candidates = [] if index is None else [index]
    return {
        "module": module,
        "qualname": qualname,
        "kind": kind,
        "index": index,
        "candidates": candidates,
        "fully_bound": fully_bound,
        "example": "fixture_example.py",
        "line": line,
    }


OVERLOADS = {MOD: inv.stub_overloads(FIXTURE_STUB.read_text())}
INSTRUMENTED = {(MOD, q): len(o) for q, o in OVERLOADS[MOD].items()} | {(MOD, "Robot.payload"): 0}
TEST_TEXT = "def test_robot():\n    Robot() == Robot()\n"


def verdict_of(rows, calls, test_text=TEST_TEXT, instrumented=INSTRUMENTED):
    report = cov.classify(rows, calls, instrumented, OVERLOADS, test_text)
    return {(r.row["qualname"], r.row["overload"]): r for r in report.results}, report


def test_a_call_of_the_listed_overload_covers_it():
    results, _ = verdict_of(
        [row("Robot.setState", overload=SET_STATE_ISO)], [call("Robot.setState", index=1)]
    )
    assert results[("Robot.setState", SET_STATE_ISO)].verdict is cov.Verdict.COVERED


def test_a_call_of_another_overload_does_not():
    results, _ = verdict_of(
        [row("Robot.setState", overload=SET_STATE_ISO)], [call("Robot.setState", index=0)]
    )
    assert results[("Robot.setState", SET_STATE_ISO)].verdict is cov.Verdict.UNCOVERED


@pytest.mark.parametrize("change", ["new_overload", "changed_signature"])
def test_new_and_changed_overloads_need_full_binding(change):
    """An old-style call that leaves the new parameter defaulted demonstrates nothing new."""
    rows = [row("Robot.setState", overload=SET_STATE_ISO, change=change)]
    unbound, _ = verdict_of(rows, [call("Robot.setState", index=1, fully_bound=False)])
    assert unbound[("Robot.setState", SET_STATE_ISO)].verdict is cov.Verdict.UNCOVERED
    assert "every parameter" in unbound[("Robot.setState", SET_STATE_ISO)].note
    bound, _ = verdict_of(rows, [call("Robot.setState", index=1, fully_bound=True)])
    assert bound[("Robot.setState", SET_STATE_ISO)].verdict is cov.Verdict.COVERED


def test_a_new_name_needs_no_full_binding():
    results, _ = verdict_of(
        [row("Robot.setState", overload=SET_STATE_ISO)],
        [call("Robot.setState", index=1, fully_bound=False)],
    )
    assert results[("Robot.setState", SET_STATE_ISO)].verdict is cov.Verdict.COVERED


def test_an_ambiguous_call_makes_its_candidates_ambiguous():
    """`setState({})`: both mapping overloads are candidates, neither is covered."""
    rows = [
        row("Robot.setState", overload=SET_STATE_FLOAT),
        row("Robot.setState", overload=SET_STATE_ISO),
    ]
    results, report = verdict_of(
        rows, [call("Robot.setState", index=None, candidates=[0, 1], fully_bound=False)]
    )
    assert {r.verdict for r in results.values()} == {cov.Verdict.AMBIGUOUS}
    assert report.gate_failed


def test_a_covering_call_wins_over_an_ambiguous_one():
    calls = [
        call("Robot.setState", index=None, candidates=[0, 1]),
        call("Robot.setState", index=1, line=2),
    ]
    results, _ = verdict_of([row("Robot.setState", overload=SET_STATE_ISO)], calls)
    assert results[("Robot.setState", SET_STATE_ISO)].verdict is cov.Verdict.COVERED


def test_a_property_is_covered_by_any_access():
    results, _ = verdict_of(
        [row("Robot.payload", kind="property")],
        [call("Robot.payload", kind="property", index=None)],
    )
    assert results[("Robot.payload", "")].verdict is cov.Verdict.COVERED


def test_a_class_is_covered_by_a_member_call_and_an_enum_by_its_use():
    rows = [row("Robot", kind="class"), row("Mode", kind="class"), row("Unused", kind="class")]
    calls = [call("Robot.make", kind="staticmethod"), call("Mode", kind="enum", index=None)]
    results, _ = verdict_of(rows, calls)
    assert [results[(q, "")].verdict for q in ("Robot", "Mode", "Unused")] == [
        cov.Verdict.COVERED,
        cov.Verdict.COVERED,
        cov.Verdict.UNCOVERED,
    ]


def test_an_entry_the_recorder_did_not_instrument_says_so():
    results, _ = verdict_of([row("Robot.ghost", overload="()")], [])
    assert results[("Robot.ghost", "()")].verdict is cov.Verdict.UNCOVERED
    assert "not instrumented" in results[("Robot.ghost", "()")].note


@pytest.mark.parametrize(
    "entry, rule",
    [
        (row("Robot.__eq__", overload="(Robot)"), "X1"),
        (row("Mode.B", kind="enum_member"), "X2"),
        (row("JointTrajectory.reserve", overload="(int)"), "X4"),
        (row("Robot.CONFIG_KEY", kind="attribute"), "X5"),
        (row("VHACDParameters.print", overload="()"), "X6"),
        (row("Environment.setStateByMap", overload="(Mapping[str, float])"), "X7"),
    ],
)
def test_exemption_rules_match_their_entries(entry, rule):
    results, _ = verdict_of([entry], [])
    (result,) = results.values()
    assert (result.verdict, result.rule) == (cov.Verdict.EXEMPT, rule)


def test_an_x1_or_x3_class_no_test_names_is_uncovered():
    """X1 and X3 lean on the tests; an exempted name that no test mentions is not exempt."""
    rows = [
        row("Robot.__eq__", overload="(Robot)"),
        row("Ghost.__eq__", overload="(Ghost)"),
        row("RobotError", kind="class"),
    ]
    results, _ = verdict_of(rows, [])
    assert results[("Robot.__eq__", "(Robot)")].verdict is cov.Verdict.EXEMPT
    assert results[("Ghost.__eq__", "(Ghost)")].verdict is cov.Verdict.UNCOVERED
    assert results[("RobotError", "")].verdict is cov.Verdict.UNCOVERED
    assert "no test names" in results[("RobotError", "")].note
    named, _ = verdict_of(rows[2:], [], test_text=TEST_TEXT + "pytest.raises(RobotError)\n")
    assert named[("RobotError", "")].verdict is cov.Verdict.EXEMPT


def test_a_stub_with_a_different_overload_count_raises():
    """The report maps a recorded overload index to the stub by position; counts must agree."""
    instrumented = INSTRUMENTED | {(MOD, "Robot.setState"): 3}
    with pytest.raises(cov.StubOverloadMismatchError):
        verdict_of(
            [row("Robot.setState", overload=SET_STATE_ISO)],
            [call("Robot.setState", index=1)],
            instrumented=instrumented,
        )


def test_a_call_the_matcher_rejected_fails_the_gate():
    """nanobind ran the call, the matcher found no overload for it: a matcher defect, never coverage."""
    _, report = verdict_of([], [call("Robot.setState", index=None, candidates=[])])
    assert report.gate_failed
    assert [(c["qualname"], c["line"]) for c in report.rejections] == [("Robot.setState", 1)]


def test_report_states_covered_over_total_first_and_lists_every_open_entry():
    rows = [
        row("Robot.setState", overload=SET_STATE_FLOAT),
        row("Robot.setState", overload=SET_STATE_ISO),
        row("helper2", kind="function", overload="(Sequence[str])"),
        row("Mode.B", kind="enum_member"),
    ]
    calls = [
        call("Robot.setState", index=0),
        call("helper2", kind="function", index=None, candidates=[0]),
    ]
    _, report = verdict_of(rows, calls)
    text = cov.render(report, "a" * 40, "b" * 40)
    first = next(ln for ln in text.splitlines() if ln and not ln.startswith(("#", "Generated")))
    assert first.startswith("**1 / 3 non-exempt entries covered**")
    open_section = text.split("## Covered")[0]
    assert f"`Robot.setState{SET_STATE_ISO}`" in open_section
    assert "`helper2(Sequence[str])`" in open_section


def test_missing_recordings_raise(tmp_path):
    with pytest.raises(cov.RecordingsMissingError):
        cov.load_recordings(tmp_path)


def _git(repo, *args):
    subprocess.run(
        ["git", "-c", "user.name=t", "-c", "user.email=t@t", *args],
        cwd=repo,
        check=True,
        capture_output=True,
    )


def test_an_inventory_of_another_head_is_stale(tmp_path):
    _git(tmp_path, "init", "-q")
    _git(tmp_path, "commit", "-q", "--allow-empty", "-m", "head")
    inventory = tmp_path / "inventory.json"
    inventory.write_text(json.dumps({"base": "0" * 40, "head": "f" * 40, "rows": []}))
    with pytest.raises(cov.StaleInventoryError):
        cov.check_inventory_head(tmp_path, json.loads(inventory.read_text()))


# --- end to end: fixture example → recorder → report ----------------------------------

COMMON_STUB = (
    REPO_ROOT / "src" / "tesseract_robotics" / "tesseract_common" / "_tesseract_common.pyi"
)
COMMON = "tesseract_common"
# Each call of the fixture example, as (source fragment of its line, the entry it covers).
FIXTURE_CALLS = {
    "acm = AllowedCollisionMatrix()": ("AllowedCollisionMatrix.__init__", 0),
    'AllowedCollisionMatrix({("a", "b"): "Adjacent"})': ("AllowedCollisionMatrix.__init__", 1),
    'removeAllowedCollision("a", "b")': ("AllowedCollisionMatrix.removeAllowedCollision", 0),
    'removeAllowedCollision("a")': ("AllowedCollisionMatrix.removeAllowedCollision", 1),
    "first = trajectory[0]": ("JointTrajectory.__getitem__", 0),
    "trajectory[0] = first": ("JointTrajectory.__setitem__", 0),
    # `==` / `!=` are recorded too (test_api_recorder), but X1 exempts them by name.
    "Isometry3d.Identity()": ("Isometry3d.Identity", 0),
    'makeOrderedLinkPair("b", "a")': ("makeOrderedLinkPair", 0),
}
# Inventory kinds of the entries above that are not methods.
KINDS = {"Isometry3d.Identity": "staticmethod", "makeOrderedLinkPair": "function"}


def fixture_inventory_rows():
    overloads = inv.stub_overloads(COMMON_STUB.read_text())
    rows = []
    for qualname, index in FIXTURE_CALLS.values():
        change = "new_overload" if qualname.endswith("removeAllowedCollision") else "new_name"
        rows.append(
            row(qualname, KINDS.get(qualname, "method"), overloads[qualname][index], change, COMMON)
        )
    return rows


def coverage_of(example_root, record_dir):
    run_recorded_session(record_dir, example_root)
    calls, instrumented = cov.load_recordings(record_dir)
    overloads = {COMMON: inv.stub_overloads(COMMON_STUB.read_text())}
    report = cov.classify(fixture_inventory_rows(), calls, instrumented, overloads, "")
    return {(r.row["qualname"], r.row["overload"]): r.verdict for r in report.results}


def test_the_fixture_example_covers_every_entry_it_calls(tmp_path):
    verdicts = coverage_of(EXAMPLE_ROOT, tmp_path)
    assert set(verdicts.values()) == {cov.Verdict.COVERED}, verdicts


# (call to delete, what replaces its line so the rest of the example still runs)
ABLATIONS = {
    'AllowedCollisionMatrix({("a", "b"): "Adjacent"})': [
        "acm_from_entries = AllowedCollisionMatrix()",
        'acm_from_entries.addAllowedCollision("a", "b", "Adjacent")',
    ],
    "trajectory[0] = first": ["pass"],
    'makeOrderedLinkPair("b", "a")': ['pair = ("a", "b")'],
}


@pytest.mark.parametrize("fragment", list(ABLATIONS))
def test_ablation_deleting_one_call_uncovers_exactly_its_entry(tmp_path, fragment):
    """Ablation: the example minus one call leaves every other entry covered and that one red."""
    source = (EXAMPLE_ROOT / "fixture_example.py").read_text().splitlines(keepends=True)
    (at,) = [i for i, line in enumerate(source) if fragment in line]
    indent = source[at][: len(source[at]) - len(source[at].lstrip())]
    source[at] = "".join(f"{indent}{line}\n" for line in ABLATIONS[fragment])
    example = tmp_path / "examples"
    example.mkdir()
    (example / "fixture_example.py").write_text("".join(source))

    verdicts = coverage_of(example, tmp_path / "recordings")
    qualname, index = FIXTURE_CALLS[fragment]
    ablated = (qualname, inv.stub_overloads(COMMON_STUB.read_text())[qualname][index])
    assert {k for k, v in verdicts.items() if v is not cov.Verdict.COVERED} == {ablated}
