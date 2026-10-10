"""Contract tests of `scripts/api_inventory.py`, the stub diff the example coverage oracle reads.

The inventory identifies an overload by its normalised parameter types. Each
test names the normalisation (or classification) it pins; a regression in any
of them shows up as false "new API" (annotation-only stub changes, #218) or as
API the oracle silently never asks an example to cover.
"""

import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
_SCRIPT = REPO_ROOT / "scripts" / "api_inventory.py"
FIXTURES = Path(__file__).resolve().parent / "fixtures" / "stubs"

_spec = importlib.util.spec_from_file_location("api_inventory", _SCRIPT)
inv = importlib.util.module_from_spec(_spec)
sys.modules["api_inventory"] = inv  # dataclasses resolve their module by name
_spec.loader.exec_module(inv)

BASE_SRC = (FIXTURES / "base.pyi").read_text()
HEAD_SRC = (FIXTURES / "head.pyi").read_text()

# The full expected diff of the fixture pair: (qualname, kind, overload, change).
EXPECTED_DIFF = {
    ("Mode.B", "enum_member", "", "new_name"),
    ("Robot.setState", "method", "(Mapping[str, Isometry3d])", "new_overload"),
    ("Robot.resize", "method", "(float)", "changed_signature"),
    ("Robot.__eq__", "method", "(Robot)", "new_name"),
    ("Robot.payload", "property", "", "new_name"),
    ("Robot.make", "staticmethod", "(str)", "new_name"),
    ("Robot.CONFIG_KEY", "attribute", "", "new_name"),
    ("RobotError", "class", "", "new_name"),
    ("helper2", "function", "(Sequence[str])", "new_name"),
}


def _diff(base, head):
    return {(e.qualname, e.kind, e.overload, c.value) for e, c in inv.diff_stub(base, head)}


def test_fixture_diff_is_exact():
    """Every classification at once: kinds, change labels, and nothing else reported."""
    assert _diff(BASE_SRC, HEAD_SRC) == EXPECTED_DIFF


@pytest.mark.parametrize(
    "name, mechanism",
    [
        ("Robot.place", "C++ spelling `Transform<double, 3, 1, 0>` is `Isometry3d`"),
        ("Robot.locate", "`ns::` prefixes are stripped from a quoted C++ name"),
        ("Robot.frame", "a quoted, module-qualified annotation equals the unquoted one"),
        ("Robot.rename", "parameter names and defaults are not overload identity"),
        ("Robot.connect", "a `from` parameter (ompl stub) parses and equals `from_`"),
        ("Robot._private", "private names are not API"),
    ],
)
def test_annotation_only_changes_report_nothing(name, mechanism):
    assert name not in {q for q, *_ in _diff(BASE_SRC, HEAD_SRC)}, mechanism


def test_added_overload_is_new_overload():
    src = "class C:\n    def f(self, a: int) -> None: ...\n"
    added = src + "    def f(self, a: str, b: int) -> None: ...\n"
    assert _diff(src, added) == {("C.f", "method", "(str, int)", "new_overload")}


def test_new_module_reports_every_public_name():
    assert _diff(None, "def g(x: int) -> int: ...\n") == {("g", "function", "(int)", "new_name")}


def test_stub_overloads_keep_dispatch_order():
    """The report maps a recorded `__nb_signature__` index to the stub overload by position."""
    overloads = inv.stub_overloads(HEAD_SRC)
    assert overloads["Robot.setState"] == ["(Mapping[str, float])", "(Mapping[str, Isometry3d])"]
    assert overloads["Robot.make"] == ["(str)"]
    assert "Robot.payload" not in overloads


def test_unparseable_stub_raises():
    with pytest.raises(inv.StubSyntaxError):
        inv.stub_entries("def broken(:\n")


def _git(repo, *args):
    subprocess.run(
        ["git", "-c", "user.name=t", "-c", "user.email=t@t", *args],
        cwd=repo,
        check=True,
        capture_output=True,
        text=True,
    )


def _rev(repo, rev):
    out = subprocess.run(
        ["git", "rev-parse", rev], cwd=repo, check=True, capture_output=True, text=True
    )
    return out.stdout.strip()


@pytest.fixture
def stub_repo(tmp_path):
    """A git repo whose fixture module stub goes base → (Mode.B added) → head."""
    stub = tmp_path / "src" / "tesseract_robotics" / "fixture_mod" / "_fixture_mod.pyi"
    stub.parent.mkdir(parents=True)
    _git(tmp_path, "init", "-q")
    stub.write_text(BASE_SRC)
    _git(tmp_path, "add", ".")
    _git(tmp_path, "commit", "-q", "-m", "base")
    stub.write_text(BASE_SRC.replace("    A = 0\n", "    A = 0\n\n    B = 1\n"))
    _git(tmp_path, "commit", "-q", "-am", "feat(mode): add B (#901)")
    stub.write_text(HEAD_SRC)
    _git(tmp_path, "commit", "-q", "-am", "feat(robot): the rest (#902)")
    return tmp_path


def test_rows_attribute_each_entry_to_its_first_commit(stub_repo):
    inventory = inv.build_inventory(stub_repo, "HEAD~2", "HEAD")
    rows = {r.qualname: r for r in inventory.rows}
    assert {(r.qualname, r.kind, r.overload, r.change) for r in inventory.rows} == EXPECTED_DIFF
    assert {r.module for r in inventory.rows} == {"fixture_mod"}
    assert rows["Mode.B"].subject == "feat(mode): add B (#901)"
    assert rows["Mode.B"].commit == _rev(stub_repo, "HEAD~1")[:8]
    assert {r.subject for q, r in rows.items() if q != "Mode.B"} == {"feat(robot): the rest (#902)"}
    assert (inventory.base, inventory.head) == (_rev(stub_repo, "HEAD~2"), _rev(stub_repo, "HEAD"))


def test_cli_writes_the_row_contract(stub_repo):
    out = stub_repo / "inventory.json"
    rc = inv.main(
        ["--repo", str(stub_repo), "--base", "HEAD~2", "--head", "HEAD", "--out", str(out)]
    )
    assert rc == 0
    data = json.loads(out.read_text())
    assert set(data) == {"base", "head", "rows"}
    assert set(data["rows"][0]) == {
        "module",
        "qualname",
        "kind",
        "overload",
        "arity",
        "change",
        "commit",
        "subject",
    }
    assert len(data["rows"]) == len(EXPECTED_DIFF)


def test_unknown_revision_raises(stub_repo):
    with pytest.raises(inv.GitRevisionError):
        inv.build_inventory(stub_repo, "no-such-rev", "HEAD")
