"""Contract tests of the committed outcome ledger `scripts/audit_ledger.json` (#256, D10).

Every open row of an audited module (gap, deviation, quoted name) maps to an open issue or a
won't-fix reason, and every ledger entry still has its row: a fix that removes a row removes
its entry in the same PR.

Run in the `audit` env only (libclang): `pixi run -e audit audit-test`.
"""

import json
import os
from pathlib import Path

import pytest

from tests.audit._load import audit
from tests.audit._prov import PROV

# Open-row tables of a module in `to_json`, with the field that is the row's ledger key.
OPEN_TABLES = (("gaps", "symbol"), ("deviations", "name"), ("quoted", "annotation"))
# Path of the findings `audit-bindings --json` wrote (.github/workflows/audit.yml).
FINDINGS_ENV = "BINDING_AUDIT_JSON"


@pytest.fixture(scope="module")
def ledger():
    return audit.load_ledger()


def test_committed_ledger_parses_and_validates(ledger):
    """A stray line in the JSON (a half-applied edit) failed every main audit run before any row."""
    assert ledger
    json.loads(audit.LEDGER_PATH.read_text(encoding="utf-8"))


def _open_rows(modules):
    for module, data in modules.items():
        for table, key in OPEN_TABLES:
            for row in data[table]:
                yield module, table, row[key], row


def _ledger_debt(modules, ledger):
    """Both directions in one message, empty when there is none: open rows with no outcome,
    and ledger entries with no open row."""
    rows = list(_open_rows(modules))
    missing = sorted(
        {(m, t, k) for m, t, k, row in rows if row["issue"] is None and row["wont_fix"] is None}
    )
    keys = {(m, k) for m, _, k, _ in rows}
    stale = sorted((m, s) for m, entries in ledger.items() for s in entries if (m, s) not in keys)
    parts = []
    if missing:
        parts.append(f"open rows with no issue or won't-fix entry in the ledger: {missing}")
    if stale:
        parts.append(f"ledger entries with no open row (fix landed: remove them): {stale}")
    return "\n".join(parts)


def _audited_modules(ledger):
    """The findings of `audit-bindings --json` when a job names them, else one fresh audit."""
    findings = os.environ.get(FINDINGS_ENV)
    if findings:
        return json.loads(Path(findings).read_text(encoding="utf-8"))["modules"]
    cov = audit.coverage()
    reports = [audit.audit_module(m) for m in cov.audited]
    return json.loads(audit.to_json(reports, PROV, cov, ledger))["modules"]


def test_every_open_row_has_an_outcome_and_every_entry_a_row(ledger):
    """Both directions on one audit of every audited module (one regeneration per run)."""
    debt = _ledger_debt(_audited_modules(ledger), ledger)
    assert not debt, debt


def test_ledger_test_reads_the_findings_it_is_pointed_at(tmp_path, monkeypatch):
    """CI names its one audit's findings: no second audit, and no file is a failure."""

    def audited(_module):
        pytest.fail("named findings must not be audited again")

    monkeypatch.setattr(audit, "audit_module", audited)
    report = audit.ModuleReport(
        module="fixture",
        covered=0,
        gaps=[audit.Gap("Open", audit.Kind.CLASS, "tesseract/fixture/a.h:1")],
        deviations=[],
        accepted=[],
        quoted=[],
    )
    cov = audit.Coverage(audited=("fixture",), unaudited={}, total=1)
    findings = tmp_path / "findings.json"
    findings.write_text(audit.to_json([report], PROV, cov, {}), encoding="utf-8")
    monkeypatch.setenv(FINDINGS_ENV, str(findings))
    assert "('fixture', 'gaps', 'Open')" in _ledger_debt(_audited_modules({}), {})
    monkeypatch.setenv(FINDINGS_ENV, str(tmp_path / "absent.json"))
    with pytest.raises(FileNotFoundError):
        _audited_modules({})


def test_ledger_debt_names_missing_and_stale_rows_together():
    """CI shows the whole debt in one failure: open rows with no outcome and entries with no
    row, not the stale half only after the missing half is paid (#373)."""
    modules = {
        "m": {
            "gaps": [{"symbol": "Open", "issue": None, "wont_fix": None}],
            "deviations": [{"name": "Settled", "issue": "#1", "wont_fix": None}],
            "quoted": [],
        }
    }
    ledger = {
        "m": {
            "Settled": audit.LedgerEntry(issue="#1", wont_fix=None),
            "Gone": audit.LedgerEntry(issue=None, wont_fix="Fixed."),
        }
    }
    message = _ledger_debt(modules, ledger)
    assert "('m', 'gaps', 'Open')" in message
    assert "('m', 'Gone')" in message
    assert (
        _ledger_debt(
            {"m": {**modules["m"], "gaps": []}}, {"m": {"Settled": ledger["m"]["Settled"]}}
        )
        == ""
    )


MALFORMED = {
    "not json": "{",
    "module not an object": json.dumps({"m": ["s"]}),
    "entry not an object": json.dumps({"m": {"s": "#1"}}),
    "both fields": json.dumps({"m": {"s": {"issue": "#1", "wont_fix": "why"}}}),
    "neither field": json.dumps({"m": {"s": {}}}),
    "unknown field": json.dumps({"m": {"s": {"issue": "#1", "note": "x"}}}),
    "issue without #": json.dumps({"m": {"s": {"issue": "163"}}}),
    "issue with a suffix": json.dumps({"m": {"s": {"issue": "#163 (part)"}}}),
    "empty reason": json.dumps({"m": {"s": {"wont_fix": ""}}}),
    "reason not a string": json.dumps({"m": {"s": {"wont_fix": 1}}}),
    "whitespace-only reason": json.dumps({"m": {"s": {"wont_fix": "  "}}}),
    "reason splits a table cell": json.dumps({"m": {"s": {"wont_fix": "a | b"}}}),
    "issue not a string": json.dumps({"m": {"s": {"issue": 163}}}),
    "top level not an object": json.dumps([]),
    "duplicate symbol": '{"m": {"s": {"issue": "#1"}, "s": {"issue": "#2"}}}',
    "duplicate module": '{"m": {"s": {"issue": "#1"}}, "m": {"t": {"issue": "#2"}}}',
}


@pytest.mark.parametrize("text", MALFORMED.values(), ids=MALFORMED.keys())
def test_ledger_entries_are_well_formed(tmp_path, text):
    path = tmp_path / "audit_ledger.json"
    path.write_text(text, encoding="utf-8")
    with pytest.raises(audit.MalformedLedgerError):
        audit.load_ledger(path)


def test_committed_ledger_entries_are_well_formed(ledger):
    entries = [e for module in ledger.values() for e in module.values()]
    assert entries
    assert all((e.issue is None) != (e.wont_fix is None) for e in entries)


def test_issue_column_renders_each_outcome():
    report = audit.ModuleReport(
        module="fixture",
        covered=0,
        gaps=[
            audit.Gap("Filed", audit.Kind.CLASS, "tesseract/fixture/a.h:1"),
            audit.Gap("Open", audit.Kind.CLASS, "tesseract/fixture/a.h:2"),
        ],
        deviations=[audit.Deviation("pyOnly", audit.Kind.FUNCTION, "fixture.pyi:1")],
        accepted=[],
        quoted=[audit.QuotedType("f", "ns::Unbound", "fixture.pyi:2")],
    )
    ledger = {
        "fixture": {
            "Filed": audit.LedgerEntry(issue="#12", wont_fix=None),
            "pyOnly": audit.LedgerEntry(issue=None, wont_fix="Python covers it."),
            "ns::Unbound": audit.LedgerEntry(issue="#13", wont_fix=None),
        }
    }
    cov = audit.Coverage(audited=("fixture",), unaudited={}, total=1)
    text = audit.render_markdown([report], PROV, cov, ledger)
    # Table rows keyed by their first backticked cell.
    rows = {
        line.split("`")[1]: line
        for line in text.splitlines()
        if line.startswith("|") and "`" in line
    }
    assert rows["Filed"].endswith("| #12 |")
    assert rows["Open"] == "| `Open` | class | tesseract/fixture/a.h:2 | — | — |"
    assert rows["pyOnly"].endswith("| won't fix: Python covers it. |")
    assert rows["f"].endswith("| #13 |")
    # Table headers of Gaps, Deviations and the quoted names.
    starts = ("| C++ symbol |", "| Python name |", "| module | Python name |")
    headers = [line for line in text.splitlines() if line.startswith(starts)]
    assert len(headers) == 3
    assert all(h.endswith("| issue |") for h in headers)

    data = json.loads(audit.to_json([report], PROV, cov, ledger))["modules"]["fixture"]
    assert [(g["issue"], g["wont_fix"]) for g in data["gaps"]] == [("#12", None), (None, None)]
    assert (data["deviations"][0]["issue"], data["deviations"][0]["wont_fix"]) == (
        None,
        "Python covers it.",
    )
    assert data["quoted"][0]["issue"] == "#13"
