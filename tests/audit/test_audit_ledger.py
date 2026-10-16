"""Contract tests of the committed outcome ledger `scripts/audit_ledger.json` (#256, D10).

Every open row of an audited module (gap, deviation, quoted name) maps to an open issue or a
won't-fix reason, and every ledger entry still has its row: a fix that removes a row removes
its entry in the same PR.

Run in the `audit` env only (libclang): `pixi run -e audit audit-test`.
"""

import json

import pytest

from tests.audit._load import audit

PROV = {"tesseract-robotics": "==0.35.0", "libclang": "clang version 23", "stubs": "abc1234"}
# Open-row tables of a module in `to_json`, with the field that is the row's ledger key.
OPEN_TABLES = (("gaps", "symbol"), ("deviations", "name"), ("quoted", "annotation"))


@pytest.fixture(scope="module")
def ledger():
    return audit.load_ledger()


@pytest.fixture(scope="module")
def audited_modules(ledger):
    """`to_json`'s per-module data for a fresh audit of every audited module."""
    cov = audit.coverage()
    reports = [audit.audit_module(m) for m in cov.audited]
    return json.loads(audit.to_json(reports, PROV, cov, ledger))["modules"]


def _open_rows(modules):
    for module, data in modules.items():
        for table, key in OPEN_TABLES:
            for row in data[table]:
                yield module, table, row[key], row


def test_every_open_row_has_an_outcome(audited_modules):
    missing = sorted(
        {
            (module, table, key)
            for module, table, key, row in _open_rows(audited_modules)
            if row["issue"] is None and row["wont_fix"] is None
        }
    )
    assert not missing, f"open rows with no issue or won't-fix entry in the ledger: {missing}"


def test_every_ledger_entry_has_a_row(ledger, audited_modules):
    rows = {(module, key) for module, _, key, _ in _open_rows(audited_modules)}
    stale = sorted((m, s) for m, entries in ledger.items() for s in entries if (m, s) not in rows)
    assert not stale, f"ledger entries with no open row (fix landed: remove them): {stale}"


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
    assert rows["Open"].endswith("| — |")
    assert rows["pyOnly"].endswith("| won't fix: Python covers it. |")
    assert rows["f"].endswith("| #13 |")

    data = json.loads(audit.to_json([report], PROV, cov, ledger))["modules"]["fixture"]
    assert [(g["issue"], g["wont_fix"]) for g in data["gaps"]] == [("#12", None), (None, None)]
    assert (data["deviations"][0]["issue"], data["deviations"][0]["wont_fix"]) == (
        None,
        "Python covers it.",
    )
    assert data["quoted"][0]["issue"] == "#13"
