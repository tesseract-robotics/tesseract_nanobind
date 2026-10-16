"""Render module reports as the mkdocs page and as JSON."""

from __future__ import annotations

import json
import re
import subprocess
from collections.abc import Mapping, Sequence
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import clang.cindex as ci
import tomllib
from audit_match import Gap, ModuleReport
from audit_ownership import (
    ANY_PREFIX,
    REPO_ROOT,
    THIRD_PARTY_HEADERS,
    UNAUDITED_HEADERS,
    Coverage,
    committed_stub,
)
from audit_rules import ACCEPTED
from audit_stub import Deviation, QuotedType, rel

REPORT_PATH = REPO_ROOT / "docs" / "developer" / "binding-api-audit.md"
# Committed outcome of every open row (#256, D10): {module: {row key: entry}}.
LEDGER_PATH = REPO_ROOT / "scripts" / "audit_ledger.json"
ISSUE_REF = re.compile(r"#\d+")
LEDGER_FIELDS = frozenset({"issue", "wont_fix"})
NO_OUTCOME = "—"  # issue column of an open row the ledger has no entry for
LIMITATION = (
    "Python member names are checked against every declaration in the TU, so a "
    "Python-only member that shares a common C++ name (`size`, `clear`) is not flagged. "
    "Overloads are matched by arity and parameter type: each C++ parameter type and each stub "
    "annotation map to a key (`scripts/audit_types.py`), and a bound overload covers a C++ one "
    "only where the keys agree. A type the mapping does not know (a template parameter, "
    "`object`, a quoted C++ name) leaves that position to arity alone; the `type fallbacks` "
    "column counts the C++ overloads covered that way. Overloads that differ only in `const` "
    "of `this` or `&`/`&&` of a parameter count as one. An overload taking a raw buffer (a "
    "pointer to a non-`char` scalar) has no Python form and is reported whatever the arity, "
    "unless it is a `(const uint8_t*, size_t)` pair bound as one `bytes` parameter. "
    "A header the binding never includes that does not compile on its own is listed as "
    "`unparseable header`, at its first error, and its API is not checked."
)


class MalformedLedgerError(ValueError):
    """The ledger is not `{module: {symbol: {"issue": "#NNN"} | {"wont_fix": reason}}}`."""


@dataclass(frozen=True)
class LedgerEntry:
    """The outcome of one open row: an issue reference or a won't-fix reason, never both."""

    issue: str | None
    wont_fix: str | None


def _unique_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    """A JSON object's pairs as a dict; a repeated key would silently drop an entry."""
    out: dict[str, Any] = {}
    for k, v in pairs:
        if k in out:
            raise MalformedLedgerError(f"duplicate key {k!r}")
        out[k] = v
    return out


def _ledger_entry(where: str, raw: object) -> LedgerEntry:
    if not isinstance(raw, dict):
        raise MalformedLedgerError(f"{where}: entry is not an object")
    if set(raw) - LEDGER_FIELDS or len(raw) != 1:
        raise MalformedLedgerError(
            f"{where}: needs exactly one of issue, wont_fix; has {sorted(raw)}"
        )
    issue, reason = raw.get("issue"), raw.get("wont_fix")
    if "issue" in raw and not (isinstance(issue, str) and ISSUE_REF.fullmatch(issue)):
        raise MalformedLedgerError(f"{where}: issue {issue!r} is not #NNN")
    if "wont_fix" in raw and not (isinstance(reason, str) and reason.strip()):
        raise MalformedLedgerError(f"{where}: wont_fix {reason!r} is not a non-empty reason")
    if "wont_fix" in raw and "|" in reason:
        raise MalformedLedgerError(
            f"{where}: wont_fix {reason!r} has a `|` (splits the table cell)"
        )
    return LedgerEntry(issue=issue, wont_fix=reason)


def load_ledger(path: Path = LEDGER_PATH) -> Mapping[str, Mapping[str, LedgerEntry]]:
    """The committed outcome ledger, validated.

    Raises:
        MalformedLedgerError: not JSON, a repeated key, or a module or entry of the wrong shape.
    """
    try:
        raw = json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=_unique_keys)
    except json.JSONDecodeError as e:
        raise MalformedLedgerError(f"{path}: {e}") from e
    if not isinstance(raw, dict):
        raise MalformedLedgerError(f"{path}: top level is not an object")
    out: dict[str, dict[str, LedgerEntry]] = {}
    for module, entries in raw.items():
        if not isinstance(entries, dict):
            raise MalformedLedgerError(f"{path}: {module} is not an object")
        out[module] = {s: _ledger_entry(f"{module}.{s}", e) for s, e in entries.items()}
    return out


def _issue_cell(entry: LedgerEntry | None) -> str:
    """The `issue` column: `#NNN`, `won't fix: <reason>`, or `—` for a row with no outcome."""
    if entry is None:
        return NO_OUTCOME
    return entry.issue if entry.issue is not None else f"won't fix: {entry.wont_fix}"


def _with_outcome(row: Gap | Deviation | QuotedType, entry: LedgerEntry | None) -> dict[str, Any]:
    """A report row as JSON, with its ledger outcome (`null` fields when it has none)."""
    return {
        **asdict(row),
        "issue": entry.issue if entry is not None else None,
        "wont_fix": entry.wont_fix if entry is not None else None,
    }


def _git(*args: str) -> str:
    return subprocess.run(
        ["git", *args], cwd=REPO_ROOT, capture_output=True, text=True, check=True
    ).stdout.strip()


def libclang_version() -> str:
    """`clang version 22.1.8`: python-clang registers no restype for this call, so set it."""
    fn = ci.conf.lib.clang_getClangVersion
    fn.restype = ci._CXString
    return ci._CXString.from_result(fn())


def provenance(modules: Sequence[str]) -> dict[str, str]:
    """Package pin, libclang version and the stubs' git SHA (no timestamps: deterministic)."""
    pins = tomllib.loads((REPO_ROOT / "pyproject.toml").read_text(encoding="utf-8"))
    stubs = [rel(committed_stub(m)) for m in modules]
    sha = _git("log", "-1", "--format=%h", "--", *stubs)
    dirty = " (uncommitted stub changes)" if _git("status", "--porcelain", "--", *stubs) else ""
    return {
        "tesseract-robotics": pins["tool"]["pixi"]["dependencies"]["tesseract-robotics"],
        "libclang": libclang_version(),
        "stubs": sha + dirty,
    }


def _table(header: tuple[str, ...], rows: list[tuple[str, ...]]) -> list[str]:
    if not rows:
        return ["None.", ""]
    return [
        "| " + " | ".join(header) + " |",
        "|" + "---|" * len(header),
        *("| " + " | ".join(r) + " |" for r in rows),
        "",
    ]


def _unaudited_patterns() -> list[tuple[str, str]]:
    """Each `UNAUDITED_HEADERS` pattern below its prefix (`*/` for `ANY_PREFIX`), with its
    reason, sorted by that path."""
    return sorted(
        (f"{'*/' if prefix == ANY_PREFIX else prefix}{pattern}", reason)
        for prefix, patterns in UNAUDITED_HEADERS.items()
        for pattern, reason in patterns.items()
    )


def render_markdown(
    reports: list[ModuleReport],
    prov: dict[str, str],
    cov: Coverage,
    ledger: Mapping[str, Mapping[str, LedgerEntry]],
) -> str:
    """The mkdocs report page: provenance, coverage, conclusion, then per-module tables.

    Each open row (gap, deviation, quoted name) shows its `ledger` outcome in an `issue` column.
    """
    out = [
        "# Binding API audit",
        "",
        "Generated by `pixi run -e audit audit-bindings`; do not edit by hand.",
        f"tesseract-robotics `{prov['tesseract-robotics']}` · libclang `{prov['libclang']}` · "
        f"stubs `{prov['stubs']}`.",
        "",
        f"{len(cov.audited)} of {cov.total} binding modules audited.",
        "",
        "Not audited:",
        "",
        *_table(("module", "reason"), list(cov.unaudited.items())),
        "| module | covered | gaps | deviations | accepted | quoted types | type fallbacks |",
        "|---|---|---|---|---|---|---|",
        *(
            f"| {r.module} | {r.covered} | {len(r.gaps)} | {len(r.deviations)} | "
            f"{len(r.accepted)} | {len(r.quoted)} | {r.type_fallbacks} |"
            for r in reports
        ),
        "",
        "Quoted C++ names in the stubs of audited modules (unbound type, or a missing "
        "`nb::module_::import_`):",
        "",
        *_table(
            ("module", "Python name", "annotation", "stub:line", "issue"),
            [
                (
                    r.module,
                    f"`{q.name}`",
                    f"`{q.annotation}`",
                    q.location,
                    _issue_cell(ledger.get(r.module, {}).get(q.annotation)),
                )
                for r in reports
                for q in r.quoted
            ],
        ),
        '!!! note "Accepted deviation rules"',
        *(f"    - `{k}`: {v}" for k, v in sorted(ACCEPTED.items())),
        "",
        '!!! note "Unaudited headers"',
        *(f"    - `{k}`: {v}" for k, v in _unaudited_patterns()),
        "    - A header its prefix owner's binding does not #include, but another binding does, is "
        "audited with the first such module by name; each module's section lists those headers.",
        "",
        '!!! note "Third-party headers (never audited, even when #included directly)"',
        *(f"    - `{k}`: {v}" for k, v in THIRD_PARTY_HEADERS.items()),
        "",
        f'!!! warning "Limitations"\n    {LIMITATION}',
        "",
    ]
    for r in reports:
        outcomes = ledger.get(r.module, {})
        out += [f"## {r.module}", "", "### Gaps", ""]
        out += _table(
            ("C++ symbol", "kind", "header:line", "missing overload (arity)", "issue"),
            [
                (
                    f"`{g.symbol}`",
                    g.kind.value,
                    g.location,
                    g.arity,
                    _issue_cell(outcomes.get(g.symbol)),
                )
                for g in r.gaps
            ],
        )
        out += ["### Deviations", ""]
        out += _table(
            ("Python name", "kind", "stub:line", "arity", "issue"),
            [
                (
                    f"`{d.name}`",
                    d.kind.value,
                    d.location,
                    d.arity,
                    _issue_cell(outcomes.get(d.name)),
                )
                for d in r.deviations
            ],
        )
        out += ["### Accepted", ""]
        out += _table(
            ("symbol", "rule", "location"),
            [(f"`{a.symbol}`", a.rule, a.location) for a in r.accepted],
        )
        out += ["### Headers audited with another module", ""]
        out += _table(
            ("header", "audited with", "status"),
            [
                (
                    f"`{h}`",
                    m,
                    "audited" if m in cov.audited else f"not yet: {cov.unaudited[m]}",
                )
                for h, m in r.delegated.items()
            ],
        )
    return "\n".join(out).rstrip() + "\n"


def to_json(
    reports: list[ModuleReport],
    prov: dict[str, str],
    cov: Coverage,
    ledger: Mapping[str, Mapping[str, LedgerEntry]],
) -> str:
    """Same data as the page, for issue drafting; open rows carry `issue` and `wont_fix`."""
    modules = {
        r.module: {
            "covered": r.covered,
            "gaps": [_with_outcome(g, ledger.get(r.module, {}).get(g.symbol)) for g in r.gaps],
            "deviations": [
                _with_outcome(d, ledger.get(r.module, {}).get(d.name)) for d in r.deviations
            ],
            "accepted": [asdict(a) for a in r.accepted],
            "quoted": [
                _with_outcome(q, ledger.get(r.module, {}).get(q.annotation)) for q in r.quoted
            ],
            "delegated": r.delegated,
            "type_fallbacks": r.type_fallbacks,
        }
        for r in reports
    }
    data = {
        "provenance": prov,
        "coverage": {"audited": list(cov.audited), "unaudited": dict(cov.unaudited)},
        "modules": modules,
    }
    return json.dumps(data, indent=2, sort_keys=True) + "\n"
