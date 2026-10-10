"""Example API coverage: does a shipped example execute each API entry added since 0.35.0.9?

Joins three inputs into `docs/developer/example-api-coverage.md` and a gate:

- the inventory (`scripts/api_inventory.py`): every public stub entry added
  between the 0.35.0.9 cut and HEAD, one row per overload;
- the exemption rules X1–X7 below, each with its reason;
- the recordings of `tests/examples` run under `-p tests.examples.api_recorder`:
  every call an example file made, with the overload it dispatched to.

A non-exempt entry is covered only by a call whose immediate caller is an
example file, matched to exactly that overload. A `new_overload` or
`changed_signature` entry also needs a call that binds every parameter (full
binding): an old-style call that leaves a new defaulted parameter unbound
demonstrates nothing new. An entry an undecided call might have hit is
ambiguous, not covered. The gate fails on any uncovered or ambiguous entry, and
on any call the overload matcher rejected although nanobind ran it.

Example:
    ```bash
    pixi run example-coverage
    ```
"""

from __future__ import annotations

import argparse
import enum
import json
import re
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

from api_inventory import stub_overloads

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_INVENTORY = Path("build/example-api-coverage/inventory.json")
DEFAULT_RECORDINGS = Path("build/example-api-coverage/recordings")
DEFAULT_REPORT = Path("docs/developer/example-api-coverage.md")
STUB_ROOT = Path("src/tesseract_robotics")
TESTS_ROOT = Path("tests")
# The oracle's own fixtures name classes to exercise the recorder; they are not tests of them.
TEST_REFERENCE_EXCLUDED = Path("tests/scripts/fixtures")
CALLABLE_KINDS = frozenset({"method", "function", "staticmethod"})
VALUE_KINDS = frozenset({"attribute", "enum_member"})
FULL_BINDING_CHANGES = frozenset({"new_overload", "changed_signature"})
ISSUE_REF = re.compile(r"\(#(\d+)\)")
SITES_SHOWN = 3  # call sites listed per entry in the report
SHORT_SHA = 8
MODULE_PREFIX = "tesseract_"  # dropped from module names in the tables


class RecordingsMissingError(Exception):
    """The recordings directory holds no recorder output."""


class StubOverloadMismatchError(Exception):
    """A callable has a different number of overloads at runtime than in its stub."""


class StaleInventoryError(Exception):
    """The inventory was built for a different HEAD than the repository's."""


class Verdict(str, enum.Enum):
    COVERED = "covered"
    UNCOVERED = "uncovered"
    AMBIGUOUS = "ambiguous"
    EXEMPT = "exempt"


@dataclass(frozen=True)
class ExemptionRule:
    """One exemption: an entry matching every given field is exempt for `reason`.

    Attributes:
        code: Rule name, X1–X7.
        reason: Why examples need not demonstrate the entries.
        kinds: Inventory kinds the rule covers (any if empty).
        leaf_names: Last qualname component (any if empty).
        leaf_suffix: Required suffix of the last qualname component.
        qualnames: Exact qualified names (any if empty), where the names are the rule.
        needs_test_reference: The rule leans on the tests, so the class (X1) or
            the exception (X3) must be named in a test under `tests/`.
    """

    code: str
    reason: str
    kinds: frozenset[str] = frozenset()
    leaf_names: frozenset[str] = frozenset()
    leaf_suffix: str = ""
    qualnames: frozenset[str] = frozenset()
    needs_test_reference: bool = False

    def matches(self, row: dict) -> bool:
        leaf = row["qualname"].rsplit(".", 1)[-1]
        return (
            (not self.kinds or row["kind"] in self.kinds)
            and (not self.leaf_names or leaf in self.leaf_names)
            and leaf.endswith(self.leaf_suffix)
            and (not self.qualnames or row["qualname"] in self.qualnames)
        )

    def referenced_name(self, row: dict) -> str:
        """The name a test must mention: the class of an X1 dunder, the X3 exception itself."""
        return (
            row["qualname"].rsplit(".", 1)[0] if row["kind"] in CALLABLE_KINDS else row["qualname"]
        )


X1 = ExemptionRule(
    "X1",
    "Value equality (#169) is pinned by the `test_value_equality.py` tests; one example shows it once.",
    leaf_names=frozenset({"__eq__", "__ne__"}),
    needs_test_reference=True,
)
X2 = ExemptionRule(
    "X2",
    "Enum members are covered when their enum class is used; the values are listed in `docs/api`.",
    kinds=frozenset({"enum_member"}),
)
X3 = ExemptionRule(
    "X3",
    "Exception classes: each issue's tests pin the raise paths red-first; examples show the success path.",
    kinds=frozenset({"class"}),
    leaf_suffix="Error",
    needs_test_reference=True,
)
X4 = ExemptionRule(
    "X4",
    "`std::vector` capacity and accessor plumbing, duplicated by `[i]`, `[0]`, `[-1]` and `len`.",
    qualnames=frozenset(
        {
            "JointTrajectory.capacity",
            "JointTrajectory.max_size",
            "JointTrajectory.reserve",
            "JointTrajectory.shrink_to_fit",
            "JointTrajectory.at",
            "JointTrajectory.front",
            "JointTrajectory.back",
            "JointTrajectory.pop_back",
            "ContactResultMap.shrinkToFit",
            "AllowedCollisionMatrix.reserveAllowedCollisionMatrix",
        }
    ),
)
X5 = ExemptionRule(
    "X5",
    "`CONFIG_KEY` YAML key constants: data, documented in `docs/api`.",
    leaf_names=frozenset({"CONFIG_KEY"}),
)
X6 = ExemptionRule(
    "X6", "A debug printer to stdout.", qualnames=frozenset({"VHACDParameters.print"})
)
X7 = ExemptionRule(
    "X7",
    "Python-only compatibility aliases; examples use the native overloads.",
    leaf_names=frozenset(
        {"setStateByMap", "setStateByNamesAndValues", "getStateByMap", "getStateByNamesAndValues"}
    ),
)
EXEMPTIONS = (X1, X2, X3, X4, X5, X6, X7)


@dataclass(frozen=True)
class Result:
    """The verdict on one inventory row.

    Attributes:
        row: The inventory row.
        verdict: covered, uncovered, ambiguous or exempt.
        rule: The exemption rule, if exempt.
        note: Why an open entry is open.
        sites: `example:line` of the calls behind the verdict.
    """

    row: dict
    verdict: Verdict
    rule: str | None = None
    note: str = ""
    sites: tuple[str, ...] = ()


@dataclass
class Report:
    """Every row's verdict, plus the calls the overload matcher rejected."""

    results: list[Result] = field(default_factory=list)
    rejections: list[dict] = field(default_factory=list)

    def count(self, verdict: Verdict) -> int:
        return sum(r.verdict is verdict for r in self.results)

    @property
    def gate_failed(self) -> bool:
        return bool(
            self.count(Verdict.UNCOVERED) or self.count(Verdict.AMBIGUOUS) or self.rejections
        )


def _site(call: dict) -> str:
    return f"{call['example']}:{call['line']}"


def _stub_overloads_of(
    overloads: dict, instrumented: dict, module: str, qualname: str
) -> list[str] | None:
    stub = overloads.get(module, {}).get(qualname)
    runtime = instrumented.get((module, qualname))
    if stub is not None and runtime is not None and runtime != len(stub):
        raise StubOverloadMismatchError(
            f"{module}.{qualname}: {runtime} overloads at runtime, {len(stub)} in the stub; "
            "regenerate the stubs (pixi run stubs)"
        )
    return stub


def classify(
    rows: list[dict],
    calls: list[dict],
    instrumented: dict[tuple[str, str], int],
    overloads: dict[str, dict[str, list[str]]],
    test_text: str,
) -> Report:
    """Decide every row's verdict.

    Args:
        rows: Inventory rows.
        calls: Recorded calls, all workers merged.
        instrumented: (module, qualname) → overload count, for everything the recorder wrapped.
        overloads: module → qualname → normalised stub overloads, in stub order.
        test_text: The text of the tests, for the X1/X3 test-reference check.

    Returns:
        The report.

    Raises:
        StubOverloadMismatchError: a recorded callable's overload count differs from its stub.
    """
    by_name: dict[tuple[str, str], list[tuple[dict, str | None, tuple[str, ...]]]] = {}
    rejections = []
    for call in calls:
        key = (call["module"], call["qualname"])
        if call["kind"] in CALLABLE_KINDS and call["index"] is None and not call["candidates"]:
            rejections.append(call)
            continue
        stub = _stub_overloads_of(overloads, instrumented, *key)
        hit = stub[call["index"]] if stub is not None and call["index"] is not None else None
        maybe = (
            tuple(stub[i] for i in call["candidates"])
            if stub is not None and call["index"] is None
            else ()
        )
        by_name.setdefault(key, []).append((call, hit, maybe))

    report = Report(
        rejections=sorted(rejections, key=lambda c: (c["module"], c["qualname"], _site(c)))
    )
    for row in rows:
        report.results.append(_verdict(row, by_name, instrumented, test_text))
    return report


def _verdict(row: dict, by_name: dict, instrumented: dict, test_text: str) -> Result:
    for rule in EXEMPTIONS:
        if not rule.matches(row):
            continue
        name = rule.referenced_name(row)
        if rule.needs_test_reference and not re.search(rf"\b{re.escape(name)}\b", test_text):
            return Result(
                row,
                Verdict.UNCOVERED,
                rule.code,
                f"exempt by {rule.code}, but no test names `{name}`",
            )
        return Result(row, Verdict.EXEMPT, rule.code)

    key = (row["module"], row["qualname"])
    kind = row["kind"]
    if kind in VALUE_KINDS:
        return Result(row, Verdict.UNCOVERED, note="a value, not a call: not observable at runtime")
    if kind == "class":
        prefix = row["qualname"] + "."
        sites = sorted(
            _site(c)
            for (module, qualname), entries in by_name.items()
            if module == row["module"]
            and (qualname.startswith(prefix) or qualname == row["qualname"])
            for c, _, _ in entries
        )
        return (
            Result(row, Verdict.COVERED, sites=tuple(sites))
            if sites
            else Result(row, Verdict.UNCOVERED)
        )
    if key not in instrumented:
        return Result(row, Verdict.UNCOVERED, note="not instrumented by the recorder")
    entries = by_name.get(key, [])
    if kind == "property":
        sites = sorted(_site(c) for c, _, _ in entries)
        return (
            Result(row, Verdict.COVERED, sites=tuple(sites))
            if sites
            else Result(row, Verdict.UNCOVERED)
        )

    full_binding = row["change"] in FULL_BINDING_CHANGES
    hits = [c for c, hit, _ in entries if hit == row["overload"]]
    covering = sorted(_site(c) for c in hits if c["fully_bound"] or not full_binding)
    if covering:
        return Result(row, Verdict.COVERED, sites=tuple(covering))
    maybe = sorted(_site(c) for c, _, cands in entries if row["overload"] in cands)
    if maybe:
        return Result(
            row, Verdict.AMBIGUOUS, note="the call also fits another overload", sites=tuple(maybe)
        )
    if hits:
        sites = tuple(sorted(_site(c) for c in hits))
        return Result(
            row, Verdict.UNCOVERED, note="called, but not with every parameter bound", sites=sites
        )
    return Result(row, Verdict.UNCOVERED)


def load_recordings(path: Path) -> tuple[list[dict], dict[tuple[str, str], int]]:
    """Merge every worker's recordings.

    Returns:
        The distinct calls, and (module, qualname) → overload count of everything instrumented.

    Raises:
        RecordingsMissingError: `path` holds no `*.json` recordings.
    """
    files = sorted(path.glob("*.json"))
    if not files:
        raise RecordingsMissingError(
            f"no recordings in {path}; run tests/examples with -p tests.examples.api_recorder"
        )
    calls: dict[str, dict] = {}
    instrumented: dict[tuple[str, str], int] = {}
    for file in files:
        data = json.loads(file.read_text())
        for call in data["calls"]:
            calls[json.dumps(call, sort_keys=True)] = call
        for item in data["instrumented"]:
            instrumented[(item["module"], item["qualname"])] = item["overloads"]
    return list(calls.values()), instrumented


def check_inventory_head(repo: Path, inventory: dict) -> None:
    """Raise unless the inventory was built for the repository's HEAD.

    Raises:
        StaleInventoryError: the inventory's head is not HEAD.
    """
    head = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=repo, capture_output=True, text=True, check=True
    )
    if inventory["head"] != head.stdout.strip():
        raise StaleInventoryError(
            f"inventory head {inventory['head'][:SHORT_SHA]} != HEAD {head.stdout.strip()[:SHORT_SHA]}; "
            "rerun scripts/api_inventory.py"
        )


def _entry(row: dict) -> str:
    module = row["module"].removeprefix(MODULE_PREFIX)
    return f"`{module}.{row['qualname']}{row['overload']}`"


def _issue(row: dict) -> str:
    m = ISSUE_REF.search(row["subject"])
    return f"#{m.group(1)}" if m else row["commit"]


def _sites(result: Result) -> str:
    shown = ", ".join(f"`{s}`" for s in result.sites[:SITES_SHOWN])
    more = len(result.sites) - SITES_SHOWN
    return shown + (f" (+{more})" if more > 0 else "")


def _table(header: list[str], lines: list[list[str]]) -> list[str]:
    if not lines:
        return ["None.", ""]
    out = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    out += ["| " + " | ".join(cells) + " |" for cells in lines]
    return [*out, ""]


FLOW = """```mermaid
flowchart LR
  stubs["committed .pyi stubs<br/>(base, HEAD)"] --> inventory["api_inventory.py"]
  inventory --> rows[("inventory rows")]
  examples["tests/examples"] -->|"-p api_recorder"| calls[("recorded calls<br/>(per overload)")]
  rows --> report["example_api_coverage.py"]
  calls --> report
  rules["exemption rules X1–X7"] --> report
  report --> page["this page"]
  report --> gate{"gate"}
  classDef runtime stroke-dasharray: 5 5
  class examples,calls runtime
```"""


def render(report: Report, base: str, head: str) -> str:
    """The coverage page: covered/total first, then every open entry, then the rest."""
    covered, uncovered = report.count(Verdict.COVERED), report.count(Verdict.UNCOVERED)
    ambiguous, exempt = report.count(Verdict.AMBIGUOUS), report.count(Verdict.EXEMPT)
    total = covered + uncovered + ambiguous
    by = {
        v: sorted(
            (r for r in report.results if r.verdict is v),
            key=lambda r: (r.row["module"], r.row["qualname"], r.row["overload"]),
        )
        for v in Verdict
    }
    out = [
        "# Example API coverage",
        "",
        f"Generated by `pixi run example-coverage` from the inventory `{base[:SHORT_SHA]}..{head[:SHORT_SHA]}`; do not edit by hand.",
        "",
        f"**{covered} / {total} non-exempt entries covered** by a shipped example: "
        f"{uncovered} uncovered, {ambiguous} ambiguous. {exempt} entries are exempt by rule."
        + (
            f" The overload matcher rejected {len(report.rejections)} calls nanobind ran."
            if report.rejections
            else ""
        ),
        "",
        "An entry is one overload of the public stub API added since 0.35.0.9. It is covered only by a call "
        "whose immediate caller is a file under `src/tesseract_robotics/examples/`, matched to exactly that "
        "overload; a new or changed overload also needs a call that binds every parameter. A call that fits "
        "more than one overload makes its candidates ambiguous, never covered. Dashed nodes run at test time.",
        "",
        FLOW,
        "",
        f"## Uncovered ({uncovered})",
        "",
        *_table(
            ["entry", "change", "added by", "note"],
            [
                [
                    _entry(r.row),
                    r.row["change"],
                    _issue(r.row),
                    r.note + (f" ({_sites(r)})" if r.sites else ""),
                ]
                for r in by[Verdict.UNCOVERED]
            ],
        ),
        f"## Ambiguous ({ambiguous})",
        "",
        *_table(
            ["entry", "change", "added by", "call sites"],
            [
                [_entry(r.row), r.row["change"], _issue(r.row), _sites(r)]
                for r in by[Verdict.AMBIGUOUS]
            ],
        ),
    ]
    if report.rejections:
        out += [
            f"## Matcher rejections ({len(report.rejections)})",
            "",
            "nanobind ran these calls, but the overload matcher found no overload that accepts them: a matcher defect.",
            "",
            *_table(
                ["module", "callable", "call site"],
                [[c["module"], f"`{c['qualname']}`", f"`{_site(c)}`"] for c in report.rejections],
            ),
        ]
    out += [
        f"## Covered ({covered})",
        "",
        *_table(
            ["entry", "change", "example"],
            [[_entry(r.row), r.row["change"], _sites(r)] for r in by[Verdict.COVERED]],
        ),
        f"## Exempt ({exempt})",
        "",
    ]
    for rule in EXEMPTIONS:
        rows = [r for r in by[Verdict.EXEMPT] if r.rule == rule.code]
        out += [f"**{rule.code}** ({len(rows)}): {rule.reason}", ""]
        if rows:
            out += [
                ", ".join(_entry(r.row) for r in rows),
                "",
            ]
    return "\n".join(out).rstrip() + "\n"


def _test_text(tests_root: Path, excluded: Path) -> str:
    return "\n".join(
        p.read_text() for p in sorted(tests_root.rglob("*.py")) if excluded not in p.parents
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--repo", type=Path, default=REPO_ROOT)
    parser.add_argument("--inventory", type=Path, default=DEFAULT_INVENTORY)
    parser.add_argument("--recordings", type=Path, default=DEFAULT_RECORDINGS)
    parser.add_argument("--out", type=Path, default=DEFAULT_REPORT)
    args = parser.parse_args(argv)
    repo = args.repo.resolve()

    inventory = json.loads((repo / args.inventory).read_text())
    check_inventory_head(repo, inventory)
    calls, instrumented = load_recordings(repo / args.recordings)
    modules = {r["module"] for r in inventory["rows"]} | {c["module"] for c in calls}
    overloads = {}
    for module in sorted(modules):
        stub = repo / STUB_ROOT / module / f"_{module}.pyi"
        if stub.exists():
            overloads[module] = stub_overloads(stub.read_text())
    report = classify(
        inventory["rows"],
        calls,
        instrumented,
        overloads,
        _test_text(repo / TESTS_ROOT, repo / TEST_REFERENCE_EXCLUDED),
    )

    out = repo / args.out
    out.write_text(render(report, inventory["base"], inventory["head"]))
    total = len(report.results) - report.count(Verdict.EXEMPT)
    print(
        f"{report.count(Verdict.COVERED)}/{total} covered, {report.count(Verdict.UNCOVERED)} uncovered, "
        f"{report.count(Verdict.AMBIGUOUS)} ambiguous, {report.count(Verdict.EXEMPT)} exempt, "
        f"{len(report.rejections)} matcher rejections -> {out.relative_to(repo)}"
    )
    return 1 if report.gate_failed else 0


if __name__ == "__main__":
    sys.exit(main())
