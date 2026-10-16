# Binding audit coverage and ownership

Every binding module is either audited or listed as unaudited, with a reason, and a run fails before parsing anything if a module is neither. The [Binding API Audit](binding-api-audit.md) report opens with that coverage ("N of M binding modules audited" and a table of the unaudited modules), and its statements cover only the audited modules. Each header belongs to exactly one module: the module with the longest prefix that matches the header path. The tables are in `scripts/audit_ownership.py`.

## What "audited" means

A module's audited API is every header it owns (spec amendment A2):

- a header its binding `#include`s directly is parsed with the binding and compared against the committed stub;
- a header it owns that its binding never includes is parsed in a synthetic TU (the binding plus that header) and shows as one `header` row. A header that does not compile on its own shows as an `unparseable header` row at its first error, and its API is not checked. Headers that compile alone but not together (two definitions of one name) fail the run with the combined error: no rows are reported for them;
- a header it owns that another binding includes but its own does not is audited with that module (`header_auditor`), and the owner's section lists it under "Headers audited with another module", with that module's status (`audited`, or `not yet:` and its `UNAUDITED_MODULES` reason);
- `UNAUDITED_HEADERS` (not API: test suites, plugin internals, serialization fragments) and `THIRD_PARTY_HEADERS` (vendored libraries) are skipped; each pattern is printed with its reason in the report.

Every other declaration in the TU only resolves Python names.

## The two module tables

| table | maps | meaning |
|---|---|---|
| `AUDITED_HEADER_PREFIX` | module → prefixes | audited; `pixi run -e audit audit-bindings` reports it |
| `UNAUDITED_MODULES` | module → (reason, prefixes) | not audited; the report lists it with the reason |

Every prefix ends with `/`. Both tables give prefixes, so header ownership does not depend on whether the owner is audited. A module may own several prefixes: `trajopt_ifopt` owns `trajopt_ifopt/` and `trajopt_common/` (it registers `TrajOptCollisionConfig`), and `tesseract_motion_planners_trajopt` owns `tesseract/motion_planners/trajopt/`, `trajopt_sco/` and `trajopt/`. A module with no headers of its own has an empty tuple (`ompl_base`, to be removed in #258).

Only the tesseract-robotics libraries are owned: `tesseract/`, `trajopt*/` and `descartes_light/`. Third-party roots that bindings include (`ompl/`, `OsqpEigen/`, `octomap/`, `cereal/`, `yaml-cpp/`, `boost/`) belong to no module and are never audited.

## Longest prefix wins

`tesseract_motion_planners` owns `tesseract/motion_planners/` and `tesseract_motion_planners_simple` owns `tesseract/motion_planners/simple/`. A header under both is the longer prefix's:

| header | owner |
|---|---|
| `tesseract/motion_planners/planner.h` | `tesseract_motion_planners` |
| `tesseract/motion_planners/simple/simple_motion_planner.h` | `tesseract_motion_planners_simple` |

The rule holds everywhere a header's owner matters: a module's own header walk (`prefix_headers`), its direct includes (`audited_headers`), delegation and `foreign_headers`. The parent module never audits, delegates or reports a header under a nested prefix. `UNAUDITED_HEADERS` patterns match the path below the owning prefix.

## Fail-loud checks

| exception | raised when | where |
|---|---|---|
| `UnclassifiedModuleError` | a binding module (`src/<m>/<m>_bindings.cpp`) is in neither table | `check_coverage`, first thing in `main` |
| `ConflictingCoverageError` | a module is in both tables | `check_coverage`, `owner_prefixes` |
| `UnknownModuleError` | a table entry is not a binding module, or the CLI names one | `check_coverage`, `resolve_module` |
| `UnauditedModuleError` | the CLI names a module from `UNAUDITED_MODULES`; the message carries the reason | `resolve_module` |
| `DuplicatePrefixError` | two modules claim the same prefix | `owner_prefixes` |
| `MalformedPrefixError` | a prefix does not end with `/` (`tesseract/x` would also own `tesseract/xy/`) | import of `scripts/audit_ownership.py`, `owner_prefixes` |
| `PrefixMismatchError` | `audit_tu` gets a module prefix that disagrees with that module's `prefixes` entry | `audit_tu`, before any parse |
| `HeaderParseError` | a binding TU does not compile, or unincluded headers conflict only together | `parse_tu`, `unincluded_header_gaps` |
| `IncludeDirError` | `include/ompl-<version>` is missing, or present more than once | import of `scripts/audit_bindings.py` |

A new binding module therefore fails the run until it gets a table entry.

## Moving a module from unaudited to audited

1. Move its entry from `UNAUDITED_MODULES` to `AUDITED_HEADER_PREFIX`, prefixes unchanged.
2. Regenerate the report: `pixi run -e audit audit-bindings`.
3. Give every new row (gap, deviation, quoted name, header or unparseable header) one outcome:
    - binding fix: an issue per fix, linking the closed issue whose defect class it repeats;
    - accepted rule: an `ACCEPTED` entry with its justification and a contract test in `tests/audit/`;
    - tool fix: an audit false positive, fixed with a failing test first;
    - not API: an `UNAUDITED_HEADERS` entry with its reason.
4. Gates: `pixi run -e audit audit-test`, and a regenerated report with no diff.

The JSON output (`--json PATH`) carries the same coverage as a top-level `coverage` key: `{"audited": [...], "unaudited": {module: reason}}`.
