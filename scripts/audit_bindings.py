"""Audit each binding's C++ API against its committed Python stub.

For each extension module, libclang parses `src/<module>/<module>_bindings.cpp`
(exactly the headers and macros the binding sees) and `ast` parses the
committed `_<module>.pyi` and the package `__init__.py`. Every header under the
module's prefix is audited: one the binding never includes is parsed in a
synthetic TU (the binding plus that header) and reported as one `header` row.
No extension module is imported. Gaps (C++ without Python), deviations (Python without C++) and
accepted deviations go to `docs/developer/binding-api-audit.md`.

Usage:
    pixi run -e audit audit-bindings [MODULE ...] [--json PATH]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from audit_match import (
    REPORT_PATH,
    audit_module,
    provenance,
    render_markdown,
    resolve_module,
    to_json,
)
from audit_ownership import (
    AUDITED_HEADER_PREFIX,
    REPO_ROOT,
    binding_modules,
    check_coverage,
    coverage,
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "modules", nargs="*", default=sorted(AUDITED_HEADER_PREFIX),
        help="short module names (default: every module with an AUDITED_HEADER_PREFIX entry)",
    )  # fmt: skip
    parser.add_argument("--json", type=Path, help="also write the findings as JSON")
    args = parser.parse_args(argv)

    check_coverage(binding_modules())
    modules = sorted(resolve_module(m) for m in args.modules)
    reports = [audit_module(m) for m in modules]
    prov = provenance(modules)
    cov = coverage()
    REPORT_PATH.write_text(render_markdown(reports, prov, cov), encoding="utf-8")
    if args.json:
        args.json.write_text(to_json(reports, prov, cov), encoding="utf-8")
    print(f"{len(reports)} modules → {REPORT_PATH.relative_to(REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
