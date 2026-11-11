"""Drop the fact-cache entries no fetch used since a stamp file was touched (`audit_cache`).

CI touches the stamp when the audit run starts and prunes before saving the cache, so the
cache holds what the last complete run used and nothing older.

Usage:
    pixi run -e audit audit-cache-prune CACHE_DIR STAMP
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from audit_cache import ENTRY_SUFFIX, FactCache


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("root", type=Path, help="the cache directory")
    parser.add_argument("stamp", type=Path, help="a file touched when the run started")
    args = parser.parse_args(argv)
    removed = FactCache(args.root).prune(args.stamp.stat().st_mtime_ns)
    kept = len(list(args.root.glob(f"*/*{ENTRY_SUFFIX}")))
    print(f"fact cache: kept {kept} entries, dropped {len(removed)} unused")
    return 0


if __name__ == "__main__":
    sys.exit(main())
