"""Print what each audit step fetched from the fact cache and spent in its timed phases
(`audit_telemetry`): one table per event log in a directory, by file name.

Usage:
    pixi run -e audit audit-telemetry EVENTS_DIR
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from audit_telemetry import read_events, render, step_logs


def main(argv: list[str] | None = None) -> int:
    """Print one table per `*.jsonl` in the directory.

    Raises:
        NoEventsDirError: the directory does not exist.
        EventShapeError: a log holds a line that is not an event.
    """
    parser = argparse.ArgumentParser(description=(__doc__ or "").splitlines()[0])
    parser.add_argument("events", type=Path, help="the directory of the steps' event logs")
    args = parser.parse_args(argv)
    logs = step_logs(args.events)
    for log in logs:
        print(render(log.stem, read_events(log)), end="\n\n")
    if not logs:
        print(f"{args.events}: no step recorded an event")
    return 0


if __name__ == "__main__":
    sys.exit(main())
