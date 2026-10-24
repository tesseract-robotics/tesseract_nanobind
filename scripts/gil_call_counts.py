"""pytest plugin: count the calls into tesseract_robotics bindings, per call site (#124).

scripts/gil_hotspots.py records a pytest run with py-spy, which samples time but cannot
count calls. This plugin counts them in the same run, keyed by the calling Python line and
the binding's Python name, so the report can divide GIL-held time by calls and name each
binding the way scripts/nogil_entry_points.yaml does. It counts, it does not time:
sys.monitoring CALL events, filtered to nanobind functions of tesseract_robotics.

Load with `-p gil_call_counts` and scripts/ on PYTHONPATH; set GIL_CALL_COUNTS to an output
prefix. Each process (xdist workers too) writes `<prefix>.<pid>.json`.
"""

from __future__ import annotations

import json
import os
import sys
from collections import Counter
from pathlib import Path
from types import CodeType

OUTPUT_ENV = "GIL_CALL_COUNTS"
# nanobind's function types: module functions, unbound and bound methods. Constructors (calls of
# an nb_type) are left out: they never qualify for a GIL release.
NB_FUNCTION_TYPES = frozenset({"nb_func", "nb_method", "nb_bound_method"})
PACKAGE_PREFIX = "tesseract_robotics."
TOOL_ID = sys.monitoring.PROFILER_ID
TOOL_NAME = "gil_call_counts"

# (file, line, module, qualname): file and line of the calling Python frame, module as named in
# nogil_entry_points.yaml (`tesseract_environment`), qualname as nanobind reports it
# (`Environment.getState`, or `parseURDFFile` for a module function).
CallSite = tuple[str, int, str, str]


class CallCounter:
    """Counts nanobind binding calls by call site."""

    def __init__(self) -> None:
        self.counts: Counter[CallSite] = Counter()
        self._lines: dict[CodeType, dict[int, int]] = {}

    def on_call(self, code: CodeType, offset: int, func: object, arg0: object) -> None:
        kind = type(func)
        if kind.__module__ != "nanobind" or kind.__name__ not in NB_FUNCTION_TYPES:
            return
        module = getattr(func, "__module__", "")
        if not module.startswith(PACKAGE_PREFIX):
            return
        extension = module.rsplit(".", 1)[-1].removeprefix("_")
        site = (code.co_filename, self._line(code, offset), extension, func.__qualname__)
        self.counts[site] += 1

    def _line(self, code: CodeType, offset: int) -> int:
        lines = self._lines.get(code)
        if lines is None:
            lines = {
                o: line
                for start, end, line in code.co_lines()
                if line is not None
                for o in range(start, end, 2)
            }
            self._lines[code] = lines
        return lines.get(offset, code.co_firstlineno)

    def write(self, path: Path) -> None:
        rows = [[*site, n] for site, n in sorted(self.counts.items())]
        path.write_text(json.dumps(rows))


_counter: CallCounter | None = None


def pytest_configure(config: object) -> None:
    global _counter
    if OUTPUT_ENV not in os.environ:
        raise RuntimeError(f"gil_call_counts: set {OUTPUT_ENV} to an output prefix")
    _counter = CallCounter()
    sys.monitoring.use_tool_id(TOOL_ID, TOOL_NAME)
    sys.monitoring.register_callback(TOOL_ID, sys.monitoring.events.CALL, _counter.on_call)
    sys.monitoring.set_events(TOOL_ID, sys.monitoring.events.CALL)


def pytest_unconfigure(config: object) -> None:
    if _counter is None:
        raise RuntimeError("gil_call_counts: unconfigure without configure")
    sys.monitoring.set_events(TOOL_ID, sys.monitoring.events.NO_EVENTS)
    sys.monitoring.free_tool_id(TOOL_ID)
    _counter.write(Path(f"{os.environ[OUTPUT_ENV]}.{os.getpid()}.json"))
