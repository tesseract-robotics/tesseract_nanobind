"""What the audit spent its time on: one event per fact-cache fetch and per timed phase.

`BINDING_AUDIT_EVENTS` names a JSONL file; unset (local runs, tests), nothing is recorded.
Every process of a step appends to the same file (xdist workers included), one `os.write`
on an `O_APPEND` descriptor per event, so lines are whole under concurrent writers. The file
lives outside the cache directory: CI saves that directory, and a log inside it would be
restored into later runs (`audit_cache.EventsInCacheError`).

Two events:

```json
{"event": "fetch", "kind": "bound", "source": "/repo/src/a/a_bindings.cpp", "hit": false,
 "compute_s": 41.2, "wait_s": 0.0, "pid": 7}
{"event": "phase", "name": "cpp-api", "seconds": 12.8, "pid": 7}
```

`wait_s` is the time a fetch waited for the entry lock (another worker computing the same
fact), `compute_s` the extraction alone (0 on a hit), `source` the TU's main file. `render`
sums them per fact kind and per phase and lists each miss's source; `scripts/audit_telemetry_report.py` prints that per step.

Needs no libclang and imports no audit script.
"""

from __future__ import annotations

import json
import os
import time
from collections.abc import Generator, Iterable, Mapping, Sequence
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import cast

EVENTS_ENV = "BINDING_AUDIT_EVENTS"
LOG_SUFFIX = ".jsonl"
# Permission bits of a new event log: owner read/write, others read.
LOG_MODE = 0o644
FETCH = "fetch"
PHASE = "phase"
# Field -> the JSON types it may hold, per event. `bool` is an `int` in Python: fields that
# hold a number are checked against `bool` separately (`_is`).
EVENT_FIELDS: Mapping[str, Mapping[str, tuple[type, ...]]] = {
    FETCH: {
        "event": (str,),
        "kind": (str,),
        "source": (str,),
        "hit": (bool,),
        "compute_s": (int, float),
        "wait_s": (int, float),
        "pid": (int,),
    },
    PHASE: {"event": (str,), "name": (str,), "seconds": (int, float), "pid": (int,)},
}


class EventShapeError(RuntimeError):
    """A line of an event log is not an event `EventLog` writes."""


class NoEventsDirError(RuntimeError):
    """The directory of event logs does not exist: no step was given one."""


@dataclass(frozen=True)
class EventLog:
    """An append-only JSONL file of events, or nothing.

    Attributes:
        path: The file; `None` records nothing. Its directory must exist.
    """

    path: Path | None

    @classmethod
    def from_env(cls) -> EventLog:
        """The log `BINDING_AUDIT_EVENTS` names; no log when it is unset or empty."""
        value = os.environ.get(EVENTS_ENV)
        return cls(Path(value) if value else None)

    def fetched(
        self, kind: str, *, source: str, hit: bool, compute_s: float, wait_s: float
    ) -> None:
        """Record one answered fetch of a `kind` fact.

        Args:
            kind: The fact (`FactKey.kind`).
            source: The TU's main file (`FactKey.source`): which TU a miss parsed.
            hit: The stored entry answered.
            compute_s: Seconds in `compute`; 0 on a hit.
            wait_s: Seconds until the entry lock was held.
        """
        self._append(
            {
                "event": FETCH,
                "kind": kind,
                "source": source,
                "hit": hit,
                "compute_s": compute_s,
                "wait_s": wait_s,
            }
        )

    @contextmanager
    def timed(self, phase: str) -> Generator[None]:
        """Record the wall time of the body as one `phase` event, also when it raises."""
        start = time.perf_counter()
        try:
            yield
        finally:
            self._append({"event": PHASE, "name": phase, "seconds": time.perf_counter() - start})

    def _append(self, event: dict[str, object]) -> None:
        if self.path is None:
            return
        line = json.dumps({**event, "pid": os.getpid()}) + "\n"
        fd = os.open(self.path, os.O_WRONLY | os.O_APPEND | os.O_CREAT, LOG_MODE)
        try:
            os.write(fd, line.encode())
        finally:
            os.close(fd)


EVENTS = EventLog.from_env()


@dataclass(frozen=True)
class FetchTotal:
    """The fetches of one fact kind.

    Attributes:
        hits: Fetches a stored entry answered.
        misses: Fetches that computed.
        compute_s: Seconds in `compute`, summed.
        wait_s: Seconds waiting for entry locks, summed.
    """

    hits: int
    misses: int
    compute_s: float
    wait_s: float


@dataclass(frozen=True)
class PhaseTotal:
    """One timed phase.

    Attributes:
        calls: Times the phase ran.
        seconds: Its wall time, summed over calls and processes.
    """

    calls: int
    seconds: float


@dataclass(frozen=True)
class Totals:
    """An event log, summed.

    Attributes:
        fetches: Per fact kind.
        phases: Per phase name.
        processes: Distinct processes that wrote an event.
        miss_sources: Misses per fact kind and TU main file.
    """

    fetches: dict[str, FetchTotal]
    phases: dict[str, PhaseTotal]
    processes: int
    miss_sources: dict[tuple[str, str], int]


def _is(value: object, types: tuple[type, ...]) -> bool:
    return isinstance(value, types) and (bool in types or not isinstance(value, bool))


def checked_event(event: object, where: str) -> Mapping[str, object]:
    """`event` if it is one `EventLog` writes.

    Raises:
        EventShapeError: not an object, an unknown `event`, or a missing, extra or
            wrongly typed field.
    """
    if not isinstance(event, Mapping):
        raise EventShapeError(f"{where}: not a JSON object")
    fields = EVENT_FIELDS.get(str(event.get("event")))
    if fields is None:
        raise EventShapeError(f"{where}: unknown event {event.get('event')!r}")
    if set(event) != set(fields) or not all(_is(event[f], t) for f, t in fields.items()):
        raise EventShapeError(f"{where}: not a {event['event']} event: {dict(event)}")
    return event


def read_events(path: Path) -> list[Mapping[str, object]]:
    """Every event of the log at `path`, in file order.

    Raises:
        EventShapeError: a line is not JSON or not an event (`checked_event`).
    """
    events = []
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        where = f"{path}:{number}"
        try:
            event = json.loads(line)
        except json.JSONDecodeError as e:
            raise EventShapeError(f"{where}: {e}") from e
        events.append(checked_event(event, where))
    return events


def totals(events: Iterable[Mapping[str, object]]) -> Totals:
    """`events` summed per fact kind and per phase.

    Raises:
        EventShapeError: see `checked_event`.
    """
    fetches: dict[str, FetchTotal] = {}
    phases: dict[str, PhaseTotal] = {}
    miss_sources: dict[tuple[str, str], int] = {}
    pids = set()
    for index, raw in enumerate(events):
        event = checked_event(raw, f"event {index}")
        pids.add(event["pid"])
        if event["event"] == FETCH:
            kind, hit = cast(str, event["kind"]), cast(bool, event["hit"])
            old = fetches.get(kind, FetchTotal(0, 0, 0.0, 0.0))
            fetches[kind] = FetchTotal(
                hits=old.hits + hit,
                misses=old.misses + (not hit),
                compute_s=old.compute_s + cast(float, event["compute_s"]),
                wait_s=old.wait_s + cast(float, event["wait_s"]),
            )
            if not hit:
                where = (kind, cast(str, event["source"]))
                miss_sources[where] = miss_sources.get(where, 0) + 1
        else:
            name = cast(str, event["name"])
            old_phase = phases.get(name, PhaseTotal(0, 0.0))
            phases[name] = PhaseTotal(
                calls=old_phase.calls + 1,
                seconds=old_phase.seconds + cast(float, event["seconds"]),
            )
    return Totals(fetches=fetches, phases=phases, processes=len(pids), miss_sources=miss_sources)


def _aligned(rows: Sequence[Sequence[str]]) -> list[str]:
    """`rows` as text lines: first column left-aligned, the others right-aligned."""
    widths = [max(len(row[i]) for row in rows) for i in range(len(rows[0]))]
    return [
        "  ".join(
            cell.ljust(width) if i == 0 else cell.rjust(width)
            for i, (cell, width) in enumerate(zip(row, widths, strict=True))
        ).rstrip()
        for row in rows
    ]


def render(step: str, events: Sequence[Mapping[str, object]]) -> str:
    """The totals of one step's events as text tables, seconds to one decimal: fetches per
    kind, phases, and the misses per kind and source.

    Args:
        step: The step's name (the log's file stem).
        events: Its events.

    Raises:
        EventShapeError: see `checked_event`.
    """
    summed = totals(events)
    noun = "process" if summed.processes == 1 else "processes"
    lines = [f"{step}: {len(events)} events from {summed.processes} {noun}"]
    if summed.fetches:
        rows = [("kind", "hits", "misses", "compute", "wait")]
        rows += [
            (kind, str(t.hits), str(t.misses), f"{t.compute_s:.1f}", f"{t.wait_s:.1f}")
            for kind, t in sorted(summed.fetches.items())
        ]
        lines += _aligned(rows)
    if summed.phases:
        phase_rows = [("phase", "calls", "seconds")]
        phase_rows += [
            (name, str(t.calls), f"{t.seconds:.1f}") for name, t in sorted(summed.phases.items())
        ]
        lines += _aligned(phase_rows)
    if summed.miss_sources:
        miss_rows = [("miss", "count", "source")]
        miss_rows += [
            (kind, str(n), source) for (kind, source), n in sorted(summed.miss_sources.items())
        ]
        lines += _aligned(miss_rows)
    return "\n".join(lines)


def step_logs(directory: Path) -> list[Path]:
    """The event logs of a run, one per step that recorded an event, by name.

    Raises:
        NoEventsDirError: `directory` does not exist.
    """
    if not directory.is_dir():
        raise NoEventsDirError(f"{directory}: no such directory")
    return sorted(directory.glob(f"*{LOG_SUFFIX}"))
