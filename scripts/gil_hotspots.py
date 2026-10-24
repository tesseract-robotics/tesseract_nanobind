"""Rank the bindings that hold the GIL longest: candidates for nogil_entry_points.yaml (#124).

The list in scripts/nogil_entry_points.yaml was compiled by reading code; this measures.
`record` runs pytest and attaches `py-spy record --gil --format raw` to it: `--gil` keeps
only samples of the thread holding the GIL, so time inside a binding that releases it drops
out. The scripts/gil_call_counts.py plugin counts, in the same run, every binding call by the
Python line it is made from.

`report` attributes each sample to its innermost Python frame. A binding runs below the
Python line that called it, so a sample whose innermost Python line made binding calls is
time inside one of them; the counts name the binding (`Environment.getState`) and give the
calls to divide by. Python run from inside a binding (a trampoline, a callback) has its own
innermost frame, so it is Python time, not the binding's. Native frames (`--native`, Linux
x86_64 and Windows only) are not needed, so this runs on macOS too.

Two limits, both visible in the table: a line that calls two bindings reports them together,
and Python work on the same line as a binding call (building its arguments) counts toward it.
Read the calling line of a candidate before listing it.

py-spy attaches to another process: it runs under sudo (macOS, and Linux unless
`kernel.yama.ptrace_scope=0`), which asks for the password once; pytest runs as you.

Usage:
    pixi run gil-hotspots                              # record benchmarks + tests, report both
    python scripts/gil_hotspots.py record NAME -- PYTEST_ARGS...
    python scripts/gil_hotspots.py report build/gil-hotspots/NAME.txt ...
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
from collections import Counter, defaultdict
from collections.abc import Iterator, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

import pytest
from gil_call_counts import OUTPUT_ENV
from gil_entry_points import ENTRY_POINTS, MODULE_SCOPE, Release, load_entry_points

REPO_ROOT = Path(__file__).resolve().parent.parent
OUT_DIR = REPO_ROOT / "build" / "gil-hotspots"
RAW_SUFFIX = ".txt"
CALLS_GLOB = "{stem}.calls.*.json"

# Samples per second: py-spy's default, one sample per 10 ms.
DEFAULT_RATE_HZ = 100
MS_PER_S = 1000.0
US_PER_MS = 1000.0
# Rows printed per report.
DEFAULT_TOP = 40

# A frame's location: `file:line`, or a bare file.
LOCATION = re.compile(r"^(?P<file>.*?)(?::(?P<line>\d+))?$")
PYTHON_SUFFIX = ".py"
# The root frame `py-spy record --subprocesses` gives every stack.
PROCESS_PREFIX = "process "
PROCESS_RECORD = re.compile(r"\n(?=process \d+:\")")
SEPARATOR = " | "

# The runs `pixi run gil-hotspots` records, benchmarks first: real workloads, where test
# timings are dominated by setup.
RUNS: dict[str, tuple[str, ...]] = {
    "benchmarks": ("tests/benchmarks", "-q", "-p", "no:cacheprovider"),
    "tests": ("tests", "--ignore=tests/benchmarks", "-q", "-n", "auto", "-p", "no:cacheprovider"),
}

# (file, line): a Python call site.
Site = tuple[str, int]


class NoCallCountsError(RuntimeError):
    """A recording has no gil_call_counts output beside it: every sample would be unnamed."""


@dataclass(frozen=True)
class Frame:
    name: str
    file: str
    line: int

    @classmethod
    def parse(cls, text: str) -> Frame:
        """`name (file:line)`, `name (file)` or a bare `name`."""
        if not text.endswith(")") or " (" not in text:
            return cls(text, "", 0)
        name, location = text[:-1].rsplit(" (", 1)
        match = LOCATION.match(location)
        if match is None:
            raise ValueError(f"unparseable py-spy frame: {text!r}")
        return cls(name, match["file"], int(match["line"] or 0))

    @property
    def is_python(self) -> bool:
        return self.file.endswith(PYTHON_SUFFIX)


@dataclass(frozen=True)
class Row:
    binding: str
    samples: int
    total_ms: float
    calls: int | None
    mean_us: float | None


@dataclass(frozen=True)
class Report:
    rows: list[Row]
    total_samples: int
    binding_samples: int
    listed_samples: int


def read_raw(path: Path) -> Iterator[tuple[list[Frame], int]]:
    """py-spy's collapsed stacks: frames root first, `;`-joined, then the sample count.

    With `--subprocesses` each stack starts with a `process PID:"COMMAND"` frame, and the
    command keeps its newlines and semicolons (`python -c ...`): a record then runs up to the
    next process frame, and the command may split into junk frames on the root side. Only the
    innermost frames are read, so the junk is harmless.
    """
    text = path.read_text()
    records = PROCESS_RECORD.split(text) if text.startswith(PROCESS_PREFIX) else text.splitlines()
    for record in records:
        if not record.strip():
            continue
        stack, count = record.rstrip("\n").rsplit(" ", 1)
        yield [Frame.parse(f) for f in stack.split(";")], int(count)


def innermost_site(frames: Sequence[Frame]) -> Site | None:
    frame = next((f for f in reversed(frames) if f.is_python), None)
    return (frame.file, frame.line) if frame else None


def read_counts(raw: Path) -> list[tuple[str, int, str, str, int]]:
    """The gil_call_counts rows recorded beside `raw`, all processes."""
    paths = sorted(raw.parent.glob(CALLS_GLOB.format(stem=raw.stem)))
    if not paths:
        raise NoCallCountsError(f"{raw}: no {CALLS_GLOB.format(stem=raw.stem)} beside it")
    return [tuple(r) for p in paths for r in json.loads(p.read_text())]  # type: ignore[misc]


def release(binding: str) -> Release:
    """`tesseract_environment: Environment.getState` -> its nogil_entry_points.yaml key."""
    module, qualname = binding.split(": ", 1)
    scope, _, name = qualname.rpartition(".")
    return Release(module, scope or MODULE_SCOPE, name)


def build_report(raws: Sequence[Path], rate_hz: float, listed: Mapping[Release, str]) -> Report:
    bindings_at: dict[Site, set[str]] = defaultdict(set)
    calls: Counter[str] = Counter()
    for raw in raws:
        for file, line, module, qualname, n in read_counts(raw):
            binding = f"{module}: {qualname}"
            bindings_at[(file, line)].add(binding)
            calls[binding] += n

    samples: Counter[str] = Counter()
    total = in_bindings = in_listed = 0
    for raw in raws:
        for frames, n in read_raw(raw):
            total += n
            site = innermost_site(frames)
            names = sorted(bindings_at.get(site, ())) if site else []
            if not names:
                continue  # Python time
            in_bindings += n
            if all(release(b) in listed for b in names):
                in_listed += n
                continue
            samples[SEPARATOR.join(names)] += n

    rows = []
    for binding, n in samples.items():
        total_ms = n * MS_PER_S / rate_hz
        count = calls[binding] if binding in calls else None  # a joined row is not countable
        mean = total_ms * US_PER_MS / count if count else None
        rows.append(Row(binding, n, total_ms, count, mean))
    rows.sort(key=lambda r: (-r.samples, r.binding))
    return Report(rows, total, in_bindings, in_listed)


def render(report: Report, title: str, top: int) -> str:
    """A Markdown table, for the terminal and the PR description."""

    def pct(n: int) -> str:
        return f"{100.0 * n / report.total_samples:.1f}%" if report.total_samples else "-"

    unlisted = report.binding_samples - report.listed_samples
    out = [
        f"### {title}",
        "",
        f"{report.total_samples} GIL-held samples: {pct(unlisted)} in unlisted bindings, "
        f"{pct(report.listed_samples)} in listed ones (argument conversion before the release).",
        "",
        "| binding | total ms | samples | calls | mean µs/call |",
        "|---|---:|---:|---:|---:|",
    ]
    for r in report.rows[:top]:
        calls = str(r.calls) if r.calls is not None else "-"
        mean = f"{r.mean_us:.0f}" if r.mean_us is not None else "-"
        out.append(f"| `{r.binding}` | {r.total_ms:.0f} | {r.samples} | {calls} | {mean} |")
    return "\n".join(out)


def record(name: str, pytest_args: Sequence[str], rate_hz: int) -> tuple[Path, bool]:
    """Run pytest with the call counter, py-spy attached. Returns the recording and whether
    every test passed: failing tests still leave a valid profile. Raises when py-spy fails or
    pytest does not complete (interrupted, internal or usage error)."""
    py_spy = shutil.which("py-spy")
    if py_spy is None:
        raise FileNotFoundError("py-spy is not on PATH: run in the default pixi env")
    sudo = [] if os.geteuid() == 0 else ["sudo"]
    if sudo:
        subprocess.run(["sudo", "-v"], check=True)  # ask now, not while pytest already runs
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    raw = OUT_DIR / f"{name}{RAW_SUFFIX}"
    for stale in OUT_DIR.glob(CALLS_GLOB.format(stem=name)):
        stale.unlink()
    scripts = str(Path(__file__).resolve().parent)
    env = {
        **os.environ,
        "PYTHONPATH": os.pathsep.join(filter(None, [scripts, os.environ.get("PYTHONPATH")])),
        OUTPUT_ENV: str(OUT_DIR / f"{name}.calls"),
    }
    tests = subprocess.Popen(
        [sys.executable, "-m", "pytest", "-p", "gil_call_counts", *pytest_args],
        cwd=REPO_ROOT,
        env=env,
    )
    spy = subprocess.run(
        [
            *sudo, py_spy, "record", "--pid", str(tests.pid), "--gil", "--subprocesses",
            "--full-filenames", "--format", "raw", "--rate", str(rate_hz), "-o", str(raw),
        ],
        check=False,
    )  # fmt: skip
    if tests.wait() not in (pytest.ExitCode.OK, pytest.ExitCode.TESTS_FAILED):
        raise subprocess.CalledProcessError(tests.returncode, tests.args)
    spy.check_returncode()
    return raw, tests.returncode == pytest.ExitCode.OK


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--rate", type=int, default=DEFAULT_RATE_HZ, help="samples per second")
    parser.add_argument("--top", type=int, default=DEFAULT_TOP, help="rows per table")
    sub = parser.add_subparsers(dest="command", required=True)
    rec = sub.add_parser("record", help="record one pytest run")
    rec.add_argument("name")
    rec.add_argument("pytest_args", nargs=argparse.REMAINDER)
    rep = sub.add_parser("report", help="rank the bindings in recorded runs")
    rep.add_argument("raws", nargs="+", type=Path)
    sub.add_parser("all", help="record benchmarks and tests, then report each")
    opts = parser.parse_args(argv)

    if opts.command == "record":
        args = opts.pytest_args[1:] if opts.pytest_args[:1] == ["--"] else opts.pytest_args
        _, passed = record(opts.name, args, opts.rate)
        return 0 if passed else 1
    listed = load_entry_points(ENTRY_POINTS)
    failed: list[str] = []
    if opts.command == "all":
        raws = []
        for name, args in RUNS.items():
            raw, passed = record(name, args, opts.rate)
            raws.append(raw)
            if not passed:
                failed.append(name)
    else:
        raws = opts.raws
    for raw in raws:
        print(render(build_report([raw], opts.rate, listed), raw.stem, opts.top))
        print()
    if failed:
        print(
            f"tests failed in: {', '.join(failed)} (profiles above are still complete)",
            file=sys.stderr,
        )
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
