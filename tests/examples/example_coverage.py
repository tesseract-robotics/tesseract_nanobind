"""Every example is executed by the test suite, or the session fails naming it (#445).

A pytest plugin. Discovery is automatic: every module under the examples package
(`tesseract_robotics.examples`, recursively) is enrolled, with no list to maintain. A module
that defines a top-level `main` is an entry example: its `main` or `run` must execute during the
session. Any other module is a helper: at least one of its functions must execute. Importing a
module proves nothing, so module-level code does not count.

Execution is recorded with `sys.monitoring` (`PY_START`), on a free tool id that is released at
the end. Each code object reports once and is then disabled, so the cost is one callback per
function. Under pytest-xdist each worker sends its executed set to the controller, which merges
them and judges.

Activation: the gate judges only a session that collected every `test_*.py` file under the
example test directory (`tests/examples`) and deselected none of its tests (`-k`, `-m`,
`--testmon`, `--lf`). A single file, a node id or a filtered run is not judged, so targeted runs
never fail on examples they did not ask for. `pixi run test` and CI's `test-artifacts` collect
everything and are judged.

The package and the test directory are ini options (`example_coverage_package`,
`example_coverage_testdir`) so the gate's own test can point it at a synthetic package.
"""

from __future__ import annotations

import ast
import enum
import importlib.util
import pkgutil
import sys
from collections.abc import Iterable
from dataclasses import dataclass, field
from pathlib import Path
from types import CodeType
from typing import TypedDict

import pytest
from xdist.workermanage import WorkerController

PACKAGE_INI = "example_coverage_package"
TESTDIR_INI = "example_coverage_testdir"
DEFAULT_PACKAGE = "tesseract_robotics.examples"
DEFAULT_TESTDIR = "tests/examples"
# Entry points of an entry example: the console-script `main`, or the testable core `run`.
ENTRY_FUNCTIONS = frozenset({"main", "run"})
# Qualified name CPython gives a module's top-level code object.
MODULE_CODE = "<module>"
WORKER_OUTPUT_KEY = "example_coverage"
# sys.monitoring has tool ids 0..5; 0-2 and 5 are named (debugger, coverage, profiler,
# optimizer), so the unnamed ones are tried first.
TOOL_IDS = (3, 4, 2, 1, 0, 5)
TOOL_NAME = "example_coverage"

Executed = set[tuple[Path, str]]


class UntestedExampleError(Exception):
    """An example module did not execute during a full test session."""


class NoFreeMonitoringToolError(Exception):
    """Every `sys.monitoring` tool id is held by another tool."""


class ExampleKind(enum.Enum):
    """How an example module proves it ran."""

    ENTRY = "entry"
    """Defines `main`: `main` or `run` must execute."""
    HELPER = "helper"
    """Defines no `main`: any of its functions must execute."""


@dataclass(frozen=True)
class ExampleModule:
    """One enrolled module.

    Attributes:
        name: Dotted module name.
        path: Resolved source file.
        kind: Entry example or helper.
    """

    name: str
    path: Path
    kind: ExampleKind


def _kind(path: Path) -> ExampleKind:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    defines_main = any(
        isinstance(node, ast.FunctionDef) and node.name == "main" for node in tree.body
    )
    return ExampleKind.ENTRY if defines_main else ExampleKind.HELPER


def _package_location(package: str) -> Path:
    """The directory of `package` as the import system spells it, found without importing it."""
    spec = importlib.util.find_spec(package)
    if spec is None or not spec.submodule_search_locations:
        raise ModuleNotFoundError(f"examples package {package!r} not found")
    return Path(next(iter(spec.submodule_search_locations)))


def package_dir(package: str) -> Path:
    """The resolved directory of `package`, found without importing it."""
    return _package_location(package).resolve()


def discover(package: str) -> list[ExampleModule]:
    """Every module under `package`, recursively, without importing any of them.

    Package `__init__` modules are not enrolled: they hold no example.
    """
    found: list[ExampleModule] = []

    def walk(directory: Path, prefix: str) -> None:
        for info in pkgutil.iter_modules([str(directory)]):
            if info.ispkg:
                walk(directory / info.name, f"{prefix}{info.name}.")
                continue
            path = (directory / f"{info.name}.py").resolve()
            found.append(ExampleModule(f"{prefix}{info.name}", path, _kind(path)))

    walk(package_dir(package), f"{package}.")
    return sorted(found, key=lambda m: m.name)


def _proves_execution(module: ExampleModule, functions: set[str]) -> bool:
    if module.kind is ExampleKind.ENTRY:
        return bool(functions & ENTRY_FUNCTIONS)
    return bool(functions - {MODULE_CODE})


def assert_all_executed(modules: Iterable[ExampleModule], executed: Executed) -> None:
    """Raise naming every module whose execution `executed` does not prove.

    Args:
        modules: Enrolled modules.
        executed: (resolved source file, code object qualname) pairs that ran.

    Raises:
        UntestedExampleError: one or more modules did not execute; the message names each file.
    """
    by_file: dict[Path, set[str]] = {}
    for path, qualname in executed:
        by_file.setdefault(path, set()).add(qualname)
    missing = [m for m in modules if not _proves_execution(m, by_file.get(m.path, set()))]
    if not missing:
        return
    lines = [
        f"  {m.path}: "
        + ("main() / run() never ran" if m.kind is ExampleKind.ENTRY else "no function ran")
        for m in missing
    ]
    raise UntestedExampleError(
        f"{len(missing)} example module(s) not executed by the test suite. Every example is a "
        "test: add a test that calls it (see docs/developer/index.md, 'Every example is a "
        "test').\n" + "\n".join(lines)
    )


class WorkerReport(TypedDict):
    """What an xdist worker sends the controller."""

    complete: bool
    executed: list[list[str]]


def _free_tool_id() -> int:
    for tool in TOOL_IDS:
        if sys.monitoring.get_tool(tool) is None:
            return tool
    raise NoFreeMonitoringToolError("example coverage needs a free sys.monitoring tool id")


class _Recorder:
    """Records which functions defined under one directory start executing."""

    def __init__(self, root: Path) -> None:
        # `co_filename` spells the path as the import system does; a site-packages reached
        # through a symlink differs from its resolved form, so both are matched.
        self._prefixes = (str(root), str(root.resolve()))
        self.executed: set[tuple[str, str]] = set()
        self._tool = _free_tool_id()
        sys.monitoring.use_tool_id(self._tool, TOOL_NAME)
        sys.monitoring.register_callback(self._tool, sys.monitoring.events.PY_START, self._on_start)
        sys.monitoring.set_events(self._tool, sys.monitoring.events.PY_START)

    def _on_start(self, code: CodeType, _offset: int) -> object:
        if code.co_filename.startswith(self._prefixes):
            self.executed.add((code.co_filename, code.co_qualname))
        return sys.monitoring.DISABLE

    def stop(self) -> None:
        sys.monitoring.set_events(self._tool, sys.monitoring.events.NO_EVENTS)
        sys.monitoring.register_callback(self._tool, sys.monitoring.events.PY_START, None)
        sys.monitoring.free_tool_id(self._tool)


@dataclass
class _State:
    package: str
    testdir: Path
    recorder: _Recorder
    complete: bool = False
    deselected: bool = False
    stopped: bool = False
    worker_reports: list[WorkerReport] = field(default_factory=list)
    workers_down: int = 0
    verdict: str | None = None
    judged: int | None = None


STATE = pytest.StashKey[_State]()


def _under(path: Path, directory: Path) -> bool:
    return path.resolve().is_relative_to(directory)


def pytest_addoption(parser: pytest.Parser) -> None:
    parser.addini(PACKAGE_INI, "Package whose modules must all execute", default=DEFAULT_PACKAGE)
    parser.addini(
        TESTDIR_INI,
        "Test directory whose full collection activates the example coverage gate",
        default=DEFAULT_TESTDIR,
    )


def pytest_configure(config: pytest.Config) -> None:
    package = config.getini(PACKAGE_INI)
    config.stash[STATE] = _State(
        package=package,
        testdir=(config.rootpath / config.getini(TESTDIR_INI)).resolve(),
        recorder=_Recorder(_package_location(package)),
    )


def pytest_deselected(items: list[pytest.Item]) -> None:
    if not items:
        return
    state = items[0].config.stash[STATE]
    if any(_under(item.path, state.testdir) for item in items):
        state.deselected = True


def pytest_collection_finish(session: pytest.Session) -> None:
    state = session.config.stash[STATE]
    collected = {item.path.resolve() for item in session.items}
    expected = {p.resolve() for p in state.testdir.rglob("test_*.py")}
    state.complete = bool(expected) and expected <= collected and not state.deselected


def _stop(state: _State) -> None:
    if not state.stopped:
        state.recorder.stop()
        state.stopped = True


@pytest.hookimpl(optionalhook=True)
def pytest_testnodedown(node: WorkerController, error: object) -> None:
    state = node.config.stash[STATE]
    state.workers_down += 1
    # A worker that crashed sent no output; the session is then not judged complete.
    output: dict[str, WorkerReport] = getattr(node, "workeroutput", {})
    report = output.get(WORKER_OUTPUT_KEY)
    if report is not None:
        state.worker_reports.append(report)


def _is_worker(config: pytest.Config) -> bool:
    return hasattr(config, "workerinput")


def _is_controller(config: pytest.Config) -> bool:
    return config.pluginmanager.has_plugin("dsession")


def pytest_sessionfinish(session: pytest.Session) -> None:
    config = session.config
    state = config.stash[STATE]
    _stop(state)
    if _is_worker(config):
        # xdist sets `workeroutput` on a worker's config; it is not in pytest's types.
        output: dict[str, WorkerReport] = getattr(config, "workeroutput")  # noqa: B009
        output[WORKER_OUTPUT_KEY] = WorkerReport(
            complete=state.complete,
            executed=sorted([f, q] for f, q in state.recorder.executed),
        )
        return
    if _is_controller(config):
        complete = (
            state.workers_down > 0
            and len(state.worker_reports) == state.workers_down
            and all(r["complete"] for r in state.worker_reports)
        )
        raw = {(f, q) for r in state.worker_reports for f, q in r["executed"]}
    else:
        complete = state.complete
        raw = state.recorder.executed
    if not complete:
        return
    executed = {(Path(f).resolve(), q) for f, q in raw}
    modules = discover(state.package)
    state.judged = len(modules)
    try:
        assert_all_executed(modules, executed)
    except UntestedExampleError as error:
        state.verdict = str(error)
        session.exitstatus = pytest.ExitCode.TESTS_FAILED


def pytest_terminal_summary(terminalreporter, exitstatus: int, config: pytest.Config) -> None:
    state = config.stash[STATE]
    if state.verdict is not None:
        terminalreporter.section("example coverage (#445)", sep="=", red=True, bold=True)
        terminalreporter.line(state.verdict, red=True)
    elif state.judged is not None:
        terminalreporter.line(f"example coverage: all {state.judged} example modules executed")


def pytest_unconfigure(config: pytest.Config) -> None:
    if STATE in config.stash:
        _stop(config.stash[STATE])
