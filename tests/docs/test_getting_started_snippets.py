"""Execute the ```python blocks of the user-facing pages, so a broken doc snippet fails CI.

Covers the README, the docs home, every page of getting-started, concepts, user-guide, api
and examples, and the two release pages `changes-0.36.md` and `breaking-changes.md`. Each page's blocks run in order in one shared namespace, as a reader pasting them top
to bottom would, and every failing block is reported, not just the first. Blocks inside tabs and
admonitions are dedented by their fence's indentation.

Reference pages show calls in isolation, so their namespace starts with `np` and the robot the
getting-started pages load (`locator`, `robot`, `env`) and then with the page's own ambient names,
declared once at the top of the page in an HTML comment that mkdocs and GitHub render as nothing:

    <!-- snippet: ambient
    program = ...
    -->

Two kinds of block are not run:

- A block whose only content is a `--8<--` include is the shipped example source, which the
  example tests run.
- A block directly under `<!-- snippet: skip (reason) -->` (an HTML comment: mkdocs and GitHub
  render nothing) is skipped. The reason is mandatory and states what the block needs that a
  test process cannot give it (a browser, a robot controller, a path of the reader's own).

Snippets carry their own asserts; this test adds no success checks of its own.
"""

import json
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
FIXTURES = Path(__file__).resolve().parent / "fixtures"
DOCS = ROOT / "docs"
PAGES = sorted(
    [
        ROOT / "README.md",
        DOCS / "index.md",
        *(DOCS / "getting-started").glob("*.md"),
        *(DOCS / "concepts").glob("*.md"),
        *(DOCS / "user-guide").glob("*.md"),
        *(DOCS / "api").glob("*.md"),
        *(DOCS / "examples").glob("*.md"),
        DOCS / "changes-0.36.md",
        DOCS / "breaking-changes.md",
    ]
)

SKIP_MARKER = re.compile(r"^<!-- snippet: skip \((?P<reason>.+)\) -->$")
AMBIENT_COMMENT = re.compile(r"<!-- snippet: ambient\n(?P<code>.*?)\n-->", re.DOTALL)
ANY_SKIP_MARKER = re.compile(r"^<!-- snippet: skip\b")
FENCE_OPEN = re.compile(r"^(?P<indent> *)```python(?:\s.*)?$")
FENCE_CLOSE = re.compile(r"^ *```\s*$")
INCLUDE = re.compile(r"^\s*--8<--")
# Reference pages show a call in isolation and assume the robot the getting-started pages load.
# Their namespace starts with `np` (numpy) and these three names: `locator`, `robot` and its `env`.
# Every other import stays in the snippets. The README, docs home, getting-started and concepts
# pages run bare.
AMBIENT_PAGES = frozenset(("user-guide", "api", "examples"))
# The release pages sit in `docs/` itself and show calls on the same robot. Each "before" block
# shows the removed 0.35 API and carries a skip marker; every "after" block runs.
AMBIENT_FILES = frozenset((DOCS / "changes-0.36.md", DOCS / "breaking-changes.md"))
AMBIENT_ROBOT = """
import numpy as np
from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.common import GeneralResourceLocator
from tesseract_robotics.planning import Robot

locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
robot = Robot.from_tesseract_support("abb_irb2400", locator)
env = robot.env
"""
# Generous upper bound per page (seconds): first plan pays ~15 s of plugin loading.
PAGE_TIMEOUT_S = 900

# Runs inside the subprocess: blocks compiled under "<page> block <i>" so a traceback
# names the page and block index that failed. Every block runs; the failures are reported
# together and the exit status is 1 when there were any.
# The pages show `TesseractViewer()` on its documented default port. A test must not depend on
# that port being free (a docs server, a running viewer, a parallel worker), so the child binds
# an ephemeral one: the default of `port`, and only in this process.
RUNNER = """
import inspect, json, sys, traceback
from tesseract_robotics.viewer.viewer import TesseractViewer
_init = TesseractViewer.__init__
_params = [p for p in inspect.signature(_init).parameters.values() if p.default is not p.empty]
_init.__defaults__ = tuple(0 if p.name == "port" else p.default for p in _params)
assert inspect.signature(TesseractViewer.__init__).parameters["port"].default == 0
namespace = {"__name__": "__main__"}
failures = 0
for label, code in json.loads(sys.argv[1]):
    try:
        exec(compile(code, label, "exec"), namespace)
    except BaseException:
        failures += 1
        traceback.print_exc()
sys.exit(1 if failures else 0)
"""


class Block:
    """One ```python fence: its index on the page, dedented source and skip reason."""

    def __init__(self, index: int, line: int, code: str, skip_reason: str | None) -> None:
        self.index = index
        self.line = line
        self.code = code
        self.skip_reason = skip_reason

    @property
    def is_include(self) -> bool:
        return all(INCLUDE.match(ln) or not ln.strip() for ln in self.code.splitlines())

    @property
    def skipped(self) -> bool:
        return self.skip_reason is not None or self.is_include


def extract_blocks(page: Path) -> list[Block]:
    """Return every ```python block of `page`, in order, dedented by its fence's indentation."""
    blocks: list[Block] = []
    lines = page.read_text(encoding="utf-8").splitlines()
    i = 0
    while i < len(lines):
        opened = FENCE_OPEN.match(lines[i])
        if opened is None:
            i += 1
            continue
        indent = len(opened["indent"])
        reason = None
        if i > 0:
            marker = SKIP_MARKER.match(lines[i - 1].strip())
            reason = marker["reason"] if marker else None
        end = i + 1
        while not FENCE_CLOSE.match(lines[end]):
            end += 1
        body = [ln[indent:] if ln[:indent].strip() == "" else ln for ln in lines[i + 1 : end]]
        blocks.append(Block(len(blocks), i + 1, "\n".join(body) + "\n", reason))
        i = end + 1
    return blocks


def skip_markers_without_reason(page: Path) -> list[int]:
    """Line numbers of skip markers that do not state a reason."""
    return [
        number
        for number, line in enumerate(page.read_text(encoding="utf-8").splitlines(), 1)
        if ANY_SKIP_MARKER.match(line.strip()) and not SKIP_MARKER.match(line.strip())
    ]


def run_page(page: Path) -> subprocess.CompletedProcess[str]:
    """Run the page's non-skipped blocks in one fresh interpreter."""
    # POSIX separators: the label is what a traceback prints, on every OS.
    name = page.relative_to(ROOT).as_posix()
    payload = [
        (f"{name} block {b.index} (line {b.line})", b.code)
        for b in extract_blocks(page)
        if not b.skipped
    ]
    if page.parent.name in AMBIENT_PAGES or page in AMBIENT_FILES:
        ambient = AMBIENT_COMMENT.search(page.read_text(encoding="utf-8"))
        if ambient is not None:
            payload.insert(0, (f"{name} ambient names", ambient["code"] + "\n"))
        payload.insert(0, (f"{name} ambient robot", AMBIENT_ROBOT))
    # Subprocess, not in-process exec: the pages load plugins, seed OMPL and start a
    # viewer server, all process-global; a crash in C++ must not take the pytest worker.
    # It runs in a scratch directory, because snippets write files (`state.xml`, `plan.viser`)
    # relative to the working directory; ROOT stays importable for the helpers in `doc_assets`.
    with tempfile.TemporaryDirectory(prefix="doc-snippets-") as scratch:
        return subprocess.run(
            # UTF-8 mode: on Windows the child would otherwise write tracebacks in the ANSI
            # code page, which cannot encode the pages' non-ASCII text.
            [sys.executable, "-X", "utf8", "-c", RUNNER, json.dumps(payload)],
            input="\n",
            capture_output=True,
            encoding="utf-8",
            cwd=scratch,
            env={
                **os.environ,
                "PYTHONPATH": os.pathsep.join([str(ROOT), os.environ.get("PYTHONPATH", "")]),
            },
            timeout=PAGE_TIMEOUT_S,
        )


@pytest.mark.planning
@pytest.mark.parametrize("page", PAGES, ids=lambda p: str(p.relative_to(ROOT)))
def test_page_snippets_run(page: Path) -> None:
    proc = run_page(page)
    assert proc.returncode == 0, f"{page.relative_to(ROOT)} snippets failed:\n{proc.stderr}"


@pytest.mark.parametrize("page", PAGES, ids=lambda p: str(p.relative_to(ROOT)))
def test_every_skip_states_its_reason(page: Path) -> None:
    assert skip_markers_without_reason(page) == [], (
        f"{page.relative_to(ROOT)}: write `<!-- snippet: skip (reason) -->`"
    )


def test_every_python_fence_of_the_covered_sections_is_extracted() -> None:
    """A fence the extractor misses would never run; count them against a plain scan."""
    for page in PAGES:
        scanned = sum(
            1
            for line in page.read_text(encoding="utf-8").splitlines()
            if line.strip().startswith("```python")
        )
        assert len(extract_blocks(page)) == scanned, page


@pytest.mark.planning
def test_runner_fails_on_unreachable_pose() -> None:
    """The runner must fail loudly, naming page and block, and honour the skip marker."""
    page = FIXTURES / "unreachable_pose.md"
    assert [b.skipped for b in extract_blocks(page)] == [False, True, False, True]
    proc = run_page(page)
    assert proc.returncode != 0
    assert 'File "tests/docs/fixtures/unreachable_pose.md block 2' in proc.stderr, proc.stderr
    assert "AssertionError" in proc.stderr, proc.stderr
    assert "skip-marked block was executed" not in proc.stderr, proc.stderr
    assert "include block was executed" not in proc.stderr, proc.stderr


def test_indented_blocks_are_dedented() -> None:
    page = FIXTURES / "unreachable_pose.md"
    indented = extract_blocks(page)[-1]
    assert indented.skipped  # the include block
    tabbed = next(b for b in extract_blocks(FIXTURES / "tabbed.md") if not b.skipped)
    compile(tabbed.code, "tabbed.md", "exec")


def test_the_runner_reports_every_failing_block() -> None:
    proc = run_page(FIXTURES / "two_failures.md")
    assert proc.returncode != 0
    assert proc.stderr.count("RuntimeError: first failure") == 1, proc.stderr
    assert proc.stderr.count("RuntimeError: second failure") == 1, proc.stderr
    assert "after the failure" not in proc.stderr
