"""Execute the ```python blocks of the user-facing pages, so a broken doc snippet fails CI.

Each page's blocks run in order in one shared namespace, as a reader pasting them
top to bottom would. A block is skipped only when the line directly above its fence
is `<!-- snippet: skip -->` (an HTML comment: mkdocs and GitHub render nothing).
Snippets carry their own asserts; this test adds no success checks of its own.
"""

import json
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
FIXTURES = Path(__file__).resolve().parent / "fixtures"
PAGES = sorted(
    [
        ROOT / "README.md",
        ROOT / "docs" / "index.md",
        *(ROOT / "docs" / "getting-started").glob("*.md"),
    ]
)

SKIP_MARKER = "<!-- snippet: skip -->"
FENCE_OPEN = "```python"
FENCE_CLOSE = "```"
# Generous upper bound per page (seconds): first plan pays ~15 s of plugin loading.
PAGE_TIMEOUT_S = 600

# Runs inside the subprocess: blocks compiled under "<page> block <i>" so a traceback
# names the page and block index that failed.
RUNNER = """
import json, sys
namespace = {"__name__": "__main__"}
for label, code in json.loads(sys.argv[1]):
    exec(compile(code, label, "exec"), namespace)
"""


def extract_blocks(page: Path) -> list[tuple[int, str, bool]]:
    """Return `(index, code, skipped)` for every ```python block of `page`, in order."""
    blocks: list[tuple[int, str, bool]] = []
    lines = page.read_text(encoding="utf-8").splitlines()
    i = 0
    while i < len(lines):
        if lines[i].strip() == FENCE_OPEN:
            skipped = i > 0 and lines[i - 1].strip() == SKIP_MARKER
            end = i + 1
            while lines[end].strip() != FENCE_CLOSE:
                end += 1
            blocks.append((len(blocks), "\n".join(lines[i + 1 : end]) + "\n", skipped))
            i = end
        i += 1
    return blocks


def run_page(page: Path) -> subprocess.CompletedProcess[str]:
    """Run the page's non-skipped blocks in one fresh interpreter."""
    # POSIX separators: the label is what a traceback prints, on every OS.
    name = page.relative_to(ROOT).as_posix()
    payload = [
        (f"{name} block {i}", code) for i, code, skipped in extract_blocks(page) if not skipped
    ]
    # Subprocess, not in-process exec: the pages load plugins, seed OMPL and start a
    # viewer server, all process-global; a crash in C++ must not take the pytest worker.
    return subprocess.run(
        # UTF-8 mode: on Windows the child would otherwise write tracebacks in the ANSI
        # code page, which cannot encode the pages' non-ASCII text.
        [sys.executable, "-X", "utf8", "-c", RUNNER, json.dumps(payload)],
        input="\n",  # quickstart's viewer block waits on input("Press Enter to exit...")
        capture_output=True,
        encoding="utf-8",
        cwd=ROOT,
        timeout=PAGE_TIMEOUT_S,
    )


@pytest.mark.planning
@pytest.mark.parametrize("page", PAGES, ids=lambda p: str(p.relative_to(ROOT)))
def test_page_snippets_run(page: Path) -> None:
    proc = run_page(page)
    assert proc.returncode == 0, f"{page.relative_to(ROOT)} snippets failed:\n{proc.stderr}"


@pytest.mark.planning
def test_runner_fails_on_unreachable_pose() -> None:
    """The runner must fail loudly, naming page and block, and honour the skip marker."""
    page = FIXTURES / "unreachable_pose.md"
    assert [skipped for _, _, skipped in extract_blocks(page)] == [False, True, False]
    proc = run_page(page)
    assert proc.returncode != 0
    assert 'File "tests/docs/fixtures/unreachable_pose.md block 2"' in proc.stderr, proc.stderr
    assert "AssertionError" in proc.stderr, proc.stderr
    assert "skip-marked block was executed" not in proc.stderr, proc.stderr
