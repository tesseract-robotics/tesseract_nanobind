"""Inner pytest session of the recorder tests: runs the fixture example under the plugin.

Collected only with `-o python_files=drive_*.py`, so the outer suite never runs it.
The example directory comes from `--api-record-example-root`, which lets the
ablation test point it at an edited copy.
"""

import importlib
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def _run_example(example_root: Path):
    sys.path[:0] = [str(example_root), str(HERE)]
    return importlib.import_module("fixture_example").run()


def test_fixture_example(request):
    _run_example(Path(request.config.getoption("--api-record-example-root")))


def test_fixture_example_again(request):
    """A second test, so an `-n 2` run puts the example on two xdist workers."""
    _run_example(Path(request.config.getoption("--api-record-example-root")))
