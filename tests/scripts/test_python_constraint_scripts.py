"""scripts/bench_python_constraints.py and plot_python_constraint_effect.py time one SQP step per tick."""

import sys
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parents[2] / "scripts"
sys.path.insert(0, str(SCRIPTS))

import bench_python_constraints as bench  # noqa: E402
import plot_python_constraint_effect as plot  # noqa: E402


class _Stop(Exception):
    """Raised by the stand-in run() once it has recorded its arguments."""


@pytest.mark.parametrize(
    "call",
    [
        pytest.param(lambda: bench.run_once(None, 2, False), id="bench"),
        pytest.param(lambda: plot.series(None, replans=2), id="plot"),
    ],
)
def test_scripts_request_a_single_sqp_step_per_tick(monkeypatch, call):
    seen = {}

    def fake_run(**kwargs):
        seen.update(kwargs)
        raise _Stop

    monkeypatch.setattr(bench.ex, "run", fake_run)
    with pytest.raises(_Stop):
        call()
    assert seen["sqp_steps_per_tick"] == 1
