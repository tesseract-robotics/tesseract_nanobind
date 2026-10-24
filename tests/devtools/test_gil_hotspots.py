"""Contract tests of scripts/gil_hotspots.py and scripts/gil_call_counts.py (#124).

Dev tooling on Python >= 3.12 (sys.monitoring), outside the wheel test suite:
`pixi run devtools-test`.

The raw lines follow py-spy's collapsed-stack format (root first, `;`-joined frames
`name (file:line)`, then the sample count), as `py-spy record --full-filenames --format raw`
writes it.
"""

import importlib.util
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
SCRIPTS = REPO_ROOT / "scripts"


def _load_script(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module  # dataclasses resolve annotations via sys.modules
    spec.loader.exec_module(module)
    return module


_load_script("gil_entry_points")
_load_script("gil_call_counts")
hot = _load_script("gil_hotspots")

TEST = "/repo/tests/test_x.py"
PYTEST = "<module> (/env/bin/pytest:8);main (/env/lib/python3.14/site-packages/_pytest/config/__init__.py:180)"


def _stack(*frames):
    return ";".join((PYTEST, *frames))


def _raw(tmp_path, lines, counts):
    path = tmp_path / "run.txt"
    path.write_text("".join(f"{stack} {n}\n" for stack, n in lines))
    (tmp_path / "run.calls.123.json").write_text(json.dumps([list(c) for c in counts]))
    return path


def test_a_sample_at_a_binding_call_line_is_time_in_that_binding(tmp_path):
    raw = _raw(
        tmp_path,
        [(_stack(f"test_a ({TEST}:10)"), 40)],
        counts=[(TEST, 10, "tesseract_environment", "Environment.setState", 8)],
    )

    (row,) = hot.build_report([raw], rate_hz=1000, listed={}).rows

    assert row.binding == "tesseract_environment: Environment.setState"
    assert row.samples == 40
    assert row.total_ms == pytest.approx(40.0)
    assert row.calls == 8
    assert row.mean_us == pytest.approx(5000.0)


def test_python_samples_are_not_binding_time(tmp_path):
    raw = _raw(
        tmp_path,
        [
            (_stack(f"t ({TEST}:3)", f"helper ({TEST}:20)"), 50),
            # A Python callback run by the binding called at line 3: its own frame is innermost.
            (_stack(f"t ({TEST}:3)", f"locateResource ({TEST}:40)"), 7),
        ],
        counts=[(TEST, 3, "tesseract_urdf", "parseURDFFile", 1)],
    )

    report = hot.build_report([raw], rate_hz=100, listed={})

    assert report.rows == []
    assert report.total_samples == 57
    assert report.binding_samples == 0


def test_listed_bindings_are_dropped(tmp_path):
    raw = _raw(
        tmp_path,
        [(_stack(f"t ({TEST}:3)"), 5)],
        counts=[(TEST, 3, "tesseract_environment", "Environment.getState", 1)],
    )
    listed = {hot.Release("tesseract_environment", "Environment", "getState"): "why"}

    report = hot.build_report([raw], rate_hz=100, listed=listed)

    assert report.rows == []
    assert report.listed_samples == 5


def test_module_function_maps_to_the_module_scope(tmp_path):
    raw = _raw(
        tmp_path,
        [(_stack(f"t ({TEST}:3)"), 5)],
        counts=[(TEST, 3, "tesseract_kinematics", "checkKinematics", 1)],
    )
    listed = {hot.Release("tesseract_kinematics", "module", "checkKinematics"): "why"}

    assert hot.build_report([raw], rate_hz=100, listed=listed).rows == []


def test_two_bindings_on_one_line_are_reported_together(tmp_path):
    raw = _raw(
        tmp_path,
        [(_stack(f"t ({TEST}:3)"), 5)],
        counts=[
            (TEST, 3, "tesseract_environment", "Environment.getState", 1),
            (TEST, 3, "tesseract_kinematics", "KinematicGroup.getJointNames", 1),
        ],
    )

    (row,) = hot.build_report([raw], rate_hz=100, listed={}).rows

    assert row.binding == (
        "tesseract_environment: Environment.getState | tesseract_kinematics: KinematicGroup.getJointNames"
    )
    assert row.mean_us is None  # which of the two took the time is unknown


def test_calls_sum_over_every_call_site(tmp_path):
    raw = _raw(
        tmp_path,
        [(_stack(f"t ({TEST}:3)"), 10)],
        counts=[
            (TEST, 3, "tesseract_environment", "Environment.setState", 2),
            (TEST, 9, "tesseract_environment", "Environment.setState", 3),
        ],
    )

    (row,) = hot.build_report([raw], rate_hz=100, listed={}).rows

    assert row.calls == 5
    assert row.mean_us == pytest.approx(20_000.0)  # 10 samples at 100 Hz = 100 ms, over 5 calls


def test_rows_rank_by_total_time(tmp_path):
    raw = _raw(
        tmp_path,
        [(_stack(f"t ({TEST}:3)"), 5), (_stack(f"t ({TEST}:4)"), 9)],
        counts=[
            (TEST, 3, "tesseract_environment", "Environment.a", 1),
            (TEST, 4, "tesseract_environment", "Environment.b", 1),
        ],
    )

    rows = hot.build_report([raw], rate_hz=100, listed={}).rows

    assert [r.binding for r in rows] == [
        "tesseract_environment: Environment.b",
        "tesseract_environment: Environment.a",
    ]


def test_a_recording_without_call_counts_fails_loudly(tmp_path):
    raw = tmp_path / "run.txt"
    raw.write_text(f"{_stack(f't ({TEST}:3)')} 5\n")

    with pytest.raises(hot.NoCallCountsError):
        hot.build_report([raw], rate_hz=100, listed={})


def test_call_counter_counts_binding_calls_by_call_site(tmp_path):
    script = tmp_path / "test_counted.py"
    script.write_text(
        "from tesseract_robotics import tesseract_common\n"
        "def test_calls():\n"
        "    for _ in range(3):\n"
        "        tesseract_common.Isometry3d.Identity()\n"
    )
    prefix = tmp_path / "counts"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "pytest",
            "-q",
            "-p",
            "gil_call_counts",
            "--noconftest",
            "-p",
            "no:cacheprovider",
            str(script),
        ],
        check=True,
        cwd=tmp_path,
        env={**os.environ, "PYTHONPATH": str(SCRIPTS), "GIL_CALL_COUNTS": str(prefix)},
    )

    (out,) = tmp_path.glob("counts.*.json")
    rows = json.loads(out.read_text())
    assert [str(script), 4, "tesseract_common", "Isometry3d.Identity", 3] in rows


def test_a_subprocess_command_with_newlines_stays_one_record(tmp_path):
    # `py-spy record --subprocesses` roots each stack in `process PID:"COMMAND"`; a `python -c`
    # command keeps its newlines, so one record spans several lines.
    process = 'process 7:"python -c import sys\nfrom pathlib import Path\nrun()"'
    raw = _raw(
        tmp_path,
        [
            (f"{process};{_stack(f't ({TEST}:3)')}", 5),
            (f'process 8:"python -m pytest";{_stack(f"t ({TEST}:4)")}', 2),
        ],
        counts=[
            (TEST, 3, "tesseract_environment", "Environment.a", 1),
            (TEST, 4, "tesseract_environment", "Environment.b", 1),
        ],
    )

    rows = hot.build_report([raw], rate_hz=100, listed={}).rows

    assert {r.binding: r.samples for r in rows} == {
        "tesseract_environment: Environment.a": 5,
        "tesseract_environment: Environment.b": 2,
    }
