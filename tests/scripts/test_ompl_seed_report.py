"""scripts/ompl_seed_report.py: the per-seed OMPL / TrajOptIfopt report of the clearance example (#362)."""

import json
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
SCRIPTS = REPO / "scripts"
sys.path.insert(0, str(SCRIPTS))

import ompl_seed_report as report  # noqa: E402

# The remap check plans the threshold case with one planner: two parallel planners race, so
# only a single planner reproduces on a seed (the plan's RNG note). The threshold ends the
# optimize loop, so the time cap never decides the plan; this cap only bounds a failed run (s).
REMAP_CHECK_SEED = 1
REMAP_CHECK_PLANNERS = 1
REMAP_CHECK_PLANNING_TIME_S = 30.0

_REMAP_CHILD = textwrap.dedent(
    """
    import importlib, json, sys
    sys.path.insert(0, {scripts!r})
    import ompl_seed_report as report
    from tesseract_robotics.planning import TaskComposer
    from tesseract_robotics.tesseract_motion_planners_ompl import RNG_setSeed

    ex = importlib.import_module("tesseract_robotics.examples.freespace_clearance_objective_example")
    variant = sys.argv[1] == "variant"
    composer = (
        TaskComposer.from_config(report.VARIANT_CONFIG) if variant else TaskComposer.from_config()
    )
    pipeline = report.VARIANT_PIPELINE if variant else ex.PIPELINE
    composer.warmup([pipeline])
    RNG_setSeed({seed})
    case = ex.run_case(
        "threshold",
        ex.LengthAndClearanceProfile(ex.PATH_LENGTH_THRESHOLD_RAD),
        {planning_time},
        composer=composer,
        pipeline=pipeline,
        num_planners={planners},
    )
    final = [
        {{
            "positions": [float(v).hex() for v in point.positions],
            "velocities": None if point.velocities is None else [float(v).hex() for v in point.velocities],
            "accelerations": None
            if point.accelerations is None
            else [float(v).hex() for v in point.accelerations],
            "time": None if point.time is None else float(point.time).hex(),
        }}
        for point in case["result"]
    ]
    try:
        ompl_states = len(report.program_positions(report.ompl_program(case["result"])))
    except report.OMPLProgramMissingError:
        ompl_states = None
    print(json.dumps({{"final": final, "ompl_states": ompl_states}}))
    """
)


def _remap_child(arm: str) -> dict:
    code = _REMAP_CHILD.format(
        scripts=str(SCRIPTS),
        seed=REMAP_CHECK_SEED,
        planning_time=REMAP_CHECK_PLANNING_TIME_S,
        planners=REMAP_CHECK_PLANNERS,
    )
    proc = subprocess.run(
        [sys.executable, "-c", code, arm], capture_output=True, text=True, check=False
    )
    assert proc.returncode == 0, proc.stderr
    return json.loads(proc.stdout.strip().splitlines()[-1])


@pytest.mark.planning
def test_ompl_output_variant_is_bit_identical_to_stock():
    """The variant pipeline only re-keys the OMPL -> TrajOptIfopt hand-off: same final program.

    Each arm is a fresh process seeded before it plans. Stock runs twice as the control: a
    difference there would be a reproducibility problem, not the remap's.
    """
    stock_a = _remap_child("stock")
    stock_b = _remap_child("stock")
    variant = _remap_child("variant")

    assert stock_a["final"] == stock_b["final"], "control: stock does not reproduce on the seed"
    assert variant["final"] == stock_a["final"]
    # The OMPL program is readable under the variant's key, and absent from stock's storage.
    assert stock_a["ompl_states"] is None
    # TrajOptIfopt optimizes the OMPL states one for one, so the counts match.
    assert variant["ompl_states"] == len(variant["final"])


def _row(seed: int, case: str) -> report.SeedRow:
    return report.SeedRow(
        seed=seed,
        case=case,
        ompl_wall_time_s=1.0,
        ompl_min_clearance_m=0.03,
        trajopt_iterations=5,
        trajopt_status="Found valid solution",
        final_min_clearance_m=0.04,
        final_length_rad=3.5,
        total_wall_time_s=2.0,
    )


def test_to_markdown_has_one_row_per_seed_and_case():
    rows = [_row(seed, case) for seed in (1, 2) for case in ("default", "threshold")]

    table = report.to_markdown(rows, commit="abc1234", platform="Darwin arm64")

    lines = table.splitlines()
    data_rows = [line for line in lines if line.startswith("| ") and line[2].isdigit()]
    assert len(data_rows) == 4
    header = "\n".join(line for line in lines if not line.startswith("|"))
    assert "N = 2" in header
    assert "abc1234" in header
    assert "Darwin arm64" in header


@pytest.mark.planning
def test_run_seed_child_emits_rows():
    proc = subprocess.run(
        [sys.executable, str(SCRIPTS / "ompl_seed_report.py"), "--seed", "1", "--json"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 0, proc.stderr

    rows = json.loads(proc.stdout.strip().splitlines()[-1])

    assert len(rows) == 2
    assert {row["case"] for row in rows} == {"default", "threshold"}
    for row in rows:
        assert row["trajopt_iterations"] > 0
        assert row["final_min_clearance_m"] > 0
        assert row["ompl_min_clearance_m"] > 0
        assert row["ompl_wall_time_s"] > 0
