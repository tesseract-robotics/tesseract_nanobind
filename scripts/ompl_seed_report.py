"""Per-seed report of the clearance example's OMPL and TrajOptIfopt stages (#362).

Runs `freespace_clearance_objective_example.py`'s two cases (default objective, cost
threshold) once per seed, each seed in its own process: OMPL's RNG cannot be re-seeded once
planning started (`RNG_setSeed`). It plans with `FreespaceIfoptOMPLOutputPipeline`
(`ompl_seed_report_task_composer.yaml`), stock `FreespaceIfoptPipeline` with the OMPL program
kept under its own key, so one run yields both the OMPL path and TrajOptIfopt's final path.

Usage: `pixi run ompl-seed-report` (all `REPORT_SEEDS`, JSON and Markdown into `--out-dir`),
or `python scripts/ompl_seed_report.py --seed 1 --json` for one seed's rows.
"""

from __future__ import annotations

import argparse
import importlib
import json
import platform
import subprocess
import sys
from collections.abc import Sequence
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from tesseract_robotics.planning import TaskComposer
from tesseract_robotics.planning.composer import PlanningResult, TrajectoryPoint
from tesseract_robotics.tesseract_command_language import CompositeInstruction
from tesseract_robotics.tesseract_motion_planners_ompl import RNG_setSeed
from tesseract_robotics.tesseract_task_composer import (
    AnyPoly_as_CompositeInstruction,
    AnyPoly_as_TaskComposerDataStorage,
)
from tesseract_robotics.trajopt_sqp import SQPCallback

example = importlib.import_module(
    "tesseract_robotics.examples.freespace_clearance_objective_example"
)

# N = 10 seeds: a seed plans both cases in about 2 x 14 s, so a report takes about 5 min.
REPORT_SEEDS = range(1, 11)
VARIANT_CONFIG = Path(__file__).with_name("ompl_seed_report_task_composer.yaml")
VARIANT_PIPELINE = "FreespaceIfoptOMPLOutputPipeline"
# The key the variant's OMPLMotionPlannerTask writes and its TrajOptIfopt task reads.
OMPL_OUTPUT_KEY = "ompl_output_data"
# The variant's nested pipeline holding the OMPL node; the run stores its data storage under
# the node's uuid (task_composer_pipeline.cpp).
MOTION_PLANNING_NODE = "MotionPlanningTask"
TRAJOPT_NODE = "TrajOptIfoptMotionPlannerTask"
DEFAULT_OUT_DIR = Path("build/ompl_seed_report")
# s: a child's time outside OMPL's capped solves (imports, plugin loading, TrajOptIfopt, clearance
# scans): 3 x the worst whole-child pass seen (32.6, a first cold child on a loaded machine), 97.8
# rounded up.
CHILD_OVERHEAD_S = 100.0
# s: both cases' OMPL stages, each capped at `planning_time`, plus the rest of the child.
SEED_CHILD_TIMEOUT_S = 2 * example.PLANNING_TIME_S + CHILD_OVERHEAD_S
GIT_COMMAND = ("git",)
# s: one git call. 100 x the slowest seen (0.27, `git status` with warm caches, load 20 on 10
# cores): cold caches are unmeasured, and this bound only costs the wait, 27 rounded up.
GIT_TIMEOUT_S = 30.0

COLUMNS = (
    ("seed", "seed"),
    ("case", "case"),
    ("ompl_wall_time_s", "OMPL (s)"),
    ("ompl_min_clearance_m", "OMPL min clearance (mm)"),
    ("trajopt_iterations", "TrajOptIfopt iterations"),
    ("trajopt_status", "TrajOptIfopt status"),
    ("final_min_clearance_m", "final min clearance (mm)"),
    ("final_length_rad", "final length (rad)"),
    ("total_wall_time_s", "total (s)"),
)
MM_PER_M = 1000.0


class OMPLProgramMissingError(LookupError):
    """No nested data storage of the run holds the OMPL program under `OMPL_OUTPUT_KEY`."""


class UnplannedWaypointError(ValueError):
    """A program the report reads holds a waypoint without joint positions."""


class SeedRunError(RuntimeError):
    """A seed's child process failed."""


class SeedHungError(SeedRunError):
    """A seed's child process outlived its bound; `stdout` and `stderr` hold what it printed."""

    def __init__(self, seed: int, timeout_s: float, stdout: str, stderr: str) -> None:
        super().__init__(
            f"seed {seed} still running after {timeout_s} s, killed.\n"
            f"--- stdout ---\n{stdout}\n--- stderr ---\n{stderr}"
        )
        self.stdout = stdout
        self.stderr = stderr


class GitHungError(RuntimeError):
    """A `git` child outlived its bound; `stdout` and `stderr` hold what it printed."""

    def __init__(self, command: Sequence[str], timeout_s: float, stdout: str, stderr: str) -> None:
        super().__init__(
            f"{' '.join(command)} still running after {timeout_s} s, killed.\n"
            f"--- stdout ---\n{stdout}\n--- stderr ---\n{stderr}"
        )
        self.command = tuple(command)
        self.stdout = stdout
        self.stderr = stderr


class TrajOptIterationCounter(SQPCallback):
    """Records the highest SQP iteration TrajOptIfopt reached.

    The solver may call a callback more than once per iteration, so this keeps the largest
    `overall_iteration` seen rather than counting calls.
    """

    def __init__(self) -> None:
        super().__init__()
        self.iterations = 0

    def execute(self, problem, sqp_results) -> bool:
        self.iterations = max(self.iterations, sqp_results.overall_iteration)
        return True


@dataclass(frozen=True)
class SeedRow:
    """One case of one seed.

    Attributes:
        seed: OMPL RNG seed of the run.
        case: `default` or `threshold`.
        ompl_wall_time_s: The OMPL node's elapsed time (s).
        ompl_min_clearance_m: Smallest cube clearance along the OMPL path (m).
        trajopt_iterations: SQP iterations TrajOptIfopt ran.
        trajopt_status: The TrajOptIfopt node's status message.
        final_min_clearance_m: Smallest cube clearance along the final path (m).
        final_length_rad: Joint-space length of the final path (rad).
        total_wall_time_s: Wall time of the whole pipeline (s).
    """

    seed: int
    case: str
    ompl_wall_time_s: float
    ompl_min_clearance_m: float
    trajopt_iterations: int
    trajopt_status: str
    final_min_clearance_m: float
    final_length_rad: float
    total_wall_time_s: float


def ompl_program(result: PlanningResult) -> CompositeInstruction:
    """The OMPL node's output program, from `MOTION_PLANNING_NODE`'s data storage.

    Raises:
        OMPLProgramMissingError: that storage has no `OMPL_OUTPUT_KEY` (a stock pipeline).
        example.NodeInfoMissingError: the run has no single `MOTION_PLANNING_NODE`.
    """
    uuid = str(example.node_info(result, MOTION_PLANNING_NODE).uuid)
    if result.data_storage is None or not result.data_storage.hasKey(uuid):
        raise OMPLProgramMissingError(f"the run stored no data for {MOTION_PLANNING_NODE}")
    nested = AnyPoly_as_TaskComposerDataStorage(result.data_storage.getData(uuid))
    if not nested.hasKey(OMPL_OUTPUT_KEY):
        raise OMPLProgramMissingError(
            f"{MOTION_PLANNING_NODE}'s storage has no {OMPL_OUTPUT_KEY!r}; plan with {VARIANT_PIPELINE}"
        )
    return AnyPoly_as_CompositeInstruction(nested.getData(OMPL_OUTPUT_KEY))


def program_positions(program: CompositeInstruction) -> list[np.ndarray]:
    """Joint positions of every move, descending into child composites.

    OMPL formats its result as its input (`format_result_as_input: true`): the start is a
    state waypoint and the planned states are joint waypoints.

    Raises:
        UnplannedWaypointError: a move holds a Cartesian waypoint.
    """
    positions = []
    for instruction in program.flatten():
        if not instruction.isMoveInstruction():
            continue
        waypoint = instruction.asMoveInstruction().getWaypoint()
        if waypoint.isStateWaypoint():
            positions.append(np.array(waypoint.asStateWaypoint().getPosition()))
        elif waypoint.isJointWaypoint():
            positions.append(np.array(waypoint.asJointWaypoint().getPosition()))
        else:
            raise UnplannedWaypointError("a move of the program holds a Cartesian waypoint")
    return positions


def min_clearance_m(robot, points: Sequence[TrajectoryPoint]) -> float:
    """Smallest cube clearance along `points` (m), via the example's clearance scan."""
    path = PlanningResult(successful=True, trajectory=list(points))
    return min(c.distance_m for c in example.closest_approaches(robot, path))


def joint_length_rad(positions: Sequence[np.ndarray]) -> float:
    """Sum of the L2 norms of the joint deltas between consecutive states (rad)."""
    stacked = np.asarray(positions)
    return float(np.sum(np.linalg.norm(np.diff(stacked, axis=0), axis=1)))


def run_seed(seed: int) -> list[SeedRow]:
    """Plan both cases on `seed` with the variant pipeline (child process)."""
    composer = TaskComposer.from_config(VARIANT_CONFIG)
    composer.warmup([VARIANT_PIPELINE])
    RNG_setSeed(seed)
    rows = []
    cases = (
        ("default", None),
        ("threshold", example.LengthAndClearanceProfile(example.PATH_LENGTH_THRESHOLD_RAD)),
    )
    for name, ompl_profile in cases:
        counter = TrajOptIterationCounter()
        case = example.run_case(
            name,
            ompl_profile,
            example.PLANNING_TIME_S,
            composer=composer,
            sqp_callback=counter,
            pipeline=VARIANT_PIPELINE,
        )
        robot, result = case["robot"], case["result"]
        ompl_points = [
            TrajectoryPoint(joint_names=[], positions=p)
            for p in program_positions(ompl_program(result))
        ]
        rows.append(
            SeedRow(
                seed=seed,
                case=name,
                ompl_wall_time_s=example.node_info(result, example.OMPL_NODE).elapsed_time,
                ompl_min_clearance_m=min_clearance_m(robot, ompl_points),
                trajopt_iterations=counter.iterations,
                trajopt_status=example.node_info(result, TRAJOPT_NODE).status_message,
                final_min_clearance_m=min_clearance_m(robot, result.trajectory),
                final_length_rad=joint_length_rad([p.positions for p in result.trajectory]),
                total_wall_time_s=case["wall_time_s"],
            )
        )
    return rows


def captured_text(stream: bytes | str | None) -> str:
    """A stream of `subprocess.TimeoutExpired` as text.

    It holds bytes even with `text=True` where the parent reads the pipes itself, and a str on
    Windows, where `subprocess.run` collects them after the kill.
    """
    if stream is None:
        return ""
    return stream.decode("utf-8", "replace") if isinstance(stream, bytes) else stream


def report(seeds: Sequence[int], *, timeout_s: float = SEED_CHILD_TIMEOUT_S) -> list[SeedRow]:
    """Run `run_seed` for each seed in its own process.

    Args:
        seeds: The seeds to run.
        timeout_s: How long one seed's child may run (s).

    Raises:
        SeedHungError: A child outlived `timeout_s`.
        SeedRunError: A child exited non-zero; the message carries its stderr.
    """
    rows: list[SeedRow] = []
    for seed in seeds:
        try:
            proc = subprocess.run(
                # -u: what the child printed survives the kill.
                [sys.executable, "-u", __file__, "--seed", str(seed), "--json"],
                capture_output=True,
                encoding="utf-8",
                errors="replace",
                check=False,
                timeout=timeout_s,
            )
        except subprocess.TimeoutExpired as expired:
            raise SeedHungError(
                seed, timeout_s, captured_text(expired.stdout), captured_text(expired.stderr)
            ) from expired
        if proc.returncode != 0:
            raise SeedRunError(f"seed {seed} exited {proc.returncode}:\n{proc.stderr}")
        rows.extend(SeedRow(**row) for row in json.loads(proc.stdout.strip().splitlines()[-1]))
    return rows


def _cell(field: str, value) -> str:
    if field.endswith("_m"):
        return f"{value * MM_PER_M:.1f}"
    if isinstance(value, float):
        return f"{value:.2f}"
    return str(value)


def to_markdown(rows: Sequence[SeedRow], *, commit: str, platform: str) -> str:
    """A Markdown table of `rows`, headed by N (distinct seeds), commit and platform."""
    n_seeds = len({row.seed for row in rows})
    lines = [
        f"N = {n_seeds} seeds, commit `{commit}`, {platform}, pipeline `{VARIANT_PIPELINE}`.",
        "",
        "| " + " | ".join(title for _, title in COLUMNS) + " |",
        "|" + "---|" * len(COLUMNS),
    ]
    for row in rows:
        values = asdict(row)
        lines.append("| " + " | ".join(_cell(f, values[f]) for f, _ in COLUMNS) + " |")
    return "\n".join(lines) + "\n"


def _git_output(command: Sequence[str], timeout_s: float) -> str:
    """Stripped stdout of one bounded `git` call.

    Raises:
        GitHungError: The call outlived `timeout_s`.
        subprocess.CalledProcessError: The call exited non-zero.
    """
    try:
        return subprocess.run(
            command, capture_output=True, text=True, check=True, timeout=timeout_s
        ).stdout.strip()
    except subprocess.TimeoutExpired as expired:
        raise GitHungError(
            command, timeout_s, captured_text(expired.stdout), captured_text(expired.stderr)
        ) from expired


def _commit(*, git: Sequence[str] = GIT_COMMAND, timeout_s: float = GIT_TIMEOUT_S) -> str:
    """The short HEAD, with `+dirty` when a tracked file changed.

    Args:
        git: The command that runs git.
        timeout_s: How long each git call may run (s).

    Raises:
        GitHungError: A call outlived `timeout_s`.
    """
    head = _git_output([*git, "rev-parse", "--short", "HEAD"], timeout_s)
    dirty = _git_output([*git, "status", "--porcelain", "--untracked-files=no"], timeout_s)
    return f"{head}+dirty" if dirty else head


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seed", type=int, help="run one seed in this process")
    parser.add_argument("--json", action="store_true", help="with --seed: print the rows as JSON")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    args = parser.parse_args()
    if args.seed is not None:
        rows = run_seed(args.seed)
        if args.json:
            print(json.dumps([asdict(row) for row in rows]))
        else:
            print(to_markdown(rows, commit=_commit(), platform=_platform()))
        return 0

    rows = report(REPORT_SEEDS)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    (args.out_dir / "rows.json").write_text(json.dumps([asdict(row) for row in rows], indent=2))
    table = to_markdown(rows, commit=_commit(), platform=_platform())
    (args.out_dir / "table.md").write_text(table)
    print(table)
    return 0


def _platform() -> str:
    return f"{platform.system()} {platform.machine()}, Python {platform.python_version()}"


if __name__ == "__main__":
    sys.exit(main())
