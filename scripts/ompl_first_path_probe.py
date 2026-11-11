"""OMPL path of the clearance example's threshold case against its first path (#362).

Plans case 2 of `freespace_clearance_objective_example.py` twice per seed: with the cost
threshold, and with `optimize = False`, which returns the first path found. A first path that
meets the threshold should come back unchanged, since the threshold ends the optimize loop at
the first solution. After a longer first path the planner re-solves and keeps its best solution,
which should meet the threshold, so it found a shorter path. Each run has its own process:
OMPL's RNG cannot be re-seeded once planning started (`RNG_setSeed`).

Usage: `pixi run ompl-first-path-probe` prints one JSON row per seed and mode. `check_pair` raises
when a seed's rows break what it checks; its docstring says what that is and what it is not.
"""

from __future__ import annotations

import argparse
import importlib
import json
import subprocess
import sys
from collections.abc import Sequence
from pathlib import Path
from typing import Any

import ompl_seed_report as report

from tesseract_robotics.planning import TaskComposer
from tesseract_robotics.tesseract_motion_planners_ompl import RNG_setSeed

example = importlib.import_module(
    "tesseract_robotics.examples.freespace_clearance_objective_example"
)

PROBE_SEEDS = (1, 2, 3, 4, 5)
THRESHOLD_MODE = "threshold"
FIRST_PATH_MODE = "first"
# One planner reproduces on a seed (5 seeds, 3 runs, identical). With the example's two
# parallel planners 1 of 42 runs differed: seed 3 gave 3.64 rad against its usual 3.57. Their
# race is the inferred cause; 36 repeats did not reproduce it.
PROBE_NUM_PLANNERS = 1
# One planner on one seed with no second path scored: the same path is bit-identical.
COMPARED_FIELDS = ("ompl_len", "ompl_states")
# s: one case's OMPL stage, capped at `planning_time`, plus the rest of the child.
PROBE_CHILD_TIMEOUT_S = example.PLANNING_TIME_S + report.CHILD_OVERHEAD_S


class FirstPathMismatchError(RuntimeError):
    """The first path met the threshold, yet the threshold case returned another path."""


class ThresholdNotMetError(RuntimeError):
    """The threshold case returned an OMPL path longer than the threshold."""


class MarginMissingError(RuntimeError):
    """A row has no OMPL collision margin, so the plan's margin is unknown."""


class MarginMismatchError(RuntimeError):
    """A row's OMPL collision margin is not the one the example plans at."""


class ProbeRunError(RuntimeError):
    """A child process running one seed and mode failed."""


class ProbeHungError(ProbeRunError):
    """A child process outlived its bound; `stdout` and `stderr` hold what it printed."""

    def __init__(self, seed: int, mode: str, timeout_s: float, stdout: str, stderr: str) -> None:
        super().__init__(
            f"seed {seed}, mode {mode} still running after {timeout_s} s, killed.\n"
            f"--- stdout ---\n{stdout}\n--- stderr ---\n{stderr}"
        )
        self.stdout = stdout
        self.stderr = stderr


def _check_margin(row: dict[str, Any]) -> None:
    margin = row.get("margin")
    if margin is None:
        raise MarginMissingError(f"{row} has no margin")
    # Exact: `probe` reads back the double the example stored.
    if margin != example.OMPL_CONTACT_MARGIN_M:
        raise MarginMismatchError(f"{row} is not at {example.OMPL_CONTACT_MARGIN_M} m")


def check_pair(threshold: dict[str, Any], first: dict[str, Any]) -> None:
    """Check one seed's two rows, from the rows alone.

    Checks that both rows carry the example's OMPL margin, that the threshold path is no longer
    than the threshold, and that when the first path meets the threshold both rows have the same
    length and state count.

    Does not show why a path came back. `ompl_s` is ignored, so nothing here says when the
    optimize loop ended. The rows record no optimize flag, so this cannot tell whether the
    `optimize = False` override took effect: without it both modes would plan alike and pass.
    The margin is the profile's value read back after planning, not what the planner applied.

    Args:
        threshold: The row of `THRESHOLD_MODE`.
        first: The row of `FIRST_PATH_MODE`.

    Raises:
        MarginMissingError: A row has no margin.
        MarginMismatchError: A row's margin is not `OMPL_CONTACT_MARGIN_M`.
        ThresholdNotMetError: The threshold case's path is longer than the threshold.
        FirstPathMismatchError: The first path met the threshold and the rows differ.
    """
    _check_margin(threshold)
    _check_margin(first)
    limit = example.PATH_LENGTH_THRESHOLD_RAD
    if threshold["ompl_len"] > limit:
        raise ThresholdNotMetError(f"{threshold} is longer than {limit} rad")
    same_path = all(threshold[field] == first[field] for field in COMPARED_FIELDS)
    if first["ompl_len"] <= limit and not same_path:
        raise FirstPathMismatchError(f"{threshold} != {first}")


def probe(seed: int, mode: str) -> dict[str, Any]:
    """Plan case 2 once in this process and describe its OMPL path.

    Args:
        seed: OMPL RNG seed.
        mode: `THRESHOLD_MODE`, or `FIRST_PATH_MODE` to turn the optimize loop off.

    Returns:
        JSON-ready row: seed, mode, OMPL time (s), length (rad), state count, margin (m).
    """
    composer = TaskComposer.from_config(report.VARIANT_CONFIG)
    composer.warmup([report.VARIANT_PIPELINE])
    RNG_setSeed(seed)
    profile = example.LengthAndClearanceProfile(example.PATH_LENGTH_THRESHOLD_RAD)
    if mode == FIRST_PATH_MODE:
        build_profiles = example._profiles

        def first_path_profiles(*args: Any, **kwargs: Any) -> Any:
            profiles = build_profiles(*args, **kwargs)
            profile.solver_config.optimize = False
            return profiles

        example._profiles = first_path_profiles
    result = example.run_case(
        mode,
        profile,
        example.PLANNING_TIME_S,
        num_planners=PROBE_NUM_PLANNERS,
        composer=composer,
        pipeline=report.VARIANT_PIPELINE,
    )["result"]
    positions = report.program_positions(report.ompl_program(result))
    return {
        "seed": seed,
        "mode": mode,
        "ompl_s": example.node_info(result, example.OMPL_NODE).elapsed_time,
        "ompl_len": report.joint_length_rad(positions),
        "ompl_states": len(positions),
        "margin": profile.contact_manager_config.default_margin,
    }


def probe_in_subprocess(
    seed: int, mode: str, *, timeout_s: float = PROBE_CHILD_TIMEOUT_S
) -> dict[str, Any]:
    """Run `probe` in a fresh process and return its row.

    Args:
        seed: OMPL RNG seed.
        mode: `THRESHOLD_MODE` or `FIRST_PATH_MODE`.
        timeout_s: How long the child may run (s).

    Raises:
        ProbeHungError: The child outlived `timeout_s`.
        ProbeRunError: The child exited non-zero; the message carries its stderr.
    """
    try:
        completed = subprocess.run(
            # -u: what the child printed survives the kill.
            [sys.executable, "-u", str(Path(__file__)), "--seed", str(seed), "--mode", mode],
            capture_output=True,
            encoding="utf-8",
            errors="replace",
            check=False,
            timeout=timeout_s,
        )
    except subprocess.TimeoutExpired as expired:
        raise ProbeHungError(
            seed,
            mode,
            timeout_s,
            report.captured_text(expired.stdout),
            report.captured_text(expired.stderr),
        ) from expired
    if completed.returncode != 0:
        raise ProbeRunError(
            f"seed {seed}, mode {mode} exited {completed.returncode}:\n{completed.stderr}"
        )
    return json.loads(completed.stdout.strip().splitlines()[-1])


def main(argv: Sequence[str] | None = None) -> None:
    """Probe every seed in both modes, or one seed and mode with `--seed` and `--mode`."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--mode", choices=(THRESHOLD_MODE, FIRST_PATH_MODE))
    args = parser.parse_args(argv)
    if (args.seed is None) != (args.mode is None):
        parser.error("--seed and --mode must be given together")
    if args.seed is not None:
        print(json.dumps(probe(args.seed, args.mode)))
        return
    for seed in PROBE_SEEDS:
        threshold = probe_in_subprocess(seed, THRESHOLD_MODE)
        first = probe_in_subprocess(seed, FIRST_PATH_MODE)
        print(json.dumps(threshold))
        print(json.dumps(first))
        check_pair(threshold, first)


if __name__ == "__main__":
    main()
