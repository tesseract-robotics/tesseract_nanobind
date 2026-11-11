"""
Planning performance benchmarks using pytest-benchmark.

Each benchmark validates the result it times: a run that returns an unusable trajectory fails
instead of recording a fast time.

Run with: pytest tests/benchmarks/ --benchmark-only -v
Compare runs: pytest tests/benchmarks/ --benchmark-compare
Save JSON: pytest tests/benchmarks/ --benchmark-only --benchmark-json=results.json
"""

import os
from dataclasses import dataclass

import pytest

from tesseract_robotics.examples import (
    basic_cartesian_example,
    car_seat_example,
    chain_example,
    freespace_hybrid_example,
    freespace_ompl_example,
    glass_upright_example,
    online_planning_example,
    pick_and_place_example,
    raster_example,
)
from tesseract_robotics.planning import Robot, TaskComposer

from ..planning_validity import assert_trajectory_valid


@dataclass(frozen=True)
class Case:
    """A planning example and how to validate what it returns.

    Attributes:
        run: The example's `run` function.
        group: Kinematic group its trajectory moves.
        check_collision: False where the environment changes during the run, so no single
            final environment is the one the trajectory was planned in.
    """

    run: object
    group: str = "manipulator"
    check_collision: bool = True

    def __str__(self) -> str:
        return self.run.__module__.rsplit(".", 1)[-1]


def assert_outcome_valid(case: Case, outcome: dict) -> None:
    """Validate every trajectory an example's `run()` returned."""
    kwargs = {"group": case.group, "check_collision": case.check_collision}
    if "result" in outcome:
        assert_trajectory_valid(outcome["robot"], outcome["result"], **kwargs)
    elif "trajectories" in outcome:
        for result in outcome["trajectories"]:
            assert_trajectory_valid(outcome["robot"], result, **kwargs)
    else:
        # Two phases: the pick was planned before the part was attached to the tool.
        # The pick ends with the tool at the part by design: its contact is not a fault.
        pick_kwargs = {**kwargs, "check_collision": False}
        assert_trajectory_valid(
            Robot.build(outcome["pick_env"]), outcome["pick_result"], **pick_kwargs
        )
        assert_trajectory_valid(outcome["robot"], outcome["place_result"], **kwargs)


# Every planning example. `scene_graph_example` plans nothing; the 9-DOF puzzle piece moves a
# joint group without a kinematic group, which `assert_trajectory_valid` cannot check, so
# `test_examples.py` times and validates it instead.
EXAMPLES = [
    Case(freespace_ompl_example),
    Case(pick_and_place_example),
    Case(car_seat_example),
    Case(glass_upright_example),
    Case(basic_cartesian_example),
    Case(raster_example),
    # The human moves between plans, so the final environment is not the planned one.
    Case(online_planning_example, check_collision=False),
    Case(chain_example),
    Case(freespace_hybrid_example),
]


@pytest.fixture(scope="module")
def composer():
    """One composer for the module, so rounds time planning, not plugin loading."""
    return TaskComposer.from_config()


class TestExampleBenchmarks:
    """Benchmark planning examples with default settings."""

    @pytest.mark.parametrize("case", EXAMPLES, ids=str)
    def test_default(self, benchmark, case, composer):
        """Time each example's run() function with default pipeline."""
        outcome = benchmark(case.run, composer=composer)
        assert_outcome_valid(case, outcome)
        benchmark.extra_info["example"] = str(case)
        benchmark.extra_info["cpus"] = os.cpu_count()


# Examples whose problem the FreespacePipeline solves and whose planner count `run()` applies.
FREESPACE_EXAMPLES = [Case(freespace_ompl_example), Case(freespace_hybrid_example)]


class TestOMPLScaling:
    """Benchmark the freespace examples with OMPL parallel planners at different CPU counts.

    The other examples need TrajOpt's Cartesian or constraint handling, or ignore `num_planners`,
    so a planner-count sweep of them would time the same call four times.
    """

    @pytest.mark.parametrize("case", FREESPACE_EXAMPLES, ids=str)
    @pytest.mark.parametrize("num_planners", [1, 2, 4, 8])
    def test_scaling(self, benchmark, case, num_planners, composer):
        """Time examples with FreespacePipeline and varying planner counts."""
        outcome = benchmark(
            case.run, pipeline="FreespacePipeline", num_planners=num_planners, composer=composer
        )
        assert_outcome_valid(case, outcome)
        benchmark.extra_info["example"] = str(case)
        benchmark.extra_info["num_planners"] = num_planners
        benchmark.extra_info["cpus"] = os.cpu_count()
