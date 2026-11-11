"""
Parallel scaling benchmarks: OMPL planner count vs solve time on one sphere obstacle.

Measured 2026-10-15 on an Apple M1 Max, one held composer (no plugin load in the
timer, #137), FreespacePipeline, 5 rounds, mean:

| planners | optimize=False (planning_time 30 s) | optimize=True (planning_time 2 s) |
|---|---|---|
| 1 | 85 ms | 128 ms |
| 2 | 85 ms | 106 ms |
| 4 | 102 ms | 106 ms |
| 8 | 133 ms | 150 ms |

More planners do not make this problem faster: it is solved within ~100 ms by
one planner, so extra threads only add overhead (8 planners: 1.6x slower in
first-solution mode). Neither mode runs to its planning_time here; why
optimize=True returns well before 2 s is not established.

The table this replaces (1 planner 6.05 s vs 4 planners 3.83 s, "1.58x faster")
timed a fresh TaskComposer per call, i.e. ~4.3 s of plugin load per round.

Run with: pytest tests/benchmarks/test_parallel_scaling.py -v --benchmark-enable
Or use:   ./scripts/run_scaling_benchmarks.sh
"""

import pytest

from tesseract_robotics.planning import TaskComposer

from ..planning_validity import assert_trajectory_valid
from .freespace_problems import GOAL, freespace_problem, plan_freespace

# OMPL's budget with optimize=True: it runs to this limit, so it bounds the timed call (s).
OPTIMIZE_PLANNING_TIME_S = 2.0
# OMPL's budget with optimize=False: a ceiling only, the first solution ends the call (s).
FIRST_SOLUTION_TIMEOUT_S = 30.0


@pytest.fixture(scope="module")
def composer():
    """One composer for the module, so the plugin load (#137) stays out of the timed call."""
    return TaskComposer.from_config()


class TestOptimizeModeScaling:
    """Planner count vs solve time with optimize=True (planning_time 2 s)."""

    @pytest.mark.parametrize("num_planners", [1, 2, 4, 8])
    def test_optimize_true(self, benchmark, composer, num_planners):
        """optimize=True with `num_planners` parallel planners."""
        robot, program = freespace_problem(1)
        result = benchmark(
            plan_freespace,
            composer,
            robot,
            program,
            num_planners=num_planners,
            optimize=True,
            planning_time=OPTIMIZE_PLANNING_TIME_S,
        )
        assert_trajectory_valid(robot, result, GOAL)
        benchmark.extra_info["mode"] = "optimize=True"
        benchmark.extra_info["num_planners"] = num_planners


class TestFirstSolutionScaling:
    """Planner count vs solve time with optimize=False (first solution wins)."""

    @pytest.mark.parametrize("num_planners", [1, 2, 4, 8])
    def test_optimize_false(self, benchmark, composer, num_planners):
        """optimize=False with `num_planners` parallel planners."""
        robot, program = freespace_problem(1)
        result = benchmark(
            plan_freespace,
            composer,
            robot,
            program,
            num_planners=num_planners,
            optimize=False,
            planning_time=FIRST_SOLUTION_TIMEOUT_S,
        )
        assert_trajectory_valid(robot, result, GOAL)
        benchmark.extra_info["mode"] = "optimize=False"
        benchmark.extra_info["num_planners"] = num_planners


class TestHarderProblemScaling:
    """Benchmark with harder problems where parallelism helps more.

    Easy problems (1 obstacle) find solutions in < 100ms - overhead dominates.
    Harder problems (3+ obstacles) need more exploration - parallelism helps.
    """

    @pytest.mark.parametrize("num_planners", [1, 2, 4, 8])
    @pytest.mark.parametrize("num_obstacles", [1, 3, 5])
    def test_difficulty_scaling(self, benchmark, composer, num_planners, num_obstacles):
        """Test how problem difficulty affects parallel scaling."""
        robot, program = freespace_problem(num_obstacles)
        result = benchmark(
            plan_freespace,
            composer,
            robot,
            program,
            num_planners=num_planners,
            optimize=False,
            planning_time=FIRST_SOLUTION_TIMEOUT_S,
        )
        assert_trajectory_valid(robot, result, GOAL)
        benchmark.extra_info["mode"] = "optimize=False"
        benchmark.extra_info["num_planners"] = num_planners
        benchmark.extra_info["num_obstacles"] = num_obstacles


class TestScalingSummary:
    """Direct comparison of scaling behavior.

    Run these tests together to see the difference:
    pytest -k TestScalingSummary --benchmark-group-by=param:mode
    """

    @pytest.mark.parametrize("num_planners", [1, 4])
    @pytest.mark.parametrize("mode", ["optimize", "first_solution"])
    def test_comparison(self, benchmark, composer, num_planners, mode):
        """Side-by-side comparison of modes."""
        optimize = mode == "optimize"
        robot, program = freespace_problem(3)  # medium difficulty
        result = benchmark(
            plan_freespace,
            composer,
            robot,
            program,
            num_planners=num_planners,
            optimize=optimize,
            planning_time=OPTIMIZE_PLANNING_TIME_S if optimize else FIRST_SOLUTION_TIMEOUT_S,
        )
        assert_trajectory_valid(robot, result, GOAL)
        benchmark.extra_info["mode"] = mode
        benchmark.extra_info["num_planners"] = num_planners
