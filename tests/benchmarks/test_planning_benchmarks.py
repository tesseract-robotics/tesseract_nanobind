"""
Planning performance benchmarks using pytest-benchmark.

Run with: pytest tests/benchmarks/ --benchmark-only -v
Compare runs: pytest tests/benchmarks/ --benchmark-compare
Save JSON: pytest tests/benchmarks/ --benchmark-only --benchmark-json=results.json
"""

import os
import pytest

# Skip if benchmark not installed
pytest.importorskip("pytest_benchmark")

from tesseract_robotics.examples import (
    freespace_ompl_example,
    pick_and_place_example,
    glass_upright_example,
    basic_cartesian_example,
    raster_example,
    online_planning_example,
    chain_example,
    freespace_hybrid_example,
    puzzle_piece_auxillary_axes_example,
    scene_graph_example,
)

# Examples that work reliably
EXAMPLES = [
    freespace_ompl_example,
    pick_and_place_example,
    glass_upright_example,
    basic_cartesian_example,
    raster_example,
    online_planning_example,
    chain_example,
    freespace_hybrid_example,
    puzzle_piece_auxillary_axes_example,
    scene_graph_example,
]

# car_seat excluded due to complex collision environment - linear path passes through 50% collisions


class TestExampleBenchmarks:
    """Benchmark planning examples with default settings."""

    @pytest.mark.parametrize("example", EXAMPLES)
    def test_default(self, benchmark, example):
        """Time each example's run() function with default pipeline."""
        result = benchmark(example)
        assert result, f"{example} failed"
        benchmark.extra_info["example"] = f'{example}'
        benchmark.extra_info["cpus"] = os.cpu_count()


class TestOMPLScaling:
    """Benchmark examples with OMPL parallel planners at different CPU counts.

    Tests all examples with FreespacePipeline and varying num_planners.
    Examples that require TrajOpt for constrained motions will be skipped.
    """

    @pytest.mark.parametrize("example", EXAMPLES)
    @pytest.mark.parametrize("num_planners", [1, 2, 4, 8])
    def test_scaling(self, benchmark, example, num_planners):
        """Time examples with FreespacePipeline and varying planner counts."""
        try:
            result = benchmark(example, pipeline="FreespacePipeline", num_planners=num_planners)
            assert result, f"{example} failed"
        except (AssertionError, RuntimeError) as e:
            pytest.skip(f"{example} requires TrajOpt: {e}")
        benchmark.extra_info["example"] = f'{example}'
        benchmark.extra_info["num_planners"] = num_planners
        benchmark.extra_info["cpus"] = os.cpu_count()
