# Planning benchmarks

Run serially with `pixi run test-benchmark` (pytest-benchmark disables itself under xdist).
Every benchmark validates the result it times with `assert_trajectory_valid`; a fast but
unusable trajectory fails.

| File | What it times |
|---|---|
| `test_planning_benchmarks.py` | Each planning example's `run()`, and the freespace examples at 1, 2, 4 and 8 OMPL planners |
| `test_parallel_scaling.py` | OMPL planner count against solve time on a sphere-obstacle freespace problem, with `optimize=True` and `optimize=False` |
| `test_pose_accessors.py` | `Pose` scalar accessors; asserts the ratios the `transforms.py` perf note claims |
| `freespace_problems.py` | The scaling problem. Its correctness tests run in the normal suite (`tests/test_freespace_benchmark_problems.py`) |

```bash
pixi run test-benchmark
pixi run pytest tests/benchmarks --benchmark-only --benchmark-json=results.json
pixi run python tests/benchmarks/plot_benchmarks.py results.json
```

## Why more planners do not make this problem faster

The measured table is at the top of `test_parallel_scaling.py`, with its hardware and method.
Two facts explain it:

- With `optimize=True` OMPL keeps improving the path until `planning_time` is spent, so extra
  planners add threads, not speed.
- With `optimize=False` the first solution ends the call, but the sphere problem is solved by
  one planner in about 100 ms; thread start-up then
  outweighs the extra samplers.

Parallel planners pay off on problems where finding any solution is slow: narrow passages,
many obstacles, tight margins. Hold one `TaskComposer` for every timed call: building one
loads the plugins again (about 4 s) and swamps the planning time.
