# Planning Benchmarks

## Why More CPUs Don't Make Planning Faster

Investigation into why benchmark tests didn't run faster with more CPUs.

### Root Cause

**OMPL's `optimize=True` mode (the default) runs for the full `planning_time` regardless of how many parallel planners are running.**

With `optimize=True`:
- OMPL loops until the timeout expires
- Finding a solution early doesn't exit the loop
- More planners = more thread overhead = actually SLOWER

With `optimize=False`:
- OMPL exits as soon as ANY planner finds a solution
- More planners = higher probability of finding solution faster
- Shows actual parallel speedup

### Benchmark Results

```
optimize=False (first-solution mode) - SHOWS SCALING:
  1 planner:  6.05s (baseline)
  2 planners: 4.03s (1.50x faster)
  4 planners: 3.83s (1.58x faster)
  8 planners: 3.85s (1.57x faster)

optimize=True (optimization mode) - NO SCALING / GETS WORSE:
  1 planner:  3.92s (baseline)
  2 planners: 3.84s (~same)
  4 planners: 4.51s (slower, high variance)
  8 planners: 7.26s (1.85x SLOWER)
```

### Why Speedup Caps at ~1.6x

Even with `optimize=False`, speedup plateaus because:

1. **TrajOpt is single-threaded**: The FreespacePipeline runs `OMPL → TrajOpt → ContactCheck → TimeParam`. TrajOpt alone takes ~2s and can't be parallelized.

2. **Problem is too easy**: For simple problems (single sphere obstacle), a single planner finds a solution in <1s. The overhead of spawning threads dominates.

3. **Collision checking contention**: All parallel planners share the collision manager. More planners = more lock contention.

### Code Change

Added `optimize` and `max_solutions` parameters to `create_freespace_pipeline_profiles()`:

```python
# Original behavior (no scaling)
profiles = create_freespace_pipeline_profiles(num_planners=8)  # optimize=True

# First-solution mode (shows scaling)
profiles = create_freespace_pipeline_profiles(
    num_planners=8,
    optimize=False,
    max_solutions=1
)
```

### When Parallelism Actually Helps

1. **Difficult problems**: Narrow passages, many obstacles, tight collision margins where finding ANY solution takes significant time

2. **First-solution mode** (`optimize=False`): When you just need a valid path quickly

3. **Multiple independent planning problems**: Plan multiple robot motions concurrently instead of throwing CPUs at one easy problem

### Running Benchmarks

```bash
# Run scaling comparison
./scripts/run_scaling_benchmarks.sh

# Run specific test
pytest tests/benchmarks/test_parallel_scaling.py -v --benchmark-enable

# Original benchmarks (for comparison)
./scripts/run_benchmarks.sh
```

### Files

- `test_planning_benchmarks.py` - Original benchmarks (show no scaling)
- `test_parallel_scaling.py` - New benchmarks demonstrating the issue
- `plot_benchmarks.py` - Visualization utilities
- `../../scripts/run_scaling_benchmarks.sh` - Run script with summary
