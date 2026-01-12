#!/bin/bash
# Run parallel scaling benchmarks to demonstrate OMPL parallelization behavior
#
# Key finding: optimize=True prevents scaling because OMPL runs for full timeout
# regardless of planner count. Use optimize=False for first-solution mode to
# see actual parallel speedup.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR/.."
cd "$PROJECT_ROOT"

source "$SCRIPT_DIR/env.sh"

echo "╔══════════════════════════════════════════════════════════════════════════════╗"
echo "║                     OMPL PARALLEL SCALING BENCHMARKS                         ║"
echo "╠══════════════════════════════════════════════════════════════════════════════╣"
echo "║ Testing why more CPUs don't make planning faster (spoiler: optimize=True)   ║"
echo "╚══════════════════════════════════════════════════════════════════════════════╝"
echo ""

JSON_FILE=$(mktemp /tmp/scaling_bench_XXXXXX.json)

# Run both optimize modes
pytest tests/benchmarks/test_parallel_scaling.py \
    -k "TestOptimizeModeScaling or TestFirstSolutionScaling" \
    --benchmark-enable \
    --benchmark-min-rounds=2 \
    --benchmark-json="$JSON_FILE" \
    --benchmark-group-by=func \
    -v "$@"

# Generate summary
python3 - "$JSON_FILE" << 'PYTHON'
import json
import sys
from collections import defaultdict

results = defaultdict(dict)

with open(sys.argv[1]) as f:
    for bench in json.load(f).get("benchmarks", []):
        name = bench["name"]
        params = bench.get("params", {})

        # Parse test name to get mode and planners
        if "optimize_true" in name:
            mode = "optimize=True"
        elif "optimize_false" in name:
            mode = "optimize=False"
        else:
            continue

        planners = params.get("num_planners", 1)
        results[mode][planners] = bench["stats"]["mean"]

print()
print("╔══════════════════════════════════════════════════════════════════════════════╗")
print("║                           SCALING RESULTS                                    ║")
print("╠══════════════════════════════════════════════════════════════════════════════╣")
print()

for mode in ["optimize=True", "optimize=False"]:
    if mode not in results:
        continue
    data = results[mode]
    baseline = data.get(1, 0)

    desc = "runs full timeout, no scaling" if mode == "optimize=True" else "exits on first solution, SHOWS SCALING"
    print(f"  {mode} ({desc}):")
    print(f"  {'Planners':<10} │ {'Time':>8} │ {'vs 1 planner':>12}")
    print(f"  " + "─" * 10 + "─┼─" + "─" * 8 + "─┼─" + "─" * 12)

    for p in sorted(data.keys()):
        t = data[p]
        if baseline and p > 1:
            ratio = baseline / t
            speedup = f"{ratio:.2f}x {'faster' if ratio > 1 else 'SLOWER'}"
        else:
            speedup = "baseline"
        print(f"  {p:<10} │ {t:>7.2f}s │ {speedup:>12}")
    print()

print("╠══════════════════════════════════════════════════════════════════════════════╣")
print("║ CONCLUSION:                                                                  ║")
print("║   • optimize=True: OMPL runs for full planning_time regardless of planners  ║")
print("║   • optimize=False: OMPL exits on first solution - parallel planners help   ║")
print("║   • TrajOpt runs after OMPL and is single-threaded (caps max speedup)       ║")
print("╚══════════════════════════════════════════════════════════════════════════════╝")
PYTHON

rm "$JSON_FILE"
