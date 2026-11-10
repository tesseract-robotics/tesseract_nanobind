"""Inspect seeded planning on the existing Windows wheel without rebuilding it."""

import json
import runpy
import subprocess
import sys
from pathlib import Path

root = Path(__file__).resolve().parents[1]
fixture = runpy.run_path(str(root / "tests/scripts/test_ompl_seed_report.py"))
code = fixture["_REMAP_CHILD"].format(
    scripts=str(root / "scripts"), seed=1, planning_time=30.0, planners=1
)
code = code.replace(
    "final = [",
    """
from tesseract_robotics.tesseract_motion_planners_ompl import RNG_getSeed
print("SEED", RNG_getSeed())
for node in case["result"].task_infos:
    print("NODE", node.name, node.elapsed_time)
if variant:
    import numpy as np
    positions = report.program_positions(report.ompl_program(case["result"]))
    print("OMPL_LENGTH", sum(float(np.linalg.norm(b-a)) for a,b in zip(positions, positions[1:])))
final = [""",
)
output = root / "planning-diagnostics"
output.mkdir(exist_ok=True)
results = []
for index, arm in enumerate(("stock", "stock", "variant", "variant")):
    process = subprocess.run([sys.executable, "-c", code, arm], text=True, capture_output=True)
    (output / f"{index}-{arm}.stdout").write_text(process.stdout)
    (output / f"{index}-{arm}.stderr").write_text(process.stderr)
    print(index, arm, "exit", process.returncode, flush=True)
    print("\n".join(process.stdout.splitlines()[:-1]), flush=True)
    if process.returncode:
        print(process.stderr, flush=True)
        process.check_returncode()
    result = json.loads(process.stdout.splitlines()[-1])
    results.append(result)
    print("final states", len(result["final"]), flush=True)
print("stock repeats equal:", results[0]["final"] == results[1]["final"])
print("variant repeats equal:", results[2]["final"] == results[3]["final"])
print("stock/variant equal:", results[0]["final"] == results[2]["final"])
