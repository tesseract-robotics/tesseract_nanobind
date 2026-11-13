"""What does a Python-defined constraint cost in the online SQP loop, and does it hold?

Runs examples/online_planning_sqp_example.py (8-DOF gantry, 12 waypoints, warm-started single
SQP step per replan, a moving obstacle) with and without Python rules from
trajopt_ifopt.python_constraints on every free waypoint (11 Python constraint sets per
problem). Reports per configuration:

- replan: mean wall time of one replan (rebuild + one SQP step), as the example measures it
- python: time spent inside the Python callbacks (getValues / getJacobian), per solve
- max dev: after the last replan, the largest angle between the tool axis and its axis at the
  target, over the free waypoints. The "carry level" rule bounds it to --max-dev.

The last row holds the same axis with trajopt's own CartAxisAlignConstraint (an equality, in
C++), as the reference for what the Python rules cost.

    pixi run python scripts/bench_python_constraints.py [--replans 30] [--repeats 5] [--continuous]
"""

from __future__ import annotations

import argparse
import contextlib
import importlib
import io
import json
import platform
import statistics
import time

import numpy as np

from tesseract_robotics import trajopt_ifopt as ti
from tesseract_robotics.examples import online_planning_scene
from tesseract_robotics.trajopt_ifopt import python_constraints as pc

ex = importlib.import_module("tesseract_robotics.examples.online_planning_sqp_example")

# One warm-started SQP step per tick, the unit timed here (the example takes up to 10).
ONE_SQP_STEP = 1
TCP = "tool0"
TOOL_AXIS = np.array([0.0, 0.0, 1.0])  # the tool's z axis, in tool coordinates
TARGET = np.array(online_planning_scene.TARGET_JOINTS)

# time spent inside the Python callbacks, summed per run
_PY_SECONDS = [0.0]


def _timed(method):
    def wrapper(self):
        t0 = time.perf_counter()
        try:
            return method(self)
        finally:
            _PY_SECONDS[0] += time.perf_counter() - t0

    return wrapper


pc.PyConstraint.getValues = _timed(pc.PyConstraint.getValues)
pc.PyConstraint.getJacobian = _timed(pc.PyConstraint.getJacobian)


def target_axis(manip):
    return manip.calcFwdKin(TARGET)[TCP].rotation[:, 2]


def deviation_deg(manip, q):
    cos = manip.calcFwdKin(q)[TCP].rotation[:, 2] @ target_axis(manip)
    return float(np.degrees(np.arccos(np.clip(cos, -1, 1))))


def configs(max_dev, w_min, coeff):
    def carry_level(analytic):
        def extra(vars_, manip, env):
            axis = target_axis(manip)
            return [
                pc.HoldAxis(
                    v,
                    manip,
                    TCP,
                    axis,
                    np.radians(max_dev),
                    analytic=analytic,
                    coeff=coeff,
                    name=f"level_{i}",
                )
                for i, v in enumerate(vars_)
                if i > 0
            ]

        return extra

    def dexterous(vars_, manip, env):
        return [
            pc.StayDexterous(v, manip, TCP, w_min, coeff=coeff, name=f"dex_{i}")
            for i, v in enumerate(vars_)
            if i > 0
        ]

    def cosine_form(vars_, manip, env):
        axis = target_axis(manip)
        return [
            pc.PointsDown(
                v, manip, TCP, np.radians(max_dev), down=axis, coeff=coeff, name=f"cos_{i}"
            )
            for i, v in enumerate(vars_)
            if i > 0
        ]

    def cpp_align(vars_, manip, env):
        axis = target_axis(manip)
        return [
            ti.CartAxisAlignConstraint(
                v, manip, TCP, TOOL_AXIS, manip.getBaseLinkId(), axis, coeff, f"align_{i}"
            )
            for i, v in enumerate(vars_)
            if i > 0
        ]

    return {
        "baseline (C++ constraints only)": None,
        "+ carry level, analytic Jacobian": carry_level(True),
        "+ carry level, numerical Jacobian": carry_level(False),
        "+ carry level, cosine form (too flat)": cosine_form,
        "+ stay dexterous, numerical Jacobian": dexterous,
        "+ axis held in C++ (CartAxisAlign)": cpp_align,
    }


def run_once(extra, replans, continuous):
    _PY_SECONDS[0] = 0.0
    with contextlib.redirect_stdout(io.StringIO()):
        r = ex.run(
            use_continuous_collision=continuous,
            extra_constraints=extra,
            num_replan=replans,
            sqp_steps_per_tick=ONE_SQP_STEP,
        )
    manip = r["robot"].env.getKinematicGroup("manipulator")
    n, steps = len(r["joint_names"]), r["steps"]
    traj = r["trajectories"][-1][: steps * n].reshape(steps, n)
    return {
        "initial_ms": r["timings"][0] * 1e3,
        "replan_ms": float(np.mean(r["timings"][1:]) * 1e3),
        "python_ms_per_solve": _PY_SECONDS[0] * 1e3 / len(r["timings"]),
        "max_dev_deg": max(deviation_deg(manip, q) for q in traj[1:]),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--replans", type=int, default=30)
    ap.add_argument("--repeats", type=int, default=5)
    ap.add_argument("--max-dev", type=float, default=0.5, help="carry-level bound, degrees")
    ap.add_argument("--w-min", type=float, default=0.05, help="manipulability bound")
    ap.add_argument("--coeff", type=float, default=50.0, help="merit coefficient of the rules")
    ap.add_argument("--continuous", action="store_true")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    print(
        f"{platform.platform()} · {platform.processor() or platform.machine()} · "
        f"{'LVS continuous' if args.continuous else 'discrete'} collision · "
        f"{args.replans} replans × {args.repeats} runs (median)\n"
    )
    rules = configs(args.max_dev, args.w_min, args.coeff)
    # round-robin over the configurations, so a slow spell of the machine hits them all alike
    runs = {label: [] for label in rules}
    for _ in range(args.repeats):
        for label, extra in rules.items():
            runs[label].append(run_once(extra, args.replans, args.continuous))
    rows = {}
    for label, results in runs.items():
        row = {k: statistics.median(r[k] for r in results) for k in results[0]}
        rows[label] = row
        print(
            f"{label:38s} replan {row['replan_ms']:6.1f} ms ({1e3 / row['replan_ms']:4.0f} Hz)"
            f"  python {row['python_ms_per_solve']:4.1f} ms"
            f"  initial {row['initial_ms']:6.1f} ms  max dev {row['max_dev_deg']:5.2f}°"
        )
    if args.json:
        with open(args.json, "w") as fh:
            json.dump(
                {"args": vars(args), "platform": platform.platform(), "rows": rows}, fh, indent=2
            )


if __name__ == "__main__":
    main()
