"""Data for the 'carry level' figure: tool-axis deviation per replan, with and without the rule.

pixi run python scripts/plot_python_constraint_effect.py out.json
"""

from __future__ import annotations

import contextlib
import importlib
import io
import json
import sys

import numpy as np

from tesseract_robotics.examples import online_planning_scene
from tesseract_robotics.trajopt_ifopt.python_constraints import HoldAxis

ex = importlib.import_module("tesseract_robotics.examples.online_planning_sqp_example")
# One warm-started SQP step per tick, the unit this figure compares (the example takes up to 10).
ONE_SQP_STEP = 1
TARGET = np.array(online_planning_scene.TARGET_JOINTS)  # the example's target


def carry_level(vars_, manip, env):
    axis = manip.calcFwdKin(TARGET)["tool0"].rotation[:, 2]
    return [
        HoldAxis(v, manip, "tool0", axis, np.radians(0.5), coeff=50.0, name=f"level_{i}")
        for i, v in enumerate(vars_)
        if i > 0
    ]


def series(extra, replans=30):
    with contextlib.redirect_stdout(io.StringIO()):
        r = ex.run(extra_constraints=extra, num_replan=replans, sqp_steps_per_tick=ONE_SQP_STEP)
    manip = r["robot"].env.getKinematicGroup("manipulator")
    axis = manip.calcFwdKin(TARGET)["tool0"].rotation[:, 2]
    n, steps = len(r["joint_names"]), r["steps"]
    out = []
    for x in r["trajectories"]:
        traj = x[: steps * n].reshape(steps, n)
        cos = [manip.calcFwdKin(q)["tool0"].rotation[:, 2] @ axis for q in traj[1:]]
        out.append(float(np.degrees(np.arccos(np.clip(min(cos), -1, 1)))))
    return out


if __name__ == "__main__":
    data = {"without": series(None), "with": series(carry_level)}
    with open(sys.argv[1], "w") as fh:
        json.dump(data, fh)
    print({k: max(v) for k, v in data.items()})
