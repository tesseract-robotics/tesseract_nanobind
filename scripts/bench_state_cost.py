"""Bench: the cost of a Python `StateCostIntegralObjective.stateCost` override (#362).

Two arms on the sphere sweep of `freespace_hybrid_example.py`: the C++ `stateCost` (no
override) and a Python override returning the same constant, 1.0. One RRTConnect planner on
a fixed seed plans the same path in both arms, so the difference in solve time is the cost
of the Python calls. Both arms interpolate the motion cost, for enough calls to measure. `optimize = True` with an infinite cost threshold: the optimize loop
prices the first solution with the objective once and ends (with `optimize = False` the
planner never builds the objective, so nothing would be measured).

Each run is its own process, seeded before it plans: OMPL's RNG cannot be re-seeded once
planning started (`RNG_setSeed`). Usage: `pixi run python scripts/bench_state_cost.py`.
"""

from __future__ import annotations

import argparse
import json
import math
import platform
import statistics
import subprocess
import sys
import time

import numpy as np

from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.command_language import (
    CompositeInstruction,
    MoveInstruction,
    MoveInstructionPoly,
    MoveInstructionType,
    ProfileDictionary,
    StateWaypoint,
    StateWaypointPoly,
)
from tesseract_robotics.common import GeneralResourceLocator, ManipulatorInfo
from tesseract_robotics.motion_planners import PlannerRequest
from tesseract_robotics.motion_planners_ompl import (
    Cost,
    OMPLMotionPlanner,
    OMPLRealVectorMoveProfile,
    RNG_setSeed,
    RRTConnectConfigurator,
    StateCostIntegralObjective,
)
from tesseract_robotics.planning import Pose, Robot, create_obstacle, sphere

ARMS = ("cpp", "python")
RUNS_PER_ARM = 5  # median of 5 processes per arm
SEED = 1
# The first solution ends the loop (infinite threshold); the cap only bounds a failed run (s).
PLANNING_TIME_S = 2.0
PLANNER_RANGE_RAD = 0.01  # the C++ example's RRTConnect range
OMPL_NAMESPACE = "OMPLMotionPlannerTask"
PROFILE = "FREESPACE"
# freespace_hybrid_example.py: joint_a1 -0.4 -> 0.4 rad around a 0.15 m sphere at (0.5, 0, 0.55) m.
JOINT_START = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
JOINT_END = np.array([0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
SPHERE_RADIUS_M = 0.15
SPHERE_CENTER_M = (0.5, 0.0, 0.55)
# The Python arm's per-state cost: the C++ default's value, so both arms price paths alike.
UNIT_STATE_COST = 1.0
# Both arms split each motion into StateSpace::validSegmentCount parts when pricing it, so a
# solve makes enough stateCost calls to resolve the per-call cost above run-to-run noise
# (without it: 316 calls, inside the C++ arm's own 14.7 to 19.5 ms spread).
INTERPOLATE_MOTION_COST = True


class BenchRunError(RuntimeError):
    """A bench child process failed to plan or exited non-zero."""


class PythonUnitCost(StateCostIntegralObjective):
    """stateCost = 1 in Python, counting its calls."""

    def __init__(self, si) -> None:
        super().__init__(si, INTERPOLATE_MOTION_COST)
        self.calls = 0

    def stateCost(self, values) -> float:
        self.calls += 1
        return UNIT_STATE_COST


class BenchProfile(OMPLRealVectorMoveProfile):
    """Returns the arm's objective with an infinite cost threshold."""

    def __init__(self, arm: str) -> None:
        super().__init__()
        self.arm = arm
        self.objective = None

    def createOptimizationObjective(self, si, env, manip):
        self.objective = (
            PythonUnitCost(si)
            if self.arm == "python"
            else StateCostIntegralObjective(si, INTERPOLATE_MOTION_COST)
        )
        self.objective.setCostThreshold(Cost(math.inf))
        return self.objective


def _request(arm: str) -> tuple[PlannerRequest, BenchProfile]:
    locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    robot = Robot.from_tesseract_support("lbr_iiwa_14_r820", locator)
    create_obstacle(
        robot, "sphere_attached", sphere(SPHERE_RADIUS_M), Pose.from_xyz(*SPHERE_CENTER_M)
    )
    joint_names = robot.get_joint_names("manipulator")
    robot.set_joints(JOINT_START, joint_names=joint_names)

    profile = BenchProfile(arm)
    rrt = RRTConnectConfigurator()
    rrt.range = PLANNER_RANGE_RAD
    profile.solver_config.planners = [rrt]
    profile.solver_config.planning_time = PLANNING_TIME_S
    profile.solver_config.optimize = True

    manip_info = ManipulatorInfo()
    manip_info.manipulator = "manipulator"
    manip_info.tcp_frame = "tool0"
    manip_info.working_frame = "base_link"
    program = CompositeInstruction(PROFILE)
    program.setManipulatorInfo(manip_info)
    for joints in (JOINT_START, JOINT_END):
        wp = StateWaypointPoly(StateWaypoint(joint_names, joints))
        program.push_back(
            MoveInstructionPoly(MoveInstruction(wp, MoveInstructionType.FREESPACE, PROFILE))
        )
    profiles = ProfileDictionary()
    profiles.addProfile(OMPL_NAMESPACE, PROFILE, profile)
    request = PlannerRequest()
    request.instructions = program
    request.env = robot.env
    request.profiles = profiles
    return request, profile


def run_child(arm: str) -> dict:
    """Plan once on `SEED` and time the solve (child process)."""
    request, profile = _request(arm)
    RNG_setSeed(SEED)
    start = time.perf_counter()
    response = OMPLMotionPlanner(OMPL_NAMESPACE).solve(request)
    solve_s = time.perf_counter() - start
    if not response.successful:
        raise BenchRunError(f"{arm}: {response.message}")
    calls = profile.objective.calls if arm == "python" else 0
    return {"arm": arm, "solve_s": solve_s, "calls": calls}


def run_parent() -> dict[str, list[dict]]:
    """Run `RUNS_PER_ARM` child processes per arm."""
    runs: dict[str, list[dict]] = {arm: [] for arm in ARMS}
    for _ in range(RUNS_PER_ARM):
        for arm in ARMS:
            proc = subprocess.run(
                [sys.executable, __file__, "--child", arm],
                capture_output=True,
                text=True,
                check=False,
            )
            if proc.returncode != 0:
                raise BenchRunError(f"{arm} child exited {proc.returncode}:\n{proc.stderr}")
            runs[arm].append(json.loads(proc.stdout.strip().splitlines()[-1]))
    return runs


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--child", choices=ARMS, help="plan once in this process and print JSON")
    args = parser.parse_args()
    if args.child:
        print(json.dumps(run_child(args.child)))
        return 0

    runs = run_parent()
    cpp_s = statistics.median(r["solve_s"] for r in runs["cpp"])
    py_s = statistics.median(r["solve_s"] for r in runs["python"])
    calls = {r["calls"] for r in runs["python"]}
    if len(calls) != 1:
        raise BenchRunError(
            f"the seeded runs priced different paths: stateCost calls {sorted(calls)}"
        )
    (n_calls,) = calls
    print(f"platform: {platform.system()} {platform.machine()}, python {platform.python_version()}")
    print(
        f"runs per arm: {RUNS_PER_ARM}, seed {SEED}, 1 RRTConnect planner, infinite threshold, "
        f"motion cost interpolation {INTERPOLATE_MOTION_COST}"
    )
    print(
        f"C++ stateCost    solve median {cpp_s * 1e3:.1f} ms  {[round(r['solve_s'] * 1e3, 1) for r in runs['cpp']]}"
    )
    print(
        f"Python stateCost solve median {py_s * 1e3:.1f} ms  {[round(r['solve_s'] * 1e3, 1) for r in runs['python']]}"
    )
    print(f"stateCost calls per solve: {n_calls}")
    print(f"calls per second of solve: {n_calls / py_s:.0f}")
    extra_us = (py_s - cpp_s) / n_calls * 1e6
    # The C++ arm's own spread is the noise the median difference has to beat.
    cpp_times = [r["solve_s"] for r in runs["cpp"]]
    noise_us = (max(cpp_times) - min(cpp_times)) / n_calls * 1e6
    print(f"extra time per call: {extra_us:.2f} us (C++ arm spread / calls: {noise_us:.2f} us)")
    if abs(extra_us) < noise_us:
        print(f"per-call cost not resolved: below {noise_us:.2f} us per call on this scene")
    return 0


if __name__ == "__main__":
    sys.exit(main())
