"""Freespace clearance objective example: end OMPL's optimize loop on cost, not on time.

The KUKA IIWA sweep of `freespace_hybrid_example.py` (joint_a1 -0.4 -> 0.4 rad around a
0.15 m sphere at (0.5, 0, 0.55)), planned twice with RRTConnect + `FreespaceIfoptPipeline`
(#362):

1. Today's objective: path length, `optimize = True`. With no solution-count exit the
   optimize loop runs for the whole `planning_time`.
2. A Python `OMPLRealVectorMoveProfile` subclass whose `createOptimizationObjective`
   returns `MultiOptimizationObjective(PathLength x 1, MaximizeMinClearance x w)` with a
   cost threshold. The loop ends at the first path whose cost meets it.

The clearance term is inert on this stack: tesseract's OMPL state validators do not
implement `clearance()`, so OMPL reports 0 for every state and the objective reduces to
path length. The objective is built as the issue specifies so that it takes effect once
upstream reports clearance. A third case, cost convergence, waits for the upstream
termination option (#362 item 3).

Related examples:
- freespace_hybrid_example.py - the same scene with the default objective
"""

from __future__ import annotations

import sys
import time

import numpy as np

from tesseract_robotics.examples.freespace_hybrid_example import add_ifopt_profiles
from tesseract_robotics.planning import (
    MotionProgram,
    Pose,
    Robot,
    StateTarget,
    TaskComposer,
    create_obstacle,
    sphere,
)
from tesseract_robotics.planning.profiles import (
    OMPL_DEFAULT_NAMESPACE,
    create_freespace_ifopt_pipeline_profiles,
    create_ompl_planner_configurators,
)
from tesseract_robotics.tesseract_motion_planners_ompl import (
    Cost,
    MaximizeMinClearanceObjective,
    MultiOptimizationObjective,
    OMPLRealVectorMoveProfile,
    PathLengthOptimizationObjective,
)

PIPELINE = "FreespaceIfoptPipeline"
PROFILE = "FREESPACE"

# freespace_hybrid_example.py, from the C++ example.
JOINT_START = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
JOINT_END = np.array([0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
SPHERE_RADIUS_M = 0.15
SPHERE_POSITION_M = (0.5, 0.0, 0.55)
NUM_PLANNERS = 2  # the C++ example's parallel RRTConnect planners
PLANNER_RANGE_RAD = 0.01  # the C++ example's RRTConnect range

# Long enough that the time cap, not the planner, ends case 1.
PLANNING_TIME_S = 5.0
# No solution-count exit: only the objective or the time cap ends the optimize loop.
UNREACHABLE_MAX_SOLUTIONS = 1_000_000
# Joint-space path length the first RRTConnect paths on this scene reach (5.2 to 5.7 rad
# over seeds 1-5; the straight sweep, 0.8 rad, collides). Case 2 stops at the first path
# no longer than this.
PATH_LENGTH_THRESHOLD_RAD = 6.0
# Weight of the clearance term (inert on this stack, see the module docstring).
CLEARANCE_WEIGHT = 0.5


class LengthAndClearanceProfile(OMPLRealVectorMoveProfile):
    """OMPL move profile: path length x 1 + max-min clearance x w, with a cost threshold.

    Args:
        cost_threshold: The optimize loop ends at the first path with a cost at or below it.
    """

    def __init__(self, cost_threshold: float) -> None:
        super().__init__()
        self.cost_threshold = cost_threshold
        self.objective_calls = 0

    def createOptimizationObjective(self, si, env, manip):
        """Build the weighted objective the planner's optimize loop checks.

        Args:
            si: The problem's OMPL `SpaceInformation`.
            env: The planning environment.
            manip: The planning `JointGroup`.

        Returns:
            The `MultiOptimizationObjective`.
        """
        self.objective_calls += 1
        objective = MultiOptimizationObjective(si)
        objective.addObjective(PathLengthOptimizationObjective(si), 1.0)
        objective.addObjective(MaximizeMinClearanceObjective(si), CLEARANCE_WEIGHT)
        objective.setCostThreshold(Cost(self.cost_threshold))
        return objective


def _scene() -> tuple[Robot, MotionProgram]:
    robot = Robot.from_tesseract_support("lbr_iiwa_14_r820")
    create_obstacle(
        robot, "sphere_attached", sphere(SPHERE_RADIUS_M), Pose.from_xyz(*SPHERE_POSITION_M)
    )
    joint_names = robot.get_joint_names("manipulator")
    robot.set_joints(JOINT_START, joint_names=joint_names)
    program = (
        MotionProgram("manipulator", tcp_frame="tool0", working_frame="base_link", profile=PROFILE)
        .set_joint_names(joint_names)
        .move_to(StateTarget(JOINT_START, names=joint_names, profile=PROFILE))
        .move_to(StateTarget(JOINT_END, names=joint_names, profile=PROFILE))
    )
    return robot, program


def _profiles(planning_time_s: float, ompl_profile: OMPLRealVectorMoveProfile | None):
    profiles = create_freespace_ifopt_pipeline_profiles(
        num_planners=NUM_PLANNERS,
        planner_range=PLANNER_RANGE_RAD,
        planning_time=planning_time_s,
        max_solutions=UNREACHABLE_MAX_SOLUTIONS,
    )
    add_ifopt_profiles(profiles)
    if ompl_profile is not None:
        ompl_profile.solver_config.planners = create_ompl_planner_configurators(
            num_planners=NUM_PLANNERS, range=PLANNER_RANGE_RAD
        )
        ompl_profile.solver_config.planning_time = planning_time_s
        ompl_profile.solver_config.max_solutions = UNREACHABLE_MAX_SOLUTIONS
        ompl_profile.solver_config.optimize = True
        profiles.addProfile(OMPL_DEFAULT_NAMESPACE, PROFILE, ompl_profile)
    return profiles


def run(
    planning_time_s: float = PLANNING_TIME_S,
    path_length_threshold_rad: float = PATH_LENGTH_THRESHOLD_RAD,
    *,
    composer: TaskComposer | None = None,
) -> dict:
    """Plan the sweep with the default objective, then with the thresholded one.

    Args:
        planning_time_s: OMPL `planning_time` of both cases (s).
        path_length_threshold_rad: Case 2's cost threshold (joint-space path length, rad).
        composer: Composer to plan with; one is built (loading the plugins) if omitted.

    Returns:
        dict with `default` and `threshold` entries, each holding `result`, `success`
        and `wall_time_s`, plus `objective_calls` of case 2's profile.
    """
    composer = composer or TaskComposer.from_config()
    composer.warmup([PIPELINE])

    cases: dict = {}
    threshold_profile = LengthAndClearanceProfile(path_length_threshold_rad)
    for name, ompl_profile in (("default", None), ("threshold", threshold_profile)):
        robot, program = _scene()
        profiles = _profiles(planning_time_s, ompl_profile)
        start = time.perf_counter()
        result = composer.plan(robot, program, pipeline=PIPELINE, profiles=profiles)
        wall_time_s = time.perf_counter() - start
        cases[name] = {"result": result, "success": result.successful, "wall_time_s": wall_time_s}
        print(f"{name:>9}: success={result.successful} wall time {wall_time_s:.2f} s")

    cases["objective_calls"] = threshold_profile.objective_calls
    return cases


def main() -> int:
    cases = run()
    ok = cases["default"]["success"] and cases["threshold"]["success"]
    ok = ok and cases["threshold"]["wall_time_s"] < PLANNING_TIME_S
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
