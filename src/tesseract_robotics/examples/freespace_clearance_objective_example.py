"""Freespace clearance objective example: end OMPL's optimize loop on cost, not on time.

A KUKA IIWA sweeps joint_a1 from -1.2 to 1.2 rad through six cubes of different sizes
and orientations, staggered in radius and height along the tool's arc. It is planned twice
with RRTConnect + `FreespaceIfoptPipeline` (#362):

1. The default objective: path length, `optimize = True`. With no solution-count exit the
   optimize loop runs for the whole `planning_time`.
2. A Python `OMPLRealVectorMoveProfile` subclass whose `createOptimizationObjective`
   returns `MultiOptimizationObjective(PathLength x 1, MaximizeMinClearance x w)` with a
   cost threshold. The loop ends at the first path whose cost meets it. Here the first
   path OMPL finds usually meets it already, so the loop usually ends at the first
   solution. With one planner that is the path `optimize = False` gives on 4 of 5 seeds;
   `PATH_LENGTH_THRESHOLD_RAD` states what was measured with two.

The clearance term is inert on this stack: tesseract's OMPL state validators do not
implement `clearance()`, so OMPL reports 0 for every state and the objective reduces to
path length.

Related examples:
- freespace_hybrid_example.py - one sphere, the default objective
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass

import numpy as np

from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.collision import (
    ContactRequest,
    ContactResultMap,
    ContactResultVector,
    ContactTestType,
)
from tesseract_robotics.common import CollisionMarginData, GeneralResourceLocator
from tesseract_robotics.examples.freespace_hybrid_example import add_ifopt_profiles
from tesseract_robotics.motion_planners_ompl import (
    Cost,
    MaximizeMinClearanceObjective,
    MultiOptimizationObjective,
    OMPLRealVectorMoveProfile,
    PathLengthOptimizationObjective,
)
from tesseract_robotics.motion_planners_trajopt_ifopt import (
    TrajOptIfoptOSQPSolverProfile,
)
from tesseract_robotics.planning import (
    MotionProgram,
    Pose,
    Robot,
    StateTarget,
    TaskComposer,
    box,
    create_obstacle,
)
from tesseract_robotics.planning.composer import PlanningResult
from tesseract_robotics.planning.profiles import (
    OMPL_CONTACT_MARGIN_M,
    OMPL_DEFAULT_NAMESPACE,
    TRAJOPT_IFOPT_DEFAULT_NAMESPACE,
    create_freespace_ifopt_pipeline_profiles,
    create_ompl_planner_configurators,
)
from tesseract_robotics.state_solver import OFKTStateSolver
from tesseract_robotics.task_composer import TaskComposerNodeInfo
from tesseract_robotics.trajopt_sqp import SQPCallback

PIPELINE = "FreespaceIfoptPipeline"
PROFILE = "FREESPACE"
OMPL_NODE = "OMPLMotionPlannerTask"

# freespace_hybrid_example.py's arm pose, swept over a wider joint_a1 range: tool0 runs
# along a 0.52 m radius arc at z = 0.62 m, 1.25 m long.
JOINT_START = np.array([-1.2, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
JOINT_END = np.array([1.2, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
# Cubes along the arc: (edge m, centre xyz m, rpy rad). Centres sit at joint_a1
# -0.85 ... 0.85, alternately inside/outside the 0.52 m arc and above/below its height.
CUBES = (
    (0.10, (0.330, -0.376, 0.70), (0.0, 0.0, np.deg2rad(30))),
    (0.14, (0.491, -0.268, 0.55), (np.deg2rad(20), 0.0, np.deg2rad(45))),
    (0.07, (0.473, -0.081, 0.66), (0.0, np.deg2rad(35), 0.0)),
    (0.12, (0.542, 0.093, 0.58), (np.deg2rad(45), 0.0, np.deg2rad(15))),
    (0.08, (0.439, 0.240, 0.68), (0.0, np.deg2rad(-25), np.deg2rad(60))),
    (0.11, (0.356, 0.406, 0.60), (np.deg2rad(30), np.deg2rad(30), np.deg2rad(30))),
)
NUM_PLANNERS = 2  # the C++ example's parallel RRTConnect planners
PLANNER_RANGE_RAD = 0.01  # the C++ example's RRTConnect range

# The freespace profile helpers' default planning_time: the cap of OMPL's optimize loop, which
# `check_loop_exits` compares the OMPL node's time with.
PLANNING_TIME_S = 10.0
# No solution-count exit: only the objective or the time cap ends the optimize loop.
UNREACHABLE_MAX_SOLUTIONS = 1_000_000
# Case 2 stops at the first path no longer than this (joint-space length). On this
# pipeline and these profiles the first path OMPL finds usually meets 4.0 rad already, so
# the threshold usually ends the loop at the first solution. One planner, 5 seeds
# (scripts/ompl_first_path_probe.py): on 4 that is the path `optimize = False` returns;
# seed 2's first path is 4.21 rad and re-solving found a 3.95 rad path. Two planners, seeds 1
# to 3, both modes: 1 of 42 runs differed, a longer `optimize = False` path on seed 3 (3.64 rad
# against 3.57). The planners' race is the inferred cause; 36 repeats did not reproduce it.
# The straight sweep, 2.4 rad, hits the cubes from 5 % to 92 % of the way.
PATH_LENGTH_THRESHOLD_RAD = 4.0
# Weight of the clearance term (inert on this stack, see the module docstring).
CLEARANCE_WEIGHT = 0.5

# Contact reporting distance of the clearance scan (m): beyond the farthest cube a path
# keeps its distance from, so each cube's closest approach is measured, not clipped.
CLEARANCE_REPORT_DISTANCE_M = 0.3
# Joint-space step of the clearance scan between planned states (rad). On joint_a1 it moves
# tool0 0.52 m x 0.005 = 2.6 mm along its arc.
CLEARANCE_SCAN_STEP_RAD = 0.005
# Viewer markers of each cube's closest approach.
CLEARANCE_COLOR_RGB = (220, 40, 40)
CLEARANCE_LINE_THICKNESS_M = 0.003
TOOLPATH_AXES_LENGTH_M = 0.03


class PlanningFailedError(RuntimeError):
    """A case of the example did not produce a plan."""


class ObjectiveNotUsedError(RuntimeError):
    """The planner did not build its objective from the Python profile exactly once."""


class ThresholdDidNotEndLoopError(RuntimeError):
    """Case 2's OMPL node took `planning_time` or longer: nothing showed the threshold ended it."""


class DefaultEndedEarlyError(RuntimeError):
    """Case 1's OMPL node took less than `planning_time`: its optimize loop stopped before the cap."""


class NodeInfoMissingError(LookupError):
    """The run has not exactly one info for a node that is read."""


class CubeContactError(RuntimeError):
    """Case 2's path touches a cube."""


@dataclass(frozen=True)
class Clearance:
    """Closest approach of the robot to one cube along a planned path.

    Attributes:
        cube: Cube link name.
        link: Robot link that comes closest.
        distance_m: Signed distance (m); negative is penetration.
        robot_point_m: Nearest point on `link`, world frame (m).
        cube_point_m: Nearest point on the cube, world frame (m).
    """

    cube: str
    link: str
    distance_m: float
    robot_point_m: np.ndarray
    cube_point_m: np.ndarray


def node_info(result: PlanningResult, name: str) -> TaskComposerNodeInfo:
    """The run's one info for node `name`.

    Raises:
        NodeInfoMissingError: none, or more than one.
    """
    infos = [info for info in result.task_infos if info.name == name]
    if len(infos) != 1:
        raise NodeInfoMissingError(f"{len(infos)} infos for node {name!r}, expected 1")
    return infos[0]


# --8<-- [start:loop_checks]
def check_loop_exits(
    default: PlanningResult, threshold: PlanningResult, planning_time_s: float
) -> None:
    """Check each case's optimize loop against its cap, from the OMPL node's own time.

    The loop's cap is wall clock, so a node that returned inside `planning_time_s` stopped before
    the cap, whatever the machine's load. Load only lengthens a node's time: it cannot raise
    `DefaultEndedEarlyError`, and raises `ThresholdDidNotEndLoopError` only if it pushes case 2's
    OMPL node itself past the cap. The rest of the pipeline does not count.

    A default-case pass is only a lower bound: the node's time also holds setup and whatever load
    adds, so a loop that stopped early, plus setup or load reaching the cap, passes as well.

    Args:
        default: Case 1's plan; nothing but the cap ends its loop.
        threshold: Case 2's plan; its cost threshold ends the loop.
        planning_time_s: OMPL `planning_time` of both cases (s).

    Raises:
        NodeInfoMissingError: a plan has not exactly one OMPL node info.
        DefaultEndedEarlyError: case 1's OMPL node took less than `planning_time_s`.
        ThresholdDidNotEndLoopError: case 2's OMPL node took `planning_time_s` or longer.
    """
    default_s = node_info(default, OMPL_NODE).elapsed_time
    if default_s < planning_time_s:
        raise DefaultEndedEarlyError(
            f"default case's OMPL node took {default_s:.2f} s < planning_time {planning_time_s} s"
        )
    threshold_s = node_info(threshold, OMPL_NODE).elapsed_time
    if threshold_s >= planning_time_s:
        raise ThresholdDidNotEndLoopError(
            f"threshold case's OMPL node took {threshold_s:.2f} s "
            f">= planning_time {planning_time_s} s"
        )


# --8<-- [end:loop_checks]


def closest_approaches(robot: Robot, result: PlanningResult) -> list[Clearance]:
    """Each cube's closest approach to the robot along `result`'s path.

    The path is scanned between planned states in `CLEARANCE_SCAN_STEP_RAD` steps.

    Args:
        robot: The robot whose environment holds the cubes.
        result: A successful plan of the manipulator group.

    Returns:
        One `Clearance` per cube, in `CUBES` order.
    """
    env = robot.env
    joint_names = robot.get_joint_names("manipulator")
    positions = [np.asarray(state.positions) for state in result]
    cubes = [f"cube_{i}" for i in range(len(CUBES))]
    solver = OFKTStateSolver(env.getSceneGraph())
    manager = env.getDiscreteContactManager()
    manager.setActiveCollisionObjects(env.getActiveLinkIds())
    manager.setCollisionMarginData(CollisionMarginData(CLEARANCE_REPORT_DISTANCE_M))
    closest: dict[str, Clearance] = {}
    for start, end in zip(positions[:-1], positions[1:]):
        steps = max(1, int(np.ceil(np.max(np.abs(end - start)) / CLEARANCE_SCAN_STEP_RAD)))
        for fraction in np.linspace(0.0, 1.0, steps + 1):
            solver.setStateByNamesAndValues(joint_names, start + fraction * (end - start))
            manager.setCollisionObjectsTransform(solver.getState().link_transforms)
            contacts = ContactResultMap()
            manager.contactTest(contacts, ContactRequest(ContactTestType.ALL))
            flat = ContactResultVector()
            contacts.flattenMoveResults(flat)
            for index in range(len(flat)):
                contact = flat[index]
                links = [str(link) for link in contact.link_ids]
                if links[0] in cubes:
                    cube_side = 0
                elif links[1] in cubes:
                    cube_side = 1
                else:
                    continue
                cube = links[cube_side]
                if cube in closest and closest[cube].distance_m <= contact.distance:
                    continue
                points = contact.nearest_points
                closest[cube] = Clearance(
                    cube=cube,
                    link=links[1 - cube_side],
                    distance_m=contact.distance,
                    robot_point_m=np.array(points[1 - cube_side]),
                    cube_point_m=np.array(points[cube_side]),
                )
    return [closest[cube] for cube in cubes]


# --8<-- [start:objective_profile]
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


# --8<-- [end:objective_profile]


def _scene() -> tuple[Robot, MotionProgram]:
    robot = Robot.from_tesseract_support(
        "lbr_iiwa_14_r820", GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    )
    for i, (edge_m, centre_m, rpy_rad) in enumerate(CUBES):
        create_obstacle(
            robot, f"cube_{i}", box(edge_m, edge_m, edge_m), Pose.from_xyz_rpy(centre_m, rpy_rad)
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


def _profiles(
    planning_time_s: float,
    ompl_profile: OMPLRealVectorMoveProfile | None,
    *,
    num_planners: int,
    sqp_callback: SQPCallback | None,
):
    profiles = create_freespace_ifopt_pipeline_profiles(
        num_planners=num_planners,
        planner_range=PLANNER_RANGE_RAD,
        planning_time=planning_time_s,
        max_solutions=UNREACHABLE_MAX_SOLUTIONS,
    )
    add_ifopt_profiles(profiles)
    if sqp_callback is not None:
        # The profile helpers' solver profile is a default TrajOptIfoptOSQPSolverProfile; this
        # one differs from it only in the callback.
        solver = TrajOptIfoptOSQPSolverProfile()
        solver.callbacks = [sqp_callback]
        profiles.addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, PROFILE, solver)
    if ompl_profile is not None:
        ompl_profile.solver_config.planners = create_ompl_planner_configurators(
            num_planners=num_planners, range=PLANNER_RANGE_RAD
        )
        ompl_profile.solver_config.planning_time = planning_time_s
        ompl_profile.solver_config.max_solutions = UNREACHABLE_MAX_SOLUTIONS
        ompl_profile.solver_config.optimize = True
        # A fresh OMPLRealVectorMoveProfile has no collision margin; case 1's profile, from the
        # helpers, has OMPL_CONTACT_MARGIN_M. The cases differ only in the objective.
        ompl_profile.contact_manager_config.default_margin = OMPL_CONTACT_MARGIN_M
        profiles.addProfile(OMPL_DEFAULT_NAMESPACE, PROFILE, ompl_profile)
    return profiles


def run_case(
    name: str,
    ompl_profile: OMPLRealVectorMoveProfile | None,
    planning_time_s: float,
    *,
    composer: TaskComposer,
    sqp_callback: SQPCallback | None = None,
    pipeline: str = PIPELINE,
    num_planners: int = NUM_PLANNERS,
) -> dict:
    """Plan the sweep once with `ompl_profile` (None: the default objective).

    Args:
        name: The case's name, for the printout and errors.
        ompl_profile: The OMPL move profile, or None for the profile helpers' default.
        planning_time_s: OMPL `planning_time` (s).
        composer: Composer to plan with; it must provide `pipeline`.
        sqp_callback: Registered on the TrajOptIfopt solver profile, if given.
        pipeline: `FreespaceIfoptPipeline`, or a variant of it with the same nodes.
        num_planners: Parallel OMPL planners (one plans reproducibly on a seed).

    Returns:
        dict with `robot`, `result`, `success` and `wall_time_s`.

    Raises:
        PlanningFailedError: the case did not plan.
    """
    robot, program = _scene()
    profiles = _profiles(
        planning_time_s, ompl_profile, num_planners=num_planners, sqp_callback=sqp_callback
    )
    start = time.perf_counter()
    result = composer.plan(robot, program, pipeline=pipeline, profiles=profiles)
    wall_time_s = time.perf_counter() - start
    print(f"{name:>9}: success={result.successful} wall time {wall_time_s:.2f} s")
    if not result.successful:
        raise PlanningFailedError(f"{name} case: {result.message}")
    return {"robot": robot, "result": result, "success": True, "wall_time_s": wall_time_s}


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
        dict with `default` and `threshold` entries, each holding `robot`, `result`,
        `success` and `wall_time_s`, plus `objective_calls` of case 2's profile and
        `clearances`, each cube's closest approach along case 2's path.

    Raises:
        PlanningFailedError: a case did not plan.
        ObjectiveNotUsedError: the Python objective was not built exactly once.
        NodeInfoMissingError: a plan has not exactly one OMPL node info.
        DefaultEndedEarlyError: case 1's OMPL node took less than `planning_time_s`.
        ThresholdDidNotEndLoopError: case 2's OMPL node took `planning_time_s` or longer.
        CubeContactError: case 2's path touches a cube.
    """
    composer = composer or TaskComposer.from_config()
    composer.warmup([PIPELINE])

    cases: dict = {}
    threshold_profile = LengthAndClearanceProfile(path_length_threshold_rad)
    for name, ompl_profile in (("default", None), ("threshold", threshold_profile)):
        cases[name] = run_case(name, ompl_profile, planning_time_s, composer=composer)

    cases["objective_calls"] = threshold_profile.objective_calls
    if threshold_profile.objective_calls != 1:
        raise ObjectiveNotUsedError(f"built {threshold_profile.objective_calls} times, not once")
    # --8<-- [start:checks]
    # Case 1 has no exit but the time cap; case 2's threshold ends it, so its OMPL node returns
    # inside the planning_time that case 1's OMPL node uses up.
    check_loop_exits(cases["default"]["result"], cases["threshold"]["result"], planning_time_s)
    cases["clearances"] = closest_approaches(
        cases["threshold"]["robot"], cases["threshold"]["result"]
    )
    for clearance in cases["clearances"]:
        print(f"{clearance.cube}: {clearance.distance_m * 1000:.1f} mm from {clearance.link}")
    touching = [c.cube for c in cases["clearances"] if c.distance_m <= 0]
    if touching:
        raise CubeContactError(f"case 2's path touches {touching}")
    # --8<-- [end:checks]
    return cases


# --8<-- [start:markers]
def draw_clearances(viewer, clearances: list[Clearance]) -> None:
    """Mark each cube's closest approach: a line between the nearest points, labelled in mm at the cube.

    Args:
        viewer: The `TesseractViewer` showing the scene.
        clearances: From `closest_approaches`.
    """
    for clearance in clearances:
        segment = np.array([[clearance.robot_point_m, clearance.cube_point_m]])
        viewer.server.scene.add_line_segments(
            f"/clearance/{clearance.cube}",
            segment,
            CLEARANCE_COLOR_RGB,
            thickness=CLEARANCE_LINE_THICKNESS_M,
        )
        viewer.server.scene.add_label(
            f"/clearance/{clearance.cube}/label",
            f"{clearance.distance_m * 1000:.0f} mm",
            position=clearance.cube_point_m,
        )


# --8<-- [end:markers]


def main() -> int:
    cases = run()

    from tesseract_robotics.viewer import TesseractViewer

    # Case 2, the thresholded objective, is the one this example introduces.
    shown = cases["threshold"]
    with TesseractViewer() as viewer:
        viewer.update_environment(shown["robot"].env)
        if shown["result"].raw_results is not None:
            viewer.update_trajectory(shown["result"].raw_results)
            viewer.plot_trajectory(
                shown["result"].raw_results,
                shown["robot"].get_manipulator_info("manipulator", "tool0", "base_link"),
                axes_length=TOOLPATH_AXES_LENGTH_M,
            )
        draw_clearances(viewer, cases["clearances"])
        print(f"Viewer at {viewer.url}")
        input("Press Enter to exit...")

    return 0


if __name__ == "__main__":
    sys.exit(main())
