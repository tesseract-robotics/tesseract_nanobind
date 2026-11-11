"""Online planning through the task composer: replan as an obstacle moves.

This is not a port of tesseract_planning's `online_planning_example.cpp`. That example
builds one `TrajOptQPProblem` and advances a `TrustRegionSQPSolver` one step per
replan, with the human moved by hand through a joint state publisher; the faithful
port is `online_planning_sqp_example.py`. This one plans the same gantry + ABB IRB2400
scene (start, target, model) from scratch through `TrajOptPipeline` each cycle, with
a person walking a scripted loop, to show what a full pipeline call costs per replan.

Scene (tesseract_support `online_planning_example`):
- Gantry: `gantry_axis_1`, `gantry_axis_2`, prismatic.
- ABB IRB2400: `joint_1` .. `joint_6`.
- Human: `human_x_joint`, `human_y_joint`, outside the `manipulator` group, so the
  obstacle moves with `Environment.setState` and no collision manager is rebuilt. A figure to
  scale stands in its cylinder (online_planning_scene.py, shared with the SQP example).

Differences from the C++ example, all deliberate:
- Every cycle is an independent pipeline call: no warm start, no incremental SQP step.
- The person walks online_planning_scene's loop; the plans are spread evenly over one lap.
- The C++ collision settings (margin 0.1 m, coefficient 10) come from
  `create_trajopt_default_profiles`, not from this file.

Related examples:
- online_planning_sqp_example.py: the SQP port of the C++ example.
- freespace_ompl_example.py: static environment planning with OMPL.
- basic_cartesian_example.py: TrajOpt for Cartesian paths.
"""

from __future__ import annotations

import sys
import time

import numpy as np

from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.common import GeneralResourceLocator
from tesseract_robotics.examples import online_planning_scene as scene
from tesseract_robotics.examples.failures import require_success
from tesseract_robotics.planning import (
    MotionProgram,
    Robot,
    StateTarget,
    TaskComposer,
)
from tesseract_robotics.planning.profiles import create_trajopt_default_profiles


def run(
    pipeline="TrajOptPipeline",
    num_iterations=5,
    num_planners=None,
    *,
    composer: TaskComposer | None = None,
):
    """Run online planning example with a walking person.

    Plans `num_iterations` times, spread evenly over one lap of the person's walk, and times
    each plan.

    Args:
        pipeline: Planning pipeline ("TrajOptPipeline" or "TrajOptIfoptPipeline")
        num_iterations: Number of replanning cycles, spread evenly over one lap; the demo
            recording plans once per tick (`scene.ticks_per_lap()`)
        num_planners: Parallel planners (unused for TrajOpt, relevant for OMPL)
        composer: Composer to plan with. Without one, each call builds a new
            composer and loads the task composer plugins again (seconds per
            call); build one with `TaskComposer.from_config()` and pass it to
            every call to load them once.

    Returns:
        dict with 'trajectories', 'ticks' (the walk's tick per plan), 'timings', 'robot',
        'joint_names', 'success'
    """
    # =========================================================================
    # STEP 1: Load Robot Model
    # =========================================================================
    # The online_planning_example model includes:
    # - Gantry: gantry_axis_1, gantry_axis_2 (prismatic, large travel)
    # - ABB IRB2400: joint_1..joint_6 (revolute, standard 6-axis arm)
    # - Human: human_x_joint, human_y_joint (prismatic, for obstacle motion)
    robot = Robot.from_tesseract_support(
        "online_planning_example", GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    )

    # "manipulator" group contains only the 8 gantry+arm joints
    # Human joints are NOT part of the manipulator - they're environment state
    joint_names = robot.get_joint_names("manipulator")
    print(f"Manipulator joints ({len(joint_names)}): {joint_names}")

    # =========================================================================
    # STEP 2: Define Start and Target States
    # =========================================================================
    # Start position: all zeros (gantry at origin, arm in home pose)
    start_position = np.zeros(len(joint_names))
    robot.set_joints(start_position, joint_names=joint_names)
    scene.add_person(robot)

    # online_planning_example.cpp: target_joint_position << 5.5, 3, 0, 0, 0, 0, 0, 0
    target_position = np.array([5.5, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # =========================================================================
    # STEP 3: Configure Planning Pipeline
    # =========================================================================
    # Default profiles provide reasonable TrajOpt settings:
    # - Collision margin and coefficients
    # - Joint velocity/acceleration limits
    # - Optimization parameters
    profiles = create_trajopt_default_profiles()

    # TaskComposer orchestrates the planning pipeline
    # TrajOptPipeline: interpolation -> TrajOpt optimization -> time parameterization
    if composer is None:
        composer = TaskComposer.from_config()

    trajectories = []
    timings = []
    ticks = [round(i * scene.ticks_per_lap() / num_iterations) for i in range(num_iterations)]

    # =========================================================================
    # STEP 4: Replanning Loop
    # =========================================================================
    for iteration, tick in enumerate(ticks):
        # --- Move the person to where its walk is at this tick ---
        scene.move_person(robot.env, tick)

        # --- Get Current Robot State ---
        # In a real system, this would come from robot encoders
        # Here we always start from the initial position (no execution)
        state = robot.get_state(joint_names)
        current_joints = state.joint_positions

        # --- Build Motion Program ---
        # StateTarget: plan in joint space (vs CartesianTarget for Cartesian)
        # First move_to sets start state, second sets goal
        program = (
            MotionProgram(
                "manipulator", tcp_frame="tool0", working_frame="base_link", profile="DEFAULT"
            )
            .set_joint_names(joint_names)
            .move_to(StateTarget(current_joints, names=joint_names, profile="DEFAULT"))
            .move_to(StateTarget(target_position, names=joint_names, profile="DEFAULT"))
        )

        # --- Execute Planning with Timing ---
        start_time = time.perf_counter()
        result = composer.plan(robot, program, pipeline=pipeline, profiles=profiles)
        planning_time = time.perf_counter() - start_time
        require_success(result, f"iteration {iteration + 1}")
        trajectories.append(result)
        timings.append(planning_time)
        print(
            f"Iteration {iteration + 1}: Success in {planning_time:.3f}s, "
            f"{len(result)} waypoints, person at {scene.walk_position_m(tick).round(2)}"
        )
        # A real online planner would execute the first segment while planning the
        # next; this example only times the planning.

    # =========================================================================
    # STEP 5: Report Statistics
    # =========================================================================
    avg_time = float(np.mean(timings))
    print("\nOnline Planning Summary:")
    print(f"  Iterations: {num_iterations}")
    print(f"  Average planning time: {avg_time:.3f}s ({1.0 / avg_time:.1f} Hz)")

    return {
        "trajectories": trajectories,
        "ticks": ticks,
        "timings": timings,
        "robot": robot,
        "joint_names": joint_names,
        "success": True,
    }


def recording_rows(results):
    """One lap of the walk, one row per tick, showing the plan made last before that tick.

    Columns: the manipulator joints, then `scene.HUMAN_JOINTS`, then the time [s].
    """
    plans = [[point.positions for point in result.trajectory] for result in results["trajectories"]]
    plan_ticks = results["ticks"]
    plan_of_tick = [
        max(i for i, start in enumerate(plan_ticks) if start <= tick)
        for tick in range(scene.ticks_per_lap())
    ]
    return scene.playback_rows(plans, plan_of_tick)


def recording_joint_names(results):
    """Column names of `recording_rows`, without the time."""
    return [*results["joint_names"], *scene.HUMAN_JOINTS]


def main() -> int:
    results = run()

    from tesseract_robotics.viewer import TesseractViewer

    with TesseractViewer() as viewer:
        viewer.update_environment(results["robot"].env)
        viewer.update_trajectory(results["trajectories"][-1].raw_results)
        print(f"Viewer at {viewer.url}")
        input("Press Enter to exit...")

    return 0


if __name__ == "__main__":
    sys.exit(main())
