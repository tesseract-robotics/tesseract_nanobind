"""Online planning through the task composer: replan as an obstacle moves.

This is not a port of tesseract_planning's `online_planning_example.cpp`. That example
builds one `TrajOptQPProblem` and advances a `TrustRegionSQPSolver` one step per
replan, with the human moved by hand through a joint state publisher; the faithful
port is `online_planning_sqp_example.py`. This one plans the same gantry + ABB IRB2400
model from scratch through `TrajOptPipeline` each cycle, to show what a full pipeline call
costs per replan. Both Python examples plan the same task (online_planning_scene.py): the robot
runs down the cell while a person walks up to its arm and back.

Scene (tesseract_support `online_planning_example`):
- Gantry: `gantry_axis_1`, `gantry_axis_2`, prismatic.
- ABB IRB2400: `joint_1` .. `joint_6`.
- Human: `human_x_joint`, `human_y_joint`, outside the `manipulator` group, so the
  obstacle moves with `Environment.setState` and no collision manager is rebuilt. A figure to
  scale stands in its cylinder (online_planning_scene.py, shared with the SQP example).

Related examples:
- online_planning_sqp_example.py: the same task through the low-level SQP API, after the C++
  example.
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
from tesseract_robotics.planning.profiles import (
    TRAJOPT_DEFAULT_NAMESPACE,
    create_trajopt_default_profiles,
    create_trajopt_profile_pair,
)

# How far beyond the margin the collision cost collects contacts [m], so it sees the person
# coming before the margin is reached.
COLLISION_COST_BUFFER_M = 0.05


def create_profiles(velocity_weights):
    """`create_trajopt_default_profiles`, its DEFAULT composite profile set for the person.

    The hard collision constraint keeps its zero margin; the soft cost drives the arm
    `online_planning_scene.COLLISION_MARGIN_M` clear of the person. The velocity cost weighs
    each joint by `velocity_weights` (one per `manipulator` joint).
    """
    profiles = create_trajopt_default_profiles()
    composite, _ = create_trajopt_profile_pair()
    composite.velocity_coeff = np.asarray(velocity_weights, dtype=np.float64)
    cost = composite.collision_cost_config
    cost.contact_manager_config.default_margin = scene.COLLISION_MARGIN_M
    cost.collision_margin_buffer = COLLISION_COST_BUFFER_M
    # Replaces the DEFAULT composite profile create_trajopt_default_profiles registered
    profiles.addProfile(TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", composite)
    return profiles


def run(
    pipeline="TrajOptPipeline",
    num_iterations=5,
    num_planners=None,
    *,
    composer: TaskComposer | None = None,
):
    """Run online planning example with a walking person.

    Plans `num_iterations` times, spread evenly over `scene.LAPS_PER_RUN` laps of the person's
    walk, and times each plan.

    Args:
        pipeline: Planning pipeline ("TrajOptPipeline" or "TrajOptIfoptPipeline")
        num_iterations: Number of replanning cycles, spread evenly over the run's laps; the demo
            recording plans once per tick (`scene.ticks_per_run()`)
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
    # The robot runs down the cell along x, the arm at zero at both ends
    start_position = np.array(scene.START_JOINTS)
    target_position = np.array(scene.TARGET_JOINTS)
    robot.set_joints(start_position, joint_names=joint_names)
    scene.add_person(robot)
    # The arm's links, not the robot's estop zone, are checked against the person
    scene.check_arm_against_person(robot)

    # =========================================================================
    # STEP 3: Configure Planning Pipeline
    # =========================================================================
    # The default TrajOpt profiles (collision, smoothing, optimization parameters), with the
    # collision cost's margin raised to keep the arm 0.1 m clear of the person, and a gantry
    # step costing more than an arm step
    profiles = create_profiles(scene.velocity_weights(robot))

    # TaskComposer orchestrates the planning pipeline
    # TrajOptPipeline: interpolation -> TrajOpt optimization -> time parameterization
    if composer is None:
        composer = TaskComposer.from_config()

    trajectories = []
    timings = []
    ticks = [round(i * scene.ticks_per_run() / num_iterations) for i in range(num_iterations)]

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
    arm = np.array(
        [
            np.abs([p.positions[2:] for p in result.trajectory]).max(axis=0)
            for result in trajectories
        ]
    ).max(axis=0)
    print("\nOnline Planning Summary:")
    print(f"  Iterations: {num_iterations}")
    print(f"  Average planning time: {avg_time:.3f}s ({1.0 / avg_time:.1f} Hz)")
    print(f"  Arm motion: up to {arm.max():.2f} rad ({joint_names[2 + int(arm.argmax())]})")

    return {
        "trajectories": trajectories,
        "ticks": ticks,
        "timings": timings,
        "robot": robot,
        "joint_names": joint_names,
        "success": True,
    }


def recording_rows(results):
    """The run's laps of the walk, one row per tick, showing the plan made last before it.

    Columns: the manipulator joints, then `scene.HUMAN_JOINTS`, then the time [s].
    """
    plans = [[point.positions for point in result.trajectory] for result in results["trajectories"]]
    plan_ticks = results["ticks"]
    plan_of_tick = [
        max(i for i, start in enumerate(plan_ticks) if start <= tick)
        for tick in range(scene.ticks_per_run())
    ]
    return scene.playback_rows(plans, plan_of_tick)


def recording_joint_names(results):
    """Column names of `recording_rows`, without the time."""
    return [*results["joint_names"], *scene.HUMAN_JOINTS]


def main() -> int:
    # One plan per tick of the run, so the replay shows at each tick the plan made for it
    results = run(num_iterations=scene.ticks_per_run())

    from tesseract_robotics.viewer import TesseractViewer, Trajectory

    print("\n=== Viewer ===")
    print("Person: walks up to the arm and back; the robot sweeps along the plan of each tick")

    env = results["robot"].env
    scene.style_for_viewer(env)
    walk = Trajectory.from_rows(recording_joint_names(results), recording_rows(results))

    with TesseractViewer() as viewer:
        viewer.update_environment(env)
        viewer.update_trajectory(walk)
        print(f"Viewer at {viewer.url}")
        input("Press Enter to exit...")

    return 0


if __name__ == "__main__":
    sys.exit(main())
