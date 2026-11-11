"""Online planning through the task composer: replan as an obstacle moves.

This is not a port of tesseract_planning's `online_planning_example.cpp`. That example
builds one `TrajOptQPProblem` and advances a `TrustRegionSQPSolver` one step per
replan, with the human moved by hand through a joint state publisher; the faithful
port is `online_planning_sqp_example.py`. This one plans the same gantry + ABB IRB2400
scene (start, target, model) from scratch through `TrajOptPipeline` each cycle, with
the human moved along a scripted sine, to show what a full pipeline call costs per
replan.

Scene (tesseract_support `online_planning_example`):
- Gantry: `gantry_axis_1`, `gantry_axis_2`, prismatic.
- ABB IRB2400: `joint_1` .. `joint_6`.
- Human: `human_x_joint`, `human_y_joint`, outside the `manipulator` group, so the
  obstacle moves with `Environment.setState` and no collision manager is rebuilt.

Differences from the C++ example, all deliberate:
- Every cycle is an independent pipeline call: no warm start, no incremental SQP step.
- The human follows `HUMAN_AMPLITUDE_M * sin(HUMAN_PHASE_STEP_RAD * k)` along x.
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
from tesseract_robotics.examples.failures import require_success
from tesseract_robotics.planning import (
    MotionProgram,
    Robot,
    StateTarget,
    TaskComposer,
)
from tesseract_robotics.planning.profiles import create_trajopt_default_profiles

# Peak displacement of the scripted human along x [m].
HUMAN_AMPLITUDE_M = 0.2
# Phase advance of the human's sine per replanning cycle [rad].
HUMAN_PHASE_STEP_RAD = 0.2


def update_human_position(robot, human_x, human_y):
    """Update human obstacle position by changing joint values.

    The human obstacle is modeled as a kinematic chain in the URDF with
    prismatic joints (human_x_joint, human_y_joint). This allows efficient
    position updates without rebuilding the collision manager.

    Alternatives, not used here: `moveLink` (rebuilds the collision manager) and
    adding and removing links (a full environment edit).

    Args:
        robot: Robot instance with online_planning_example model
        human_x: X position of human obstacle in meters
        human_y: Y position of human obstacle in meters
    """
    # setState() efficiently updates joint values and recomputes transforms
    # The collision manager automatically uses the new positions
    joint_names = ["human_x_joint", "human_y_joint"]
    joint_values = np.array([human_x, human_y])
    robot.env.setState(joint_names, joint_values)


def run(
    pipeline="TrajOptPipeline",
    num_iterations=5,
    num_planners=None,
    *,
    composer: TaskComposer | None = None,
):
    """Run online planning example with moving human obstacle.

    Plans `num_iterations` times while a scripted human obstacle oscillates, and times each
    plan.

    Args:
        pipeline: Planning pipeline ("TrajOptPipeline" or "TrajOptIfoptPipeline")
        num_iterations: Number of replanning cycles to run
        num_planners: Parallel planners (unused for TrajOpt, relevant for OMPL)
        composer: Composer to plan with. Without one, each call builds a new
            composer and loads the task composer plugins again (seconds per
            call); build one with `TaskComposer.from_config()` and pass it to
            every call to load them once.

    Returns:
        dict with 'trajectories', 'timings', 'robot', 'joint_names', 'success'
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

    # =========================================================================
    # STEP 4: Replanning Loop
    # =========================================================================
    for iteration in range(num_iterations):
        # --- Update Dynamic Obstacle ---
        human_x = HUMAN_AMPLITUDE_M * np.sin(HUMAN_PHASE_STEP_RAD * iteration)
        human_y = 0.0
        update_human_position(robot, human_x, human_y)

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
            f"{len(result)} waypoints, human_x={human_x:.3f}"
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
        "timings": timings,
        "robot": robot,
        "joint_names": joint_names,
        "success": True,
    }


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
