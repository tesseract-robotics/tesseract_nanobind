"""
Basic Cartesian Planning Example (High-Level API)

Demonstrates multi-phase Cartesian motion planning using TrajOpt trajectory optimization.
The robot (KUKA IIWA 7-DOF) executes a sequence of FREESPACE and LINEAR moves around an obstacle.

Pipeline Overview:
    1. Load KUKA IIWA robot from tesseract_support
    2. Build the C++ example's octomap from point-cloud data: a 1 m cube of points at
       0.05 m pitch, voxelised into a 0.1 m box octree attached at (1.0, 0, 0)
    3. Create 4-phase motion program:
        - Phase 1: Start at known joint configuration
        - Phase 2: FREESPACE move to Cartesian waypoint 1 (0.5, -0.2, 0.62)
        - Phase 3: LINEAR move to Cartesian waypoint 2 (0.5, 0.3, 0.62)
        - Phase 4: FREESPACE return to start joint configuration
    4. Execute TrajOptPipeline: seed trajectory -> optimization -> time parameterization

Key Concepts:
    - TrajOpt: Trajectory optimizer minimizing costs (smoothness, collision) with constraints
    - FREESPACE motion: Any collision-free path to goal (joint-space interpolation)
    - LINEAR motion: Straight-line Cartesian path (tool pose interpolated along line)
    - Profile names: Control per-waypoint behavior ("freespace_profile", "RASTER")
    - StateTarget vs CartesianTarget: Joint config vs 6D pose goal specification

Motion Types:
    - move_to(CartesianTarget): FREESPACE motion to pose (any collision-free path)
    - linear_to(CartesianTarget): LINEAR motion (straight-line Cartesian path required)
    - move_to(StateTarget): Return to known joint configuration

Quaternion Note:
    Project canonical: scalar-last `[qx, qy, qz, qw]`. The C++ literal
    `Eigen::Quaterniond(0, 0, 1.0, 0)` is scalar-first — w=0, x=0, y=1, z=0 —
    a 180° rotation about Y that points the tool's z axis down. Reordered to
    scalar-last it is `[0, 1.0, 0, 0]`. Copying the literal unchanged gives
    `[0, 0, 1.0, 0]`, a 180° rotation about Z, which leaves the tool pointing
    *up*: against the C++ obstacle that pose is infeasible, and TrajOpt ends at
    OPT_SCO_ITERATION_LIMIT.

C++ Source: tesseract_planning/tesseract_examples/src/basic_cartesian_example.cpp

C++ Parameters (verified):
    - Robot: KUKA LBR IIWA 14 R820 (7-DOF)
    - Obstacle: octomap from a 20 x 20 x 20 point cloud at 0.05 m pitch, 0.1 m
      box octree (1000 boxes, a solid 1 m cube) at (1.0, 0, 0) on base_link
    - Start joints: [-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0]
    - wp1: (0.5, -0.2, 0.62), Eigen quat (w=0, x=0, y=1, z=0) = scalar-last [0, 1.0, 0, 0]
    - wp2: (0.5, 0.3, 0.62) with same orientation
    - TrajOpt profiles: composite "cartesian_program" (LVS_DISCRETE collision cost at
      0.025 m and constraint at 0.0 m, coefficient 1, velocity smoothing); move profile
      with Cartesian and joint constraints for "RASTER" and "freespace_profile"

Related Examples:
    - glass_upright_example.py - Orientation-constrained TrajOpt
    - puzzle_piece_example.py - Complex Cartesian toolpath from CSV
    - freespace_ompl_example.py - Joint-space planning with OMPL
    - lowlevel/basic_cartesian_c_api_example.py - Same with low-level API
"""

import sys

import numpy as np

from tesseract_robotics.planning import (
    CartesianTarget,
    MotionProgram,
    Pose,
    Robot,
    StateTarget,
    TaskComposer,
    create_obstacle,
)
from tesseract_robotics.planning.profiles import (
    TRAJOPT_DEFAULT_NAMESPACE,
    create_freespace_pipeline_profiles,
    create_trajopt_default_profiles,
)
from tesseract_robotics.tesseract_collision import CollisionEvaluatorType
from tesseract_robotics.tesseract_command_language import ProfileDictionary
from tesseract_robotics.tesseract_geometry import Octree, OctreeSubType, PointCloud, createOctree
from tesseract_robotics.tesseract_motion_planners_trajopt import (
    ProfileDictionary_addTrajOptCompositeProfile,
    ProfileDictionary_addTrajOptMoveProfile,
    TrajOptCollisionConfig,
    TrajOptDefaultCompositeProfile,
    TrajOptDefaultMoveProfile,
)

TesseractViewer = None
if "pytest" not in sys.modules:
    from tesseract_robotics.viewer import TesseractViewer

# C++ addPointCloud(): points every 0.05 m across a 1 m cube, voxelised at twice that pitch.
POINT_PITCH_M = 0.05
OCTREE_ORIGIN = (1.0, 0.0, 0.0)


def add_point_cloud_octree(robot):
    """Attach the C++ example's octomap, built from point-cloud data, as `octomap_attached`.

    Mirrors ``addPointCloud()`` in basic_cartesian_example.cpp: a 20 x 20 x 20 grid of points at
    0.05 m pitch spanning [-0.5, 0.45] m, ``createOctree(cloud, 2 * pitch, prune=False,
    binary=True)``, BOX sub-shapes, on ``base_link`` at (1.0, 0, 0). Every 0.1 m voxel holds points,
    so the octree is a solid 1 m cube of 1000 boxes — the collision load of a scanned part, with no
    gaps to route through. Swap the grid for a scanner's points and the rest is unchanged.
    """
    count = round(1.0 / POINT_PITCH_M)
    cloud = PointCloud()
    for i in range(count):
        for j in range(count):
            for k in range(count):
                cloud.addPoint(
                    -0.5 + i * POINT_PITCH_M, -0.5 + j * POINT_PITCH_M, -0.5 + k * POINT_PITCH_M
                )
    tree = createOctree(cloud, 2 * POINT_PITCH_M, prune=False, binary=True)
    octree = Octree(tree, OctreeSubType.BOX, pruned=False, binary_octree=True)
    create_obstacle(robot, "octomap_attached", octree, Pose.from_xyz(*OCTREE_ORIGIN))
    return octree


def _discrete_collision_config(margin, coeff):
    """``trajopt_common::TrajOptCollisionConfig(margin, coeff)``, enabled, checked LVS-discrete.

    The nested ``collision_check_config`` is configured and assigned back whole, so the write
    does not depend on whether the binding's getter returns a reference or a copy.
    """
    config = TrajOptCollisionConfig(margin, coeff)
    config.enabled = True
    check = config.collision_check_config
    check.type = CollisionEvaluatorType.LVS_DISCRETE
    config.collision_check_config = check
    return config


def basic_cartesian_trajopt_profiles(joint_count):
    """The C++ example's TrajOpt profiles, value for value, under the names its program uses."""
    composite = TrajOptDefaultCompositeProfile()
    composite.collision_cost_config = _discrete_collision_config(0.025, 1.0)
    composite.collision_constraint_config = _discrete_collision_config(0.0, 1.0)
    composite.smooth_velocities = True
    composite.smooth_accelerations = False
    composite.smooth_jerks = False
    composite.velocity_coeff = np.ones(1)

    move = TrajOptDefaultMoveProfile()
    cartesian_cost = move.cartesian_cost_config
    cartesian_cost.enabled = False
    move.cartesian_cost_config = cartesian_cost
    cartesian = move.cartesian_constraint_config
    cartesian.enabled = True
    cartesian.coeff = np.ones(6)
    move.cartesian_constraint_config = cartesian
    joint_cost = move.joint_cost_config
    joint_cost.enabled = False
    move.joint_cost_config = joint_cost
    joint = move.joint_constraint_config
    joint.enabled = True
    joint.coeff = np.ones(joint_count)
    move.joint_constraint_config = joint

    profiles = ProfileDictionary()
    ProfileDictionary_addTrajOptCompositeProfile(
        profiles, TRAJOPT_DEFAULT_NAMESPACE, "cartesian_program", composite
    )
    for name in ("RASTER", "freespace_profile"):
        ProfileDictionary_addTrajOptMoveProfile(profiles, TRAJOPT_DEFAULT_NAMESPACE, name, move)
    return profiles


def run(pipeline="TrajOptPipeline", num_planners=None):
    """Run basic Cartesian planning example.

    Plans a multi-phase trajectory combining FREESPACE and LINEAR moves using
    TrajOpt trajectory optimization. Demonstrates mixing motion types and
    profile selection in a single motion program.

    Args:
        pipeline: Planning pipeline to use. Options:
            - "TrajOptPipeline" (default): TrajOpt trajectory optimization
            - "FreespacePipeline": OMPL for FREESPACE moves (ignores LINEAR)
        num_planners: Number of parallel OMPL planners (only for FreespacePipeline).

    Returns:
        dict with keys:
            - result: PlanningResult with trajectory and success status
            - robot: Robot instance with environment state
            - joint_names: List of 7 KUKA IIWA joint names
    """
    # Load KUKA IIWA 7-DOF robot
    robot = Robot.from_tesseract_support("lbr_iiwa_14_r820")

    # The obstacle is the C++ example's octomap, built from point-cloud data
    octree = add_point_cloud_octree(robot)
    print(f"Octomap: {octree.calcNumSubShapes()} boxes at {OCTREE_ORIGIN}")

    # Get joint names and set initial configuration
    joint_names = robot.get_joint_names("manipulator")
    joint_pos = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
    robot.set_joints(joint_pos, joint_names=joint_names)

    # Create Cartesian waypoints (6D poses), tool pointing down. The C++
    # Eigen::Quaterniond(0, 0, 1.0, 0) is scalar-first (w=0, x=0, y=1, z=0):
    # 180° about Y. In the project's scalar-last order that is [0, 1.0, 0, 0].
    wp1 = Pose.from_xyz_quat([0.5, -0.2, 0.62], [0, 1.0, 0, 0])
    wp2 = Pose.from_xyz_quat([0.5, 0.3, 0.62], [0, 1.0, 0, 0])

    # Build 4-phase motion program:
    # 1. StateTarget: Start from known joint configuration
    # 2. CartesianTarget + move_to: FREESPACE to wp1 (any collision-free path)
    # 3. CartesianTarget + linear_to: LINEAR to wp2 (straight-line Cartesian path)
    # 4. StateTarget: FREESPACE return to start joints
    program = (
        MotionProgram("manipulator", tcp_frame="tool0", profile="cartesian_program")
        .set_joint_names(joint_names)
        .move_to(StateTarget(joint_pos, names=joint_names, profile="freespace_profile"))
        .move_to(CartesianTarget(wp1, profile="freespace_profile"))  # FREESPACE to pose
        .linear_to(CartesianTarget(wp2, profile="RASTER"))  # LINEAR Cartesian path
        .move_to(StateTarget(joint_pos, names=joint_names, profile="freespace_profile"))
    )

    # Select profiles based on pipeline type
    # TrajOptPipeline: the C++ example's own profiles, under the names the program uses
    # OMPL profiles configure RRTConnect planner parameters
    if pipeline == "TrajOptPipeline":
        profiles = basic_cartesian_trajopt_profiles(len(joint_names))
    elif "Freespace" in pipeline or "OMPL" in pipeline:
        profiles = create_freespace_pipeline_profiles(num_planners=num_planners)
    else:
        profiles = create_trajopt_default_profiles()

    # Execute planning
    composer = TaskComposer.from_config()
    result = composer.plan(robot, program, pipeline=pipeline, profiles=profiles)

    assert result.successful, f"Planning failed: {result.message}"
    print(f"Planning successful! Trajectory: {len(result)} waypoints")

    return {"result": result, "robot": robot, "joint_names": joint_names}


def main():
    results = run()
    if TesseractViewer is not None and results["result"].raw_results is not None:
        print("\nStarting viewer at http://localhost:8000")
        viewer = TesseractViewer()
        viewer.update_environment(results["robot"].env, [0, 0, 0])
        viewer.update_trajectory(results["result"].raw_results)
        viewer.start_serve_background()
        input("Press Enter to exit...")
    return True


if __name__ == "__main__":
    main()
