# Examples

High-level Pythonic API examples for tesseract_robotics.

These examples use `tesseract_robotics.planning` - a clean, Pythonic interface built on top of the low-level bindings. This is the recommended way to use the library.

## Examples

Most modules have a `run()` that returns its results and a `main()` that shows them in the viewer. Each is a console script named `tesseract_<module>` (for example `tesseract_freespace_ompl_example`) and runs with `python -m tesseract_robotics.examples.<module>`.

A planning example raises `PlanningFailedError` when its plan is not successful.

### Planning

| Example | Description |
|---------|-------------|
| `freespace_ompl_example.py` | OMPL freespace planning, smoothed by TrajOpt |
| `freespace_hybrid_example.py` | OMPL then TrajOpt or TrajOptIfopt, planner count and range as in the C++ example |
| `freespace_clearance_objective_example.py` | End OMPL's optimize loop on a path-length threshold; clearance objective |
| `basic_cartesian_example.py` | Cartesian path with TrajOpt, with a point-cloud octree obstacle |
| `glass_upright_example.py` | Orientation-constrained motion: keep the glass upright |
| `puzzle_piece_auxillary_axes_example.py` | 9-DOF Cartesian toolpath: KUKA IIWA with a 2-axis positioner |
| `pick_and_place_example.py` | Pick, attach, place |
| `car_seat_example.py` | Multi-phase pick and place with meshes |
| `raster_example.py` | Industrial raster pattern (welding, milling) |
| `chain_example.py` | Descartes then TrajOpt, chained raster |
| `online_planning_example.py` | Replan through the task composer as an obstacle moves |
| `online_planning_sqp_example.py` | Port of the C++ online planning example: `TrajOptQPProblem` and `TrustRegionSQPSolver` |
| `python_constraints_example.py` | Python-defined constraints in one live solver loop |
| `pointcloud_octree_collision_example.py` | Point cloud to octree; collision-aware planning around it |
| `pythonic_api_example.py` | Tour of the `tesseract_robotics.planning` API |

### Environment, geometry, collision and kinematics

| Example | Description |
|---------|-------------|
| `tesseract_collision_example.py` | Collision checking |
| `contact_managers_example.py` | Contact managers and their configuration |
| `trajectory_collision_example.py` | Collision checks along a trajectory |
| `sdf_collision_example.py` | Signed distance field obstacle and the distances a contact manager reports |
| `tesseract_kinematics_example.py` | Forward and inverse kinematics |
| `kinematics_plugins_example.py` | Kinematics plugin selection and configuration |
| `kinematics_analysis_example.py` | Jacobians, singularities, manipulability |
| `scene_graph_example.py` | Scene graph construction and queries |
| `state_solver_example.py` | State solver |
| `environment_commands_example.py` | Environment commands (add, move, remove links) |
| `environment_state_example.py` | Environment state and TCP offsets |
| `plugin_configuration_example.py` | Plugin configuration files |
| `geometry_showcase_example.py` | Every geometry type |
| `mesh_roundtrip_example.py` | Mesh import and export round trip |
| `convex_decomposition_example.py` | Convex decomposition of a mesh |

### Viewer

| Example | Description |
|---------|-------------|
| `shapes_viewer.py` | Primitive shapes |
| `tesseract_material_mesh_viewer.py` | GLB and DAE meshes with materials |
| `abb_irb2400_viewer.py` | ABB IRB2400, OMPL then TrajOpt, animated |
| `twc_workcell_positioner_viewer.py` | Industrial workcell (needs the submodule: `git submodule update --init`; `--twc-support DIR` or `TWC_SUPPORT_PATH` for another checkout) |

## Quick Start

```python
from tesseract_robotics.planning import (
    Robot,
    MotionProgram,
    CartesianTarget,
    JointTarget,
    Pose,
    TaskComposer,
)

# Load robot
robot = Robot.from_urdf("package://my_robot/urdf/robot.urdf", "package://my_robot/urdf/robot.srdf")

# Build motion program
program = (
    MotionProgram("manipulator", tcp_frame="tool0", working_frame="base_link")
    .move_to(JointTarget([0, 0, 0, -1.57, 0, 0, 0]))
    .move_to(CartesianTarget(Pose.from_xyz(0.5, 0.0, 0.5)))
)

# Plan
composer = TaskComposer.from_config()
result = composer.plan(robot, program)
if result.successful:
    print(f"Trajectory: {len(result)} waypoints")
```

## Performance Note

Building a `TaskComposer` loads the task composer plugins (`dlopen`); every
`TaskComposer.from_config()` loads them again, so build one per process and pass it as
`composer=` to each `run()`:

```python
composer = TaskComposer.from_config()
composer.warmup(["TrajOptPipeline"])  # load the pipeline's plugins now
result = composer.plan(robot, program)
```

Measured on an Apple M1 Max (10 cores, 64 GB), macOS, release build: `TaskComposer.from_config()`
takes 0.3 to 1.0 s, a following `warmup(["TrajOptPipeline"])` 0.01 s, and `basic_cartesian_example`
(robot load included) 1.9 s for the first plan and 1.5 s for the next ones. Timings differ by
machine and build; do not time a solve through a fresh composer.

## Low-Level API

For direct access to the C++ bindings, see [lowlevel/](lowlevel/).
