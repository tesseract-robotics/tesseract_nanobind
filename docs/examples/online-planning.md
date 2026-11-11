# Online planning

<!-- snippet: ambient
from tests.docs.doc_assets import sqp_scene

globals().update(sqp_scene(robot))
-->

Two examples replan a gantry robot's trajectory while a person walks through the cell.
`online_planning_sqp_example` is a port of tesseract_planning's C++ `online_planning_example.cpp`:
it builds one `TrajOptQPProblem` and, every tick, moves the person and re-solves it with
`TrustRegionSQPSolver`, starting from the previous solution. `online_planning_example` is not a
port: it does the same job through the task composer, running a whole pipeline per plan.

## The person

The scene models the person as a cylinder, radius 0.5 m and 2 m tall, on two prismatic joints;
the robot keeps its own 2 m estop cylinder, on the gantry, out of it. In the C++ example someone
drags the person's joints by hand. Both Python examples share a scripted walk instead
(`online_planning_scene.py`): a closed loop at 1.3 m/s, a comfortable adult walking speed, one
replan every 0.1 s, the spacing of the C++ example's planned waypoints. The lower half of the
loop comes within 1.42 m of the straight path to the target, so the plan bends away from the
person on every lap; the loop keeps clear of the plan's start and target, which stay fixed.

Inside the cylinder stands a figure 1.75 m tall, built from spheres, capsules and boxes with
segment lengths from standard ratios of stature. It is visual only: the planner sees the
cylinder. The figure faces one way throughout, because the scene gives the person no rotation.
The recordings show the robot sweeping along the plan in force at each tick, forward and back,
as the C++ example replays its current plan.

## Overview

--8<-- "docs/demos/players/online_planning_example.html"

Each tick of the loop:

```mermaid
sequenceDiagram
    participant Sensor
    participant Planner
    participant Controller

    loop Every control cycle
        Sensor->>Planner: Obstacle position
        Planner->>Planner: Update environment
        Planner->>Planner: Replan trajectory
        Planner->>Controller: New waypoint
        Controller->>Controller: Execute
    end
```

## Performance comparison

| Method | Use Case |
|--------|----------|
| Task Composer | Moderate scene changes; each `plan` runs a whole pipeline |
| Low-Level SQP (discrete) | Replanning from the last solution; collisions checked at the waypoints |
| Low-Level SQP (LVS continuous) | Swept collision between waypoints; slower per step |

The reference
[`online_planning_sqp_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/online_planning_sqp_example.py)
prints the mean replan step after the initial solve on its 8-DOF gantry workcell, for each
mode. Run it to get numbers for your machine and build; no rate or latency bound is claimed
here.

## Low-level SQP example

--8<-- "docs/demos/players/online_planning_sqp_example.html"

Replanning with discrete collision. The snippets below
are the real shipped source, split into three regions: `setup`, `problem`,
`sqp_loop`. For the accompanying conceptual guide see the
[Low-Level SQP API](../user-guide/low-level-sqp.md) page.

### Setup

```python title="online_planning_sqp_example.py (setup)"
--8<-- "src/tesseract_robotics/examples/online_planning_sqp_example.py:setup"
```

### Problem construction

The problem is a `TrajOptQPProblem`, built as tesseract_planning's
`online_planning_example.cpp` builds it. The start and target poses and one collision constraint
per step are constraint sets, and the joint velocity is a squared cost. Continuous mode sets the
collision check type to `LVS_DISCRETE`, which its evaluator requires; the C++ reference sets
`DISCRETE` there and throws.

```python title="online_planning_sqp_example.py (problem)"
--8<-- "src/tesseract_robotics/examples/online_planning_sqp_example.py:problem"
```

### SQP loop

```python title="online_planning_sqp_example.py (sqp_loop)"
--8<-- "src/tesseract_robotics/examples/online_planning_sqp_example.py:sqp_loop"
```

Run the complete example:

```bash
tesseract_online_planning_sqp_example
```

??? example "Expected output (the timings vary by machine)"
    ```text
    Manipulator joints (8): ['gantry_axis_1', 'gantry_axis_2', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    Building optimization problem (collision: discrete)...
    Running initial global solve...
    Initial solve: cost=3.4399, time=<ms>
    Online replanning (52 iterations)...
    Replan timing: avg=<ms> (<Hz>)
    Trajectory: 12 waypoints x 8 joints
    ```

    Between the second and third line, `problem.print()` dumps the problem's state, as the C++
    reference does. Before the first solve, trajopt prints zero counts and uninitialized
    bounds there.

## Key concepts

### 1. Variable hierarchy (0.34+)

Use `createNodesVariables` to build `NodesVariables` → `Node` → `Var`:

```python
from tesseract_robotics.trajopt_ifopt import Bounds, createNodesVariables

bounds = [Bounds(-3.14, 3.14)] * len(joint_names)  # one Bounds per joint
nodes_variables = createNodesVariables(
    "trajectory", joint_names, initial_states, bounds,
)
vars_list = [node.getVar("joints") for node in nodes_variables.getNodes()]
```

### 2. Warm-start from the previous solution

Between ticks, rebuild the problem with the previous trajectory as the variables' initial values,
as the C++ `setupProblem` does:

```python
from tesseract_robotics.trajopt_ifopt import createNodesVariables

seeded = createNodesVariables("trajectory", joint_names, list(trajectory), bounds)
# ... a new TrajOptQPProblem(seeded) with the constraint and cost sets, setup(), then:
solver.init(problem)
solver.stepSQPSolver()
```

Writing the values into an existing problem with `NodesVariables.setVariables` is not the same:
it only writes the values, and the collision constraints keep evaluating the trajectory they were
set up with. The solver then reports convergence without moving the plan.

### 3. Reset the trust region each tick

`stepSQPSolver` shrinks the trust region — if you don't reset it, successive
ticks get smaller and smaller steps:

```python
solver.stepSQPSolver()
solver.setBoxSize(0.01)  # Reset trust region
```

### 4. Single step vs full solve

- Use `solver.solve(problem)` on the first tick for global convergence.
- Use `solver.stepSQPSolver()` per tick afterwards for incremental optimization. The C++ example
  takes one step per loop, as fast as the solver runs. At a 0.1 s tick and walking speed one step
  of the 0.01 trust box lets the person into the plan's margin, so the example takes up to
  `SQP_STEPS_PER_TICK` (10) steps a tick and stops at the first that converges.

### 5. Sparse collision checking

Check collision every N waypoints for speed:

```python
from tesseract_robotics.trajopt_ifopt import DiscreteCollisionConstraint

# Every other waypoint (faster)
for i in range(0, n_steps, 2):
    collision = DiscreteCollisionConstraint(evaluator, vars_list[i], name=f"collision_{i}")

# Every waypoint (slower, safer)
for i in range(n_steps):
    collision = DiscreteCollisionConstraint(evaluator, vars_list[i], name=f"collision_{i}")
```

## Continuous collision

To check the motion between waypoints, not only the waypoints, use
`ContinuousCollisionConstraint` with an LVS evaluator. Each evaluator requires its own check type
in the collision config, and its constructor raises otherwise: `LVSDiscreteCollisionEvaluator`
needs `LVS_DISCRETE` (the example's continuous mode), and
`LVSContinuousCollisionEvaluator` needs `LVS_CONTINUOUS` or `CONTINUOUS`:

```python
from tesseract_robotics.collision import CollisionEvaluatorType
from tesseract_robotics.trajopt_ifopt import (
    ContinuousCollisionConstraint,
    LVSContinuousCollisionEvaluator,
    TrajOptCollisionConfig,
)

from tesseract_robotics.trajopt_sqp import TrajOptQPProblem

problem = TrajOptQPProblem(nodes_variables)  # over the variables built above
config = TrajOptCollisionConfig(0.1, 10.0)  # margin (m), coefficient
config.collision_check_config.type = CollisionEvaluatorType.LVS_CONTINUOUS

# One constraint per segment between consecutive waypoints. The first segment starts at the
# pinned current state, so its start is fixed.
for i in range(n_steps - 1):
    evaluator = LVSContinuousCollisionEvaluator(manip, env, config, dynamic_environment=True)
    constraint = ContinuousCollisionConstraint(
        evaluator,
        vars_list[i], vars_list[i + 1],
        fixed0=(i == 0), fixed1=False,
        max_num_cnt=config.max_num_cnt,
        name=f"cont_collision_{i}",
    )
    problem.addConstraintSet(constraint)
```

## Visualization

Animate a trajectory in the viewer:

<!-- snippet: skip (starts the viewer on its default port and waits for Enter) -->
```python
from tesseract_robotics.viewer import TesseractViewer, Trajectory

# Rows of joint values plus a time column
dt = 0.1
rows = [wp.tolist() + [i * dt] for i, wp in enumerate(trajectory)]

with TesseractViewer() as viewer:
    viewer.update_environment(robot.env)
    viewer.update_trajectory(Trajectory.from_rows(joint_names, rows))
    print(f"Viewer at {viewer.url}")
    input("Press Enter to exit...")
```

## Task composer alternative

`online_planning_example.py` replans through the task composer instead: each plan runs a whole
pipeline, with profiles and the high-level API, at the cost of rebuilding the problem every
time. It is not a port of the C++ example; the SQP example above is.
See [`online_planning_example.py`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/online_planning_example.py):

```bash
tesseract_online_planning_example
```

## Running the examples

```bash
# Low-level SQP (discrete collision)
tesseract_online_planning_sqp_example

# Task Composer based
tesseract_online_planning_example
```

Or via module invocation:

```bash
pixi run python -m tesseract_robotics.examples.online_planning_sqp_example
pixi run python -m tesseract_robotics.examples.online_planning_example
```

## Performance tuning

### Faster planning

```python
# Fewer waypoints
n_steps = 8  # Instead of 12

# Fewer collision checks
for i in range(0, n_steps, 3):  # Every third waypoint
    ...

# Smaller trust region (faster convergence)
solver.params.initial_trust_box_size = 0.01

# Fewer iterations per step
solver.params.max_iter = 5
```

### Better quality

```python
# More waypoints
n_steps = 20

# Continuous collision
# (use ContinuousCollisionConstraint + LVSContinuousCollisionEvaluator)

# More iterations
solver.params.max_iter = 50

# Tighter convergence
solver.params.min_approx_improve = 1e-5
```

## Next steps

- [Low-Level SQP Guide](../user-guide/low-level-sqp.md) - Full API reference
- [Collision Detection](../user-guide/collision.md) - Collision configuration
