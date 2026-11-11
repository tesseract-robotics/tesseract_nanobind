# API Reference

Auto-generated API documentation from docstrings.

## Modules Overview

### High-Level API

| Module | Description |
|--------|-------------|
| [`tesseract_robotics.planning`](planning.md) | `Robot`, `MotionProgram`, `TaskComposer`, `plan_freespace`/`plan_ompl`/`plan_cartesian` |

### Core Modules

| Module | Description |
|--------|-------------|
| [`tesseract_robotics.common`](common.md) | Common types (Isometry3d, etc.) |
| [`tesseract_robotics.geometry`](geometry.md) | Geometric primitives |
| [`tesseract_robotics.scene_graph`](scene_graph.md) | Links, joints, scene graph |
| [`tesseract_robotics.environment`](environment.md) | Environment, commands |
| [`tesseract_robotics.collision`](collision.md) | Collision managers, contacts |
| [`tesseract_robotics.kinematics`](kinematics.md) | Kinematic groups, solvers |

### Planning Modules

| Module | Description |
|--------|-------------|
| [`tesseract_robotics.command_language`](command_language.md) | Waypoints, instructions |
| [`tesseract_robotics.motion_planners`](motion_planners.md) | Base planner types |
| [`tesseract_robotics.motion_planners_ompl`](motion_planners_ompl.md) | OMPL planner |
| [`tesseract_robotics.motion_planners_trajopt`](motion_planners_trajopt.md) | TrajOpt planner |
| [`tesseract_robotics.motion_planners_descartes`](motion_planners_descartes.md) | Descartes planner |
| [`tesseract_robotics.motion_planners_simple`](motion_planners_simple.md) | Simple interpolation |
| [`tesseract_robotics.task_composer`](task_composer.md) | Task composition |

### Low-Level SQP Modules

| Module | Description |
|--------|-------------|
| [`tesseract_robotics.trajopt_ifopt`](trajopt_ifopt.md) | Variables, constraints, costs (Var, Node, collision, Cartesian) |
| [`tesseract_robotics.trajopt_sqp`](trajopt_sqp.md) | QP problem and SQP solver (TrajOptQPProblem, TrustRegionSQPSolver, OSQP) |

## Import Patterns

### High-Level (Recommended)

```python
from tesseract_robotics.planning import (
    Robot,
    MotionProgram,
    JointTarget,
    plan_freespace,
    TaskComposer,
)

locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])  # the [support] extra
robot = Robot.from_tesseract_support("abb_irb2400", locator)
program = (
    MotionProgram("manipulator", tcp_frame="tool0", working_frame="base_link")
    .move_to(JointTarget([0, 0, 0, 0, 0, 0]))
    .move_to(JointTarget([0.5, 0, 0, 0, 0, 0]))
)
result = plan_freespace(robot, program)

# Or reuse a composer (amortizes plugin loading):
composer = TaskComposer.from_config(warmup=True)
result = composer.plan(robot, program, pipeline="TrajOptPipeline")
```

### Direct Module Access

```python
from tesseract_robotics.environment import Environment
from tesseract_robotics.scene_graph import Link, Joint
from tesseract_robotics.geometry import Box, Sphere
```

### SQP API

```python
from tesseract_robotics.trajopt_ifopt import Bounds, CartPosConstraint, createNodesVariables
from tesseract_robotics.trajopt_sqp import TrajOptQPProblem, TrustRegionSQPSolver, OSQPEigenSolver
```

## Type Conventions

| C++ Type | Python Type |
|----------|-------------|
| `Eigen::Isometry3d` | `common.Isometry3d` |
| `Eigen::VectorXd` | `numpy.ndarray` |
| `Eigen::MatrixXd` | `numpy.ndarray` |
| `std::vector<std::string>` | `list[str]` |
| `std::shared_ptr<T>` | Python object (ref counted) |
| `std::optional<T>` | `T` or `None` |

## Memory Management

Objects are reference-counted via nanobind/pybind. Generally:

- Objects returned by functions are owned by Python
- Objects stored in containers maintain references
- No manual memory management needed

!!! warning "Vector Members"
    C++ `std::vector` members return copies. Use explicit methods:

    ```python
    # Wrong (silent no-op)
    link.visual.append(visual)

    # Correct
    link.addVisual(visual)
    ```
