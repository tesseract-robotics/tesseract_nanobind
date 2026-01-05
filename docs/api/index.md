# API Reference

Auto-generated API documentation from docstrings.

## Modules Overview

### High-Level API

| Module | Description |
|--------|-------------|
| [`tesseract_robotics.planning`](planning.md) | Robot, Planner, Composer classes |

### Core Modules

| Module | Description |
|--------|-------------|
| [`tesseract_robotics.tesseract_common`](tesseract_common.md) | Common types (Isometry3d, etc.) |
| [`tesseract_robotics.tesseract_geometry`](tesseract_geometry.md) | Geometric primitives |
| [`tesseract_robotics.tesseract_scene_graph`](tesseract_scene_graph.md) | Links, joints, scene graph |
| [`tesseract_robotics.tesseract_environment`](tesseract_environment.md) | Environment, commands |
| [`tesseract_robotics.tesseract_collision`](tesseract_collision.md) | Collision managers, contacts |
| [`tesseract_robotics.tesseract_kinematics`](tesseract_kinematics.md) | Kinematic groups, solvers |

### Planning Modules

| Module | Description |
|--------|-------------|
| `tesseract_robotics.tesseract_command_language` | Waypoints, instructions |
| `tesseract_robotics.tesseract_motion_planners` | Base planner types |
| `tesseract_robotics.tesseract_motion_planners_ompl` | OMPL planner |
| `tesseract_robotics.tesseract_motion_planners_trajopt` | TrajOpt planner |
| `tesseract_robotics.tesseract_motion_planners_descartes` | Descartes planner |
| `tesseract_robotics.tesseract_motion_planners_simple` | Simple interpolation |
| `tesseract_robotics.tesseract_task_composer` | Task composition |

### Low-Level SQP Modules

| Module | Description |
|--------|-------------|
| [`tesseract_robotics.ifopt`](ifopt.md) | Base optimization (Bounds, VariableSet, ConstraintSet) |
| [`tesseract_robotics.trajopt_ifopt`](trajopt_ifopt.md) | Robotics constraints (collision, Cartesian, joints) |
| [`tesseract_robotics.trajopt_sqp`](trajopt_sqp.md) | SQP solver (TrustRegionSQPSolver, OSQP) |

## Import Patterns

### High-Level (Recommended)

```python
from tesseract_robotics.planning import Robot, Planner, Composer

robot = Robot.from_tesseract_support("abb_irb2400")
planner = Planner(robot)
```

### Direct Module Access

```python
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_scene_graph import Link, Joint
from tesseract_robotics.tesseract_geometry import Box, Sphere
```

### SQP API

```python
from tesseract_robotics.ifopt import Bounds
from tesseract_robotics.trajopt_ifopt import JointPosition, CartPosConstraint
from tesseract_robotics.trajopt_sqp import TrustRegionSQPSolver, OSQPEigenSolver
```

## Type Conventions

| C++ Type | Python Type |
|----------|-------------|
| `Eigen::Isometry3d` | `tesseract_common.Isometry3d` |
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
