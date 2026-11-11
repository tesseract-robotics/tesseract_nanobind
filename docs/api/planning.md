# tesseract_robotics.planning

High-level planning API. See the [Planning User Guide](../user-guide/planning.md)
and [TaskComposer User Guide](../user-guide/task-composer.md) for walkthroughs.

## Classes

### Robot

Entry point for working with a robot — loads URDF/SRDF, exposes FK/IK, state
management, and access to the underlying `Environment`.

```python
from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.common import GeneralResourceLocator
from tesseract_robotics.planning import Robot

# Bundled test robots
locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])  # the [support] extra
robot = Robot.from_tesseract_support("abb_irb2400", locator)

# From URDF/SRDF (package:// or file:// URLs)
robot = Robot.from_urdf(
    "package://tesseract/support/urdf/abb_irb2400.urdf",
    "package://tesseract/support/urdf/abb_irb2400.srdf",
    locator,
)

# FK / IK — group_name is the first positional argument
pose = robot.fk("manipulator", [0, 0, 0, 0, 0, 0], "tool0")
solution = robot.ik("manipulator", pose, "tool0", seed=[0, 0, 0, 0, 0, 0])  # IKNoSolutionError if unreachable

# Access the underlying Environment
env = robot.env
```

### MotionProgram

Builder for motion sequences. Chain `.move_to(...)` calls with `JointTarget`,
`CartesianTarget`, or `StateTarget` waypoints.

```python
from tesseract_robotics.planning import (
    MotionProgram,
    JointTarget,
    CartesianTarget,
    Pose,
)

program = (
    MotionProgram("manipulator", tcp_frame="tool0", working_frame="base_link")
    .move_to(JointTarget([0, 0, 0, 0, 0, 0]))
    .move_to(CartesianTarget(Pose.from_xyz(0.5, 0.0, 0.5)))
    .move_to(JointTarget([0.5, 0, 0, 0, 0, 0]))
)
```

### TaskComposer

Orchestrates multi-stage planning pipelines.
See [TaskComposer User Guide](../user-guide/task-composer.md) for the full
workflow (including pipeline warmup).

```python
from tesseract_robotics.planning import TaskComposer

composer = TaskComposer.from_config(warmup=True)  # loads the factory and creates the common pipelines
result = composer.plan(robot, program, pipeline="TrajOptPipeline")
```

### Pose

Scalar-last (`qx, qy, qz, qw`) quaternion convention.

```python
from tesseract_robotics.planning import Pose

pose = Pose.from_xyz_quat(0.5, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0)
print(pose.quaternion)  # [qx, qy, qz, qw]
```

## Functions

| Function | Pipeline default | Description |
|---|---|---|
| `plan_freespace(robot, program)` | `TrajOptPipeline` | Optimization-based freespace motion |
| `plan_ompl(robot, program)` | `FreespacePipeline` | OMPL sampling + TrajOpt smoothing |
| `plan_cartesian(robot, program)` | `DescartesDPipeline` | Descartes Cartesian path search |
| `assign_current_state_as_seed(program, robot)` | — | Seed Cartesian waypoints with the current robot state |

All three `plan_*` helpers accept an optional `pipeline=` override and a
`profiles=` `ProfileDictionary`.

## Reference

::: tesseract_robotics.planning
