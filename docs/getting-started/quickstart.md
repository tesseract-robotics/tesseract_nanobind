# Quickstart

This guide walks you through your first motion planning task using the high-level `tesseract_robotics.planning` API.

## Load a Robot

```python
from tesseract_robotics.planning import Robot

# Load a bundled example robot (ABB IRB2400, KUKA LBR IIWA, etc.)
robot = Robot.from_tesseract_support("abb_irb2400")

# Or from your own URDF/SRDF via package:// URLs
robot = Robot.from_urdf(
    urdf_url="package://my_robot/urdf/robot.urdf",
    srdf_url="package://my_robot/urdf/robot.srdf",
)

# Or from local files
robot = Robot.from_files("/path/to/robot.urdf", "/path/to/robot.srdf")
```

## Forward Kinematics

```python
import numpy as np

joints = np.zeros(6)
# group_name is the first positional argument; tip_link defaults to the chain's tip.
tcp_pose = robot.fk("manipulator", joints)
print(f"Position: {tcp_pose.position}")
print(f"Quaternion (scalar-last, matches scipy): {tcp_pose.quaternion}")
```

## Inverse Kinematics

```python
from tesseract_robotics.planning import Pose

# Scalar-last quaternion convention (matches scipy.spatial.transform.Rotation)
target = Pose.from_xyz_rpy(0.6, 0.0, 0.5, 0.0, 0.0, 0.0)
ik_solution = robot.ik("manipulator", target, seed=joints)

if ik_solution is not None:
    print(f"IK solution: {ik_solution}")
else:
    print("Unreachable target")
```

## Collision Checking

Use the environment's discrete contact manager directly:

```python
from tesseract_robotics.tesseract_collision import (
    ContactRequest,
    ContactResultMap,
    ContactTestType_ALL,
)

robot.set_joints({"joint_1": 0.5, "joint_2": -0.3})

manager = robot.env.getDiscreteContactManager()
manager.setActiveCollisionObjects(robot.env.getActiveLinkNames())
manager.setCollisionObjectsTransform(robot.env.getState().link_transforms)

contacts = ContactResultMap()
manager.contactTest(contacts, ContactRequest(ContactTestType_ALL))
print(f"Collision-free: {contacts.size() == 0}")
```

## Motion Planning (freespace)

```python
from tesseract_robotics.planning import (
    MotionProgram, JointTarget, CartesianTarget, plan_freespace,
)

program = (
    MotionProgram("manipulator")
    .move_to(JointTarget(np.zeros(6)))
    .move_to(CartesianTarget(target))
)

result = plan_freespace(robot, program)
if result.successful:
    for i, state in enumerate(result.trajectory):
        print(f"[{i}] {state.positions}")
```

## Cartesian Planning (Descartes)

The same `program` works with different planning backends — here Descartes for dense Cartesian toolpaths:

```python
from tesseract_robotics.planning import plan_cartesian

result = plan_cartesian(robot, program)
```

## OMPL Planning (pure sampling)

Or OMPL for pure sampling-based search:

```python
from tesseract_robotics.planning import plan_ompl

result = plan_ompl(robot, program)
```

## Using the Task Composer (Advanced)

For multi-stage pipelines (sampling then optimization then time parameterization):

```python
from tesseract_robotics.planning import TaskComposer

composer = TaskComposer.from_config()
composer.warmup(["TrajOptPipeline"])  # optional: pay plugin load cost up front
result = composer.plan(robot, program, pipeline="TrajOptPipeline")
```

See [`TaskComposer.get_available_pipelines()`](../user-guide/task-composer.md) for the full list of 36 pipelines.

## Visualization

```python
from tesseract_robotics.viewer import TesseractViewer

viewer = TesseractViewer()
viewer.update_environment(robot.env, [0, 0, 0])
if result.successful:
    viewer.update_trajectory(result.raw_results)

viewer.start_serve_background()
print("Open http://localhost:8000 in your browser")
input("Press Enter to exit...")
```

## Installation

```bash
pip install tesseract-robotics-nanobind
```

See [Installation](installation.md) for developer-mode setup via pixi.

## Next Steps

- [Core Concepts](concepts.md) — Understand the architecture
- [Motion Planning Guide](../user-guide/planning.md) — Deep dive into planners
- [Examples](../examples/index.md) — Working examples
