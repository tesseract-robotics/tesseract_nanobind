# Quickstart

One robot, the ABB IRB 2400, from loading to a planned and played trajectory: forward and
inverse kinematics, a collision check, then three planners on the same program. Run the blocks
in order in one Python session; each builds on the names the previous ones defined.

```bash
pip install --pre "tesseract-robotics-nanobind[support]"
```

The `[support]` extra installs tesseract's example robots. [Installation](installation.md) covers
platforms and source builds.

## Load a robot

```python
from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.common import GeneralResourceLocator
from tesseract_robotics.planning import Robot

# Resolves package://tesseract/... URLs to the [support] extra's files
locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
robot = Robot.from_tesseract_support("abb_irb2400", locator)

print(robot.get_joint_names("manipulator"))  # the SRDF group's six joints
```

Your own robot loads from its URDF and SRDF:

<!-- snippet: skip (loads a robot from the reader's own files; the paths are placeholders) -->
```python
robot = Robot.from_urdf(
    "package://my_robot/urdf/robot.urdf",
    "package://my_robot/urdf/robot.srdf",
    locator,
)
robot = Robot.from_files("/path/to/robot.urdf", "/path/to/robot.srdf")
```

## Forward kinematics

```python
import numpy as np

joints = np.zeros(6)  # rad
tcp_pose = robot.fk("manipulator", joints, "tool0")
print(tcp_pose.translation)  # m, tool0 in base_link
print(tcp_pose.quaternion)   # scalar-last: x, y, z, w
```

`"manipulator"` is the SRDF group; `"tool0"` is the link whose pose you want. Every pose is
relative to a frame: [Frames and poses](../concepts/frames.md) names them.

## Inverse kinematics

```python
from tesseract_robotics.planning import IKNoSolutionError, Pose

# Tool pointing down at (0.8, -0.2, 0.8) m; quaternion [qx, qy, qz, qw]
target = Pose.from_xyz_quat(0.8, -0.2, 0.8, 0.0, -1.0, 0.0, 0.0)
try:
    solution = robot.ik("manipulator", target, "tool0", seed=joints)
    print(solution)  # rad
except IKNoSolutionError:
    print("unreachable")
```

The IRB 2400 uses the OPW solver, which is closed form: `all_solutions=True` returns every
branch (elbow up and down, wrist flipped) instead of the first.

## Check for collisions

```python
from tesseract_robotics.collision import ContactRequest, ContactResultMap, ContactTestType

robot.set_joints({"joint_1": 0.5, "joint_2": -0.3})  # rad

manager = robot.env.getDiscreteContactManager()
manager.setActiveCollisionObjects(robot.env.getActiveLinkIds())
manager.setCollisionObjectsTransform(robot.env.getState().link_transforms)

contacts = ContactResultMap()
manager.contactTest(contacts, ContactRequest(ContactTestType.ALL))
print("collision-free:", contacts.size() == 0)
```

The manager is a copy of the environment's collision world; its transforms are set from the
current state before the test. [Collision checking](../user-guide/collision.md) covers margins,
distances and swept motions.

## Plan

A `MotionProgram` lists targets; a planner turns it into a trajectory. Building a
`TaskComposer` loads the planning plugins (0.3 to 1.0 s on an Apple M1 Max), so build one and pass it
to every call.

```python
from tesseract_robotics.planning import (
    CartesianTarget, JointTarget, MotionProgram, TaskComposer, plan_freespace,
)

composer = TaskComposer.from_config()

program = (
    MotionProgram("manipulator", tcp_frame="tool0", working_frame="base_link")
    .move_to(JointTarget(np.zeros(6)))
    .move_to(CartesianTarget(target))
)

result = plan_freespace(robot, program, composer=composer)
assert result.successful, result.message
for point in result.trajectory:
    print(f"{point.time:5.2f} s  {np.round(point.positions, 3)}")
```

The same program runs under other planners. `plan_ompl` samples a collision-free path with OMPL,
then smooths it with TrajOpt; `plan_cartesian` searches the IK solutions of every Cartesian
waypoint with Descartes:

```python
from tesseract_robotics.planning import plan_cartesian, plan_ompl

for plan in (plan_ompl, plan_cartesian):
    result = plan(robot, program, composer=composer)
    assert result.successful, result.message
    print(plan.__name__, len(result.trajectory), "states")
```

Any pipeline the composer can load runs by name:

```python
result = composer.plan(robot, program, pipeline="TrajOptPipeline")
assert result.successful, result.message
```

[Task composer](../user-guide/task-composer.md#available-pipelines) lists the pipelines.

## Watch it

<!-- snippet: skip (starts a viser server and blocks until the reader presses Enter) -->
```python
from tesseract_robotics.viewer import TesseractViewer

with TesseractViewer() as viewer:
    viewer.update_environment(robot.env)
    viewer.update_trajectory(result.raw_results)
    input(f"Open {viewer.url}; Enter quits")
```

The viewer serves the scene to a browser tab and plays the trajectory with its time stamps;
[Viewer](../user-guide/viewer.md) adds TCP paths, markers and recording.

## Next steps

- [Concepts](../concepts/index.md): the environment, programs, pipelines and profiles.
- [Motion planning](../user-guide/planning.md): choosing a planner, costs and constraints,
  failures.
- [Examples](../examples/index.md): pick and place, raster, online replanning, all playable in the
  browser.
