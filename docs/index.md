# tesseract_robotics

--8<-- "docs/demos/players/hero.html"

Python bindings for [Tesseract](https://github.com/tesseract-robotics/tesseract), the
ROS-Industrial motion planning framework. Load a robot, check collisions, solve kinematics and
plan collision-free trajectories from Python, without ROS.
{ .tr-lede }

`tesseract_robotics` binds the Tesseract C++ libraries module by module: the environment and
scene graph, kinematics (KDL, OPW, UR), collision checking (Bullet, FCL), the OMPL, TrajOpt,
TrajOpt-Ifopt and Descartes planners, time parameterization and the task composer that chains
them into pipelines. On top sits a short Python layer: a planning API, a browser viewer that
plays trajectories, and emitters that write controller programs (ABB RAPID, KUKA KRL, Fanuc TP,
Yaskawa INFORM, URScript). One wheel per platform covers Python 3.12 and later.

## Plan and watch a trajectory

```bash
pip install --pre "tesseract-robotics-nanobind[support]"
```

<!-- snippet: skip (the README's hero block, which runs; tests/docs/test_readme_structure.py keeps the two identical) -->
```python
from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.common import GeneralResourceLocator
from tesseract_robotics.planning import (
    CartesianTarget, JointTarget, MotionProgram, Pose, Robot, plan_freespace,
)
from tesseract_robotics.viewer import TesseractViewer

locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
robot = Robot.from_tesseract_support("abb_irb2400", locator)

# All-zero joints (rad) to a tool-down pose of tool0 in base_link: metres, [qx, qy, qz, qw]
program = (
    MotionProgram("manipulator", tcp_frame="tool0", working_frame="base_link")
    .move_to(JointTarget([0.0] * 6))
    .move_to(CartesianTarget(Pose.from_xyz_quat(0.8, -0.2, 0.8, 0.0, -1.0, 0.0, 0.0)))
)
result = plan_freespace(robot, program)
assert result.successful, result.message

with TesseractViewer() as viewer:
    viewer.update_environment(robot.env)
    viewer.update_trajectory(result.raw_results)
    input(f"{len(result.trajectory)} states, {result.trajectory[-1].time:.2f} s: open {viewer.url}, Enter quits")
```

`plan_freespace` runs the `TrajOptPipeline`: TrajOpt, a discrete collision check of every
state, and time parameterization. `result.trajectory` holds joint positions (rad), velocities,
accelerations and time stamps (s); the viewer plays it in the browser.

## Next steps

<div class="grid cards" markdown>

-   **Install**

    ---

    Wheels for Linux, macOS and Windows, or a source build with pixi.

    [:octicons-arrow-right-24: Installation](getting-started/installation.md)

-   **Quickstart**

    ---

    Kinematics, a collision check and three planners on one robot, in ten minutes.

    [:octicons-arrow-right-24: Quickstart](getting-started/quickstart.md)

-   **Concepts**

    ---

    The environment, programs, pipelines and profiles, and the frames a target names.

    [:octicons-arrow-right-24: Concepts](concepts/index.md)

</div>

## What is in the package

| Task | Guide | Modules |
| --- | --- | --- |
| Load and edit a robot cell | [Environment and scene graph](user-guide/environment.md) | `environment`, `scene_graph`, `urdf`, `srdf` |
| Forward and inverse kinematics, Jacobians | [Kinematics](user-guide/kinematics.md) | `kinematics`, `state_solver` |
| Distances and contacts, one state or a swept motion | [Collision checking](user-guide/collision.md) | `collision`, `geometry` |
| Plan freespace, Cartesian and raster motions | [Motion planning](user-guide/planning.md) | `planning`, `motion_planners_*` |
| Chain planners into pipelines | [Task composer](user-guide/task-composer.md) | `task_composer`, `task_composer_planning` |
| Replan in a loop, add constraints in Python | [Low-level SQP](user-guide/low-level-sqp.md) | `trajopt_ifopt`, `trajopt_sqp` |
| Watch and record trajectories | [Viewer](user-guide/viewer.md) | `viewer` |
| Write controller programs | [Program emitters](user-guide/emitters.md) | `emitters` |
| Save environments and programs | [Serialization](user-guide/serialization.md) | `serialization` |

Every recorded example plays in the browser on the [examples](examples/index.md) pages.

## Versions

A release `0.A.B.C` wraps the Tesseract `A.B` series; `C` counts the binding releases. The 0.36
series builds tesseract, trajopt and tesseract_planning from upstream `master` plus the
upstream pull requests it carries; `tesseract_robotics.build_identity.UPSTREAM_SOURCES` names
the exact commits. Changes are in the [changelog](CHANGELOG.md) and the
[breaking API changes](breaking-changes.md).
