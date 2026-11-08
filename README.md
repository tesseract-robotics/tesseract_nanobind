# Tesseract Python (nanobind)

[![A rail-mounted robot places a car seat through a car's door opening. Click to play the demo in your browser.](https://tesseract-robotics.github.io/tesseract_nanobind/demos/posters/car_seat_example.png)](https://tesseract-robotics.github.io/tesseract_nanobind/#play)

[![PyPI](https://img.shields.io/pypi/v/tesseract-robotics-nanobind.svg)](https://pypi.org/project/tesseract-robotics-nanobind/)
[![Python](https://img.shields.io/badge/python-3.12%20|%203.13%20|%203.14%20|%203.15-blue.svg)](https://github.com/tesseract-robotics/tesseract_nanobind)
[![Wheels](https://github.com/tesseract-robotics/tesseract_nanobind/actions/workflows/wheels.yml/badge.svg)](https://github.com/tesseract-robotics/tesseract_nanobind/actions/workflows/wheels.yml)
[![Documentation](https://img.shields.io/badge/docs-online-brightgreen.svg)](https://tesseract-robotics.github.io/tesseract_nanobind/)
[![license - Apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)

> **Note:** This is a friendly fork of [tesseract_python](https://github.com/tesseract-robotics/tesseract_python) that replaces SWIG bindings with modern [nanobind](https://github.com/wjakob/nanobind) bindings.

Python bindings for [Tesseract](https://github.com/tesseract-robotics/tesseract) robotics motion planning using [nanobind](https://github.com/wjakob/nanobind).

## Documentation 

Maintained in `/docs` and published to https://tesseract-robotics.github.io/tesseract_nanobind/

## Features

- Scene loading and management (URDF, SRDF, meshes)
- Collision checking (Bullet, FCL)
- Kinematics (KDL, OPW, UR)
- Motion planning (OMPL, Descartes, TrajOpt)
- Time parameterization (TOTG, ISP, Ruckig)
- Task composition and pipelines
- Pythonic high-level API

for an excellent overview of `Tesseract's` architecture & features see [this presentation on Vimeo](https://vimeo.com/649656119)

## Test suite

1900+ pytest tests in `tests/`, mirroring the module layout (`tesseract_collision`, `tesseract_kinematics`, `tesseract_task_composer`, `trajopt_sqp`, …) plus the high-level `planning` API, examples and viewer. CI installs each built wheel into a clean environment and runs the full suite on Linux, macOS and Windows before anything is published to PyPI. See [Running tests](https://tesseract-robotics.github.io/tesseract_nanobind/developer/#running-tests).

## Installation

```bash
pip install tesseract-robotics-nanobind
```

**Platform support:** 

- linux
   - x86_64 - 3.12-3.15
   - aarch64 - 3.12-3.15
- windows
    - x86_64 - 3.12-3.15
- macOS
    - arm64 - 3.12-3.15

## Examples

Recorded examples play in the browser, inline on these pages. Orbit, scrub and change speed without installing anything.

- [Planning](https://tesseract-robotics.github.io/tesseract_nanobind/examples/planning/): car seat, pick and place, raster, Cartesian, freespace and a 9-DOF toolpath
- [Online planning](https://tesseract-robotics.github.io/tesseract_nanobind/examples/online-planning/): replanning around a moving obstacle
- [Basic](https://tesseract-robotics.github.io/tesseract_nanobind/examples/basic/), [collision](https://tesseract-robotics.github.io/tesseract_nanobind/examples/collision/) and [geometry](https://tesseract-robotics.github.io/tesseract_nanobind/examples/geometry/): scene graphs, point clouds, signed distance fields and shapes

The source is in [`src/tesseract_robotics/examples`](https://github.com/tesseract-robotics/tesseract_nanobind/tree/main/src/tesseract_robotics/examples). Each example installs as a console script, such as `tesseract_car_seat_example`.

## Quick Start

```python
from tesseract_robotics.planning import (
    Robot, MotionProgram, JointTarget, CartesianTarget, Pose, TaskComposer,
)

# Load a bundled robot (urdf + srdf from tesseract's support package)
robot = Robot.from_tesseract_support("abb_irb2400")

# Start at all-zero joints, goal is a tool0 pose (upstream freespace_example.cpp);
# quaternion is scalar-last [qx, qy, qz, qw]
start = [0.0] * 6
goal = Pose.from_xyz_quat(0.8, -0.2, 0.8, 0.0, -1.0, 0.0, 0.0)

# Build motion program
program = (MotionProgram("manipulator", tcp_frame="tool0", working_frame="base_link")
    .set_joint_names(robot.get_joint_names("manipulator"))
    .move_to(JointTarget(start))
    .move_to(CartesianTarget(goal))
)

# Plan
composer = TaskComposer.from_config()
result = composer.plan(robot, program)

assert result.successful, result.message
for pt in result.trajectory:
    print(pt.positions)
```

## Low-Level API

For direct C++ API access:

<!-- snippet: skip -->
```python
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_common import GeneralResourceLocator

env = Environment()
locator = GeneralResourceLocator()
env.init("/path/to/robot.urdf", "/path/to/robot.srdf", locator)

print(f"Joints: {env.getJointNames()}")
print(f"Links: {env.getLinkNames()}")
```

## Versioning

Version follows `0.A.B.C` where `A.B` tracks the upstream [Tesseract](https://github.com/tesseract-robotics/tesseract) release and `C` is the nanobind patch number. For example, `0.34.1.0` wraps Tesseract `0.34.1`.

See [CHANGELOG.md](CHANGELOG.md) for release notes.

## Development

Setup, tasks, tests and wheel builds are in the [developer guide](https://tesseract-robotics.github.io/tesseract_nanobind/developer/).

## Acknowledgments

This project builds upon the excellent work of [John Wason](https://github.com/johnwason) and the [Tesseract Robotics](https://github.com/tesseract-robotics) team. The original [tesseract_python](https://github.com/tesseract-robotics/tesseract_python) SWIG bindings laid the foundation for this nanobind implementation.

Special thanks to:
- **John Wason** ([Wason Technology, LLC](https://www.wasontech.com/)) - Original tesseract_python author and Tesseract maintainer
- **Levi Armstrong** - Tesseract core developer
- **Jelle Feringa** ([Terrestrial](http://terrestrial.construction)) - nanobind port developer
- **Joel Kang** ([Tekton Dynamics](https://tekton-dynamics.com))
- The ROS-Industrial consortium for supporting Tesseract development

## License

Apache 2.0
