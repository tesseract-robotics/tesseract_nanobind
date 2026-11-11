# Tesseract Python (nanobind)

[![A rail-mounted robot places a car seat through a car's door opening. Click to play the demo in your browser.](https://tesseract-robotics.github.io/tesseract_nanobind/demos/posters/car_seat_example.png)](https://tesseract-robotics.github.io/tesseract_nanobind/#play)

[![PyPI](https://img.shields.io/pypi/v/tesseract-robotics-nanobind.svg)](https://pypi.org/project/tesseract-robotics-nanobind/)
[![Python](https://img.shields.io/badge/python-3.12%20|%203.13%20|%203.14%20|%203.15-blue.svg)](https://github.com/tesseract-robotics/tesseract_nanobind)
[![Wheels](https://github.com/tesseract-robotics/tesseract_nanobind/actions/workflows/wheels.yml/badge.svg)](https://github.com/tesseract-robotics/tesseract_nanobind/actions/workflows/wheels.yml)
[![Documentation](https://img.shields.io/badge/docs-online-brightgreen.svg)](https://tesseract-robotics.github.io/tesseract_nanobind/)
[![license - Apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)

Python bindings for [Tesseract](https://github.com/tesseract-robotics/tesseract), the
ROS-Industrial motion planning framework. Load a robot, check collisions, solve kinematics and
plan collision-free trajectories from Python, without ROS.

The bindings cover the Tesseract C++ libraries module by module: 
- `environment` and `scene graph`,
- `kinematics` (KDL, OPW, UR), `collision checking` (Bullet, FCL)
- `OMPL` motion-planning, `TrajOpt-Ifopt`, `Descartes` and simple planners.
- `time parameterization` (TOTG, ISP, constant TCP speed, Ruckig) and the `task composer`

A Python layer adds a `planning API`, `constraints written in Python` for the
online SQP solver (`trajopt_ifopt.python_constraints`)
A browser viewer and controller program emitters (ABB `RAPID`, KUKA `KRL`, Fanuc `TP`, Yaskawa `INFORM`, Universal Robots `URScript`). 

Over 4,300 tests run against every wheel CI builds.

## Installation

```bash
pip install --pre "tesseract-robotics-nanobind[support]"
```

`--pre` selects the current release candidate, `0.36.0.0rc1`; drop it once 0.36.0.0 is final. The `[support]` extra adds tesseract's example robots and meshes (`tesseract-robotics-nanobind-support`), which the examples and the Quick Start load. Leave it out if you only use your own robots.

One `cp312-abi3` wheel per platform covers Python 3.12 to 3.15:

- Linux: x86_64 and aarch64, `manylinux_2_39` (Ubuntu 24.04+, Debian 13+)
- macOS: arm64 and x86_64, macOS 15.0 or later
- Windows: x86_64

## Examples

Recorded examples play in the browser, inline on the documentation pages. Orbit, scrub and change speed without installing anything.

- [Planning](https://tesseract-robotics.github.io/tesseract_nanobind/examples/planning/): car seat, pick and place, raster, Cartesian, freespace and a 9-DOF toolpath
- [Online planning](https://tesseract-robotics.github.io/tesseract_nanobind/examples/online-planning/): replanning around a moving obstacle
- [Python constraints](https://tesseract-robotics.github.io/tesseract_nanobind/examples/python-constraints/): constraints written in Python, solved in a live loop
- [Basics](https://tesseract-robotics.github.io/tesseract_nanobind/examples/basic/), [collision](https://tesseract-robotics.github.io/tesseract_nanobind/examples/collision/) and [geometry](https://tesseract-robotics.github.io/tesseract_nanobind/examples/geometry/): scene graphs, point clouds, signed distance fields and shapes

The source is in [`src/tesseract_robotics/examples`](https://github.com/tesseract-robotics/tesseract_nanobind/tree/main/src/tesseract_robotics/examples). Each example installs as a console script, such as `tesseract_car_seat_example`.

## Quick Start

Plan a motion for the ABB IRB 2400 and play it in the browser:

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

`plan_freespace` runs the `TrajOptPipeline`: TrajOpt, a discrete collision check of every state, and time parameterization. The [quickstart](https://tesseract-robotics.github.io/tesseract_nanobind/getting-started/quickstart/) goes on to kinematics, collision checks and the other planners.

## Documentation

The [documentation](https://tesseract-robotics.github.io/tesseract_nanobind/) has the [concepts](https://tesseract-robotics.github.io/tesseract_nanobind/concepts/), [user guide](https://tesseract-robotics.github.io/tesseract_nanobind/user-guide/planning/), [API reference](https://tesseract-robotics.github.io/tesseract_nanobind/api/) and [breaking changes](https://tesseract-robotics.github.io/tesseract_nanobind/breaking-changes/); its sources are in [`docs/`](docs/).

## Versioning

Version follows `0.A.B.C`: `A.B` is the [Tesseract](https://github.com/tesseract-robotics/tesseract) series the bindings wrap and `C` is the nanobind patch number; a pre-release appends `rcN`. Releases up to `0.35.0.x` wrap a tagged Tesseract release. The `0.36` series builds tesseract, trajopt and tesseract_planning from upstream `master` plus the open upstream pull requests we carry (`tesseract_robotics.build_identity.UPSTREAM_SOURCES` lists the exact commits), until the 0.36 tags exist upstream.

See [CHANGELOG.md](CHANGELOG.md) for release notes.

## Development

Setup, tasks, tests and wheel builds are in the [developer guide](https://tesseract-robotics.github.io/tesseract_nanobind/developer/).

## Acknowledgments

This project builds upon the excellent work of [John Wason](https://github.com/johnwason) and the [Tesseract Robotics](https://github.com/tesseract-robotics) team. The original [tesseract_python](https://github.com/tesseract-robotics/tesseract_python) SWIG bindings laid the foundation for this nanobind implementation.

## License

[Apache 2.0](LICENSE). Attribution notices are in [NOTICE](NOTICE).
