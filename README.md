# Tesseract Python (nanobind)

[![PyPI](https://img.shields.io/pypi/v/tesseract-robotics-nanobind.svg)](https://pypi.org/project/tesseract-robotics-nanobind/)
[![Python](https://img.shields.io/badge/python-3.9%20|%203.10%20|%203.11%20|%203.12-blue.svg)](https://github.com/tesseract-robotics/tesseract_nanobind)
[![Build Status](https://github.com/tesseract-robotics/tesseract_nanobind/actions/workflows/wheels.yml/badge.svg)](https://github.com/tesseract-robotics/tesseract_nanobind/actions)
[![Documentation](https://img.shields.io/badge/docs-online-brightgreen.svg)](https://tesseract-robotics.github.io/tesseract_nanobind/)
[![license - Apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)

> **Note:** This is a friendly fork of [tesseract_python](https://github.com/tesseract-robotics/tesseract_python) that replaces SWIG bindings with modern [nanobind](https://github.com/wjakob/nanobind) bindings.

Python bindings for [Tesseract](https://github.com/tesseract-robotics/tesseract) robotics motion planning using [nanobind](https://github.com/wjakob/nanobind).

## Features

- Scene loading and management (URDF, SRDF, meshes)
- Collision checking (Bullet, FCL)
- Kinematics (KDL, OPW, UR)
- Motion planning (OMPL, Descartes, TrajOpt)
- Time parameterization (TOTG, ISP, Ruckig)
- Task composition and pipelines
- Pythonic high-level API

## Installation

```bash
pip install tesseract-robotics-nanobind
```

**Platform support:** Linux x86_64. macOS arm64 coming soon.

## Quick Start

```python
from tesseract_robotics.planning import (
    Robot, MotionProgram, JointTarget, CartesianTarget,
    Pose, box, create_obstacle, TaskComposer,
)

# Load robot
robot = Robot.from_urdf(
    "package://tesseract_support/urdf/abb_irb2400.urdf",
    "package://tesseract_support/urdf/abb_irb2400.srdf"
)

# Add obstacle
create_obstacle(robot, "box", box(0.5, 0.5, 0.5), Pose.from_xyz(0.5, 0, 0.3))

# Build motion program
program = (MotionProgram("manipulator", tcp_frame="tool0")
    .set_joint_names(robot.get_joint_names("manipulator"))
    .move_to(JointTarget([0, 0, 0, 0, 0, 0]))
    .move_to(CartesianTarget(Pose.from_xyz(0.5, 0.3, 0.8)))
)

# Plan
composer = TaskComposer.from_config()
result = composer.plan(robot, program)

if result.successful:
    for pt in result.trajectory:
        print(pt.positions)
```

## Low-Level API

For direct C++ API access:

```python
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_common import GeneralResourceLocator

env = Environment()
locator = GeneralResourceLocator()
env.init("/path/to/robot.urdf", "/path/to/robot.srdf", locator)

print(f"Joints: {env.getJointNames()}")
print(f"Links: {env.getLinkNames()}")
```

## Examples

See the `examples/` directory for:
- `basic_cartesian_example.py` - Simple Cartesian planning
- `freespace_ompl_example.py` - OMPL freespace planning
- `pick_and_place_example.py` - Pick and place with TrajOpt
- `puzzle_piece_example.py` - Cartesian path following
- And more...

## Versioning

Version follows `0.A.B.C` where `A.B` tracks the upstream [Tesseract](https://github.com/tesseract-robotics/tesseract) release and `C` is the nanobind patch number. For example, `0.34.1.0` wraps Tesseract `0.34.1`.

## Development

This project uses [pixi](https://pixi.sh) exclusively for dependency management and task running — no pip, conda, or venv needed. Pixi manages both the C++ toolchain (cmake, eigen, boost, bullet, ompl, ...) and Python deps in a single lockfile.

### Prerequisites

Install pixi ([docs](https://pixi.sh/latest/#installation)):

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

### First-time setup

```bash
git clone --recurse-submodules https://github.com/tesseract-robotics/tesseract_nanobind.git
cd tesseract_nanobind
pixi run build    # builds C++ libs + installs bindings (editable)
```

`pixi run build` chains two steps: `build-cpp` (compiles tesseract C++ via colcon) → `install` (editable pip install of the Python package). First build takes ~15 min; subsequent rebuilds are incremental.

### Available tasks

| Task | Description |
|------|-------------|
| `pixi run build` | Build C++ libs + install bindings |
| `pixi run build-cpp` | Build only the C++ libs |
| `pixi run install` | Editable install (assumes C++ already built) |
| `pixi run test` | Run pytest with xdist parallelism |
| `pixi run lint` | Lint with ruff |
| `pixi run fmt` | Format with ruff |
| `pixi run typecheck` | Type check with pyright |
| `pixi run docs` | Live docs server (localhost:8000) |
| `pixi run docs-build` | Build static docs to `site/` |

### Running tests

```bash
pixi run test                              # full suite, parallel
pixi shell && pytest tests -x --testmon    # incremental (only changed)
```

### Pre-commit hooks

```bash
pixi shell
pre-commit install
pre-commit install --hook-type pre-push
```

This runs ruff + auto-staging on commit, and pyright + pytest on push (configured in `.pre-commit-config.yaml`).

### Project layout

```
├── pyproject.toml            # package config + pixi workspace
├── CMakeLists.txt            # nanobind build
├── src/tesseract_robotics/   # Python package + C++ extension modules
├── tests/                    # pytest suite
├── examples/                 # usage examples
├── ws/                       # C++ workspace (colcon src/ + install/)
├── scripts/                  # build scripts (build_tesseract_cpp.sh, build_wheel.sh)
└── docs/                     # mkdocs-material documentation
```

### Building portable wheels

Editable installs bake absolute paths — not portable. For distributable wheels:

```bash
./scripts/build_wheel.sh              # builds + delocates (bundles dylibs)
pip install wheelhouse/tesseract*.whl
```

## Acknowledgments

This project builds upon the excellent work of [John Wason](https://github.com/johnwason) and the [Tesseract Robotics](https://github.com/tesseract-robotics) team. The original [tesseract_python](https://github.com/tesseract-robotics/tesseract_python) SWIG bindings laid the foundation for this nanobind implementation.

Special thanks to:
- **John Wason** (Wason Technology, LLC) - Original tesseract_python author and Tesseract maintainer
- **Levi Armstrong** - Tesseract core developer
- **Jelle Feringa** ([Terrestrial](http://terrestrial.construction)) - nanobind port developer
- The ROS-Industrial consortium for supporting Tesseract development

## License

Apache 2.0
