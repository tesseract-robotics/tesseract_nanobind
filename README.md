# Tesseract Python (nanobind)

[![PyPI](https://img.shields.io/pypi/v/tesseract-robotics-nanobind.svg)](https://pypi.org/project/tesseract-robotics-nanobind/)
[![Python](https://img.shields.io/badge/python-3.12%20|%203.13%20|%203.14-blue.svg)](https://github.com/tesseract-robotics/tesseract_nanobind)
[![Linux](https://github.com/tesseract-robotics/tesseract_nanobind/actions/workflows/wheels-linux.yml/badge.svg)](https://github.com/tesseract-robotics/tesseract_nanobind/actions/workflows/wheels-linux.yml)
[![macOS](https://github.com/tesseract-robotics/tesseract_nanobind/actions/workflows/wheels-macos.yml/badge.svg)](https://github.com/tesseract-robotics/tesseract_nanobind/actions/workflows/wheels-macos.yml)
[![Windows](https://github.com/tesseract-robotics/tesseract_nanobind/actions/workflows/wheels-windows.yml/badge.svg)](https://github.com/tesseract-robotics/tesseract_nanobind/actions/workflows/wheels-windows.yml)
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

1900+ pytest tests in `tests/`, mirroring the module layout (`tesseract_collision`, `tesseract_kinematics`, `tesseract_task_composer`, `trajopt_sqp`, …) plus the high-level `planning` API, examples and viewer. CI installs each built wheel into a clean environment and runs the full suite on Linux, macOS and Windows before anything is published to PyPI. See [Running tests](#running-tests).

## Installation

```bash
pip install tesseract-robotics-nanobind
```

**Platform support:** 

- linux
   - x86_64 - 3.12-3.14
   - aarch64 - 3.12-3.14
- windows
    - x86_64 - 3.12-3.14
- macOS
    - arm64 - 3.12-3.14

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

## Examples

[examples](https://github.com/tesseract-robotics/tesseract_nanobind/tree/main/src/tesseract_robotics/examples) are located at `src/tesseract_robotics/examples`.

- `basic_cartesian_example.py` - Simple Cartesian planning
- `freespace_ompl_example.py` - OMPL freespace planning
- `pick_and_place_example.py` - Pick and place with TrajOpt
- `puzzle_piece_auxillary_axes_example.py` - 9-DOF Cartesian toolpath
- And more...

## Versioning

Version follows `0.A.B.C` where `A.B` tracks the upstream [Tesseract](https://github.com/tesseract-robotics/tesseract) release and `C` is the nanobind patch number. For example, `0.34.1.0` wraps Tesseract `0.34.1`.

See [CHANGELOG.md](CHANGELOG.md) for release notes.

## Development

This project uses [pixi](https://pixi.sh) exclusively for dependency management and task running — no pip, conda, or venv needed. Pixi manages the build toolchain (cmake, ninja, nanobind), the prebuilt tesseract C++ libraries (from the [`tesseract-robotics` conda channel](https://github.com/tesseract-robotics-packaging)), and Python deps in a single lockfile.

### Prerequisites

Install pixi ([docs](https://pixi.sh/latest/#installation)):

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

### First-time setup

```bash
git clone --recurse-submodules https://github.com/tesseract-robotics/tesseract_nanobind.git
cd tesseract_nanobind
pixi run build    # installs prebuilt C++ libs + builds/installs bindings (editable)
```

`pixi run build` resolves the pixi env (which pulls the prebuilt tesseract C++ libs from the `tesseract-robotics` conda channel) and runs `install` (editable pip install of the Python package). Only the nanobind extensions compile locally — no more from-source C++ build.

### Available tasks

| Task | Description |
|------|-------------|
| `pixi run build` | Install C++ libs (conda) + build/install bindings |
| `pixi run install` | Editable install of the bindings |
| `pixi run test` | Run pytest with xdist parallelism (excludes benchmarks) |
| `pixi run test-benchmark` | Run benchmark tests (serial) |
| `pixi run lint` | Lint with ruff |
| `pixi run fmt` | Format with ruff |
| `pixi run typecheck` | Type check with pyright |
| `pixi run stubs` | Regenerate the committed `.pyi` stubs |
| `pixi run stubs-check` | Fail if committed stubs differ from the built modules |
| `pixi run check-changelog` | Check CHANGELOG.md release structure |
| `pixi run docs` | Live docs server (localhost:8000) |
| `pixi run docs-build` | Build static docs to `site/` |
| `pixi run docs-dotgraphs` | Regenerate pipeline DOT graph SVGs (requires graphviz) |
| `pixi run build-wheel` | Build a portable wheel (see below) |

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

This runs ruff + auto-staging on commit, and `pixi run typecheck` on push (configured in `.pre-commit-config.yaml`).

### Project layout

```
├── pyproject.toml            # package config + pixi workspace
├── CMakeLists.txt            # nanobind build
├── src/tesseract_robotics/   # Python package + C++ extension modules + examples/
├── tests/                    # pytest suite
├── scripts/                  # build scripts (build_{linux,macos}_wheel.sh)
└── docs/                     # mkdocs-material documentation
```

### Building portable wheels

Editable installs bake absolute paths — not portable. For distributable wheels:

```bash
pixi run build-wheel                  # bundles native deps (Linux: patchelf, macOS: delocate)
pip install wheelhouse/tesseract*.whl
```

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
