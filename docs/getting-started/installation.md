# Installation

## Users: Install from PyPI

```bash
pip install tesseract-robotics
```

The wheel bundles all C++ dependencies (~50MB). No compiler needed.

```python
import tesseract_robotics
from tesseract_robotics.planning import Robot

robot = Robot.from_tesseract_support("abb_irb2400")
print(f"Loaded: {len(robot.get_joint_names('manipulator'))} joints")
```

**Supported platforms:** macOS (ARM/Intel), Linux. Windows not yet supported.

---

## Developers: Build from Source

### Prerequisites

Install [pixi](https://pixi.sh):

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

You'll also need a C++17 compiler and ~4GB disk space.

### Build (3 commands)

```bash
git clone https://github.com/tesseract-robotics/tesseract_nanobind.git
cd tesseract_nanobind
pixi run build-cpp   # ~15-30 min first time
```

That's it. Run an example:

```bash
pixi run python examples/freespace_ompl_example.py
```

---

## What Pixi Does

Pixi replaces the usual conda/pip/brew chaos with a single lockfile. For this project, it manages:

| Category | Examples |
| -------- | -------- |
| C++ libraries | Eigen, PCL, OMPL, FCL, Bullet, OSQP, console_bridge |
| Python packages | numpy, scipy, nanobind, pytest, loguru |
| Build tools | cmake, ninja, colcon, vcstool |
| Platform-specific | llvm-openmp (macOS), compilers |

### Why This Matters for C++ Bindings

Pure Python packages are forgiving - pip resolves versions at install time and everything just works. C++ bindings are not forgiving.

The nanobind-generated code must match the *exact* headers it was compiled against. If you build against Eigen 3.4.0 but run against 3.4.1, subtle struct layout differences can cause silent memory corruption or crashes. These bugs are notoriously hard to diagnose.

The `pixi.lock` file pins **exact versions** of all ~200 transitive dependencies - down to libc and OpenSSL. Every developer and CI machine builds against identical binaries. This eliminates an entire class of "works on my machine" bugs.

```bash
# See what's pinned
pixi list

# Update after someone changes pyproject.toml
pixi install
```

---

## C++ Dependencies

The build fetches 9 repositories via `vcs import` from `ws/src/dependencies.rosinstall`:

| Repository | Version | What it provides |
| ---------- | ------- | ---------------- |
| tesseract | 0.33.1 | Environment, collision detection, kinematics, scene graph |
| tesseract_planning | 0.33.1 | Task composer, motion planning pipelines |
| trajopt | 0.33.0 | Sequential convex trajectory optimization |
| descartes_light | 0.4.9 | Cartesian planning via ladder graph search |
| opw_kinematics | 0.5.2 | Analytical IK for 6-DOF industrial arms (ABB, KUKA, Fanuc) |
| ifopt | 2.1.3 | Nonlinear optimization interface (OSQP/IPOPT backends) |
| ruckig | 0.9.2 | Time-optimal trajectory parametrization |
| boost_plugin_loader | 0.4.3 | Runtime plugin loading for kinematics/collision/planners |
| ros_industrial_cmake_boilerplate | 0.7.1 | CMake utilities shared across ROS-Industrial projects |

### Submodule (Optional)

One git submodule provides industrial workcell models:

```bash
git submodule update --init
```

This fetches `ws/src/tesseract_ros_workcell` (~50MB) - URDF/mesh assets for a complex multi-robot workcell with positioner and rail. Required for `examples/twc_workcell_positioner_viewer.py`.

---

## Troubleshooting

### Plugin Loading Errors

```text
Error: Failed to load plugin library
```

Run within pixi: `pixi run python your_script.py` or enter `pixi shell` first.

### OpenMP Crashes (macOS)

Pixi installs `llvm-openmp` from conda-forge. If you have Homebrew's libomp installed, library conflicts can cause crashes. The pixi environment isolates this, but running outside `pixi shell` may pick up the wrong library.

### RTTI Errors

If TaskComposer fails with "Input is not a Composite Instruction", the C++ libraries were built with hidden symbol visibility. Rebuild with:

```bash
pixi run build-cpp  # Already configured correctly
```

The build scripts set `-DCMAKE_CXX_VISIBILITY_PRESET=default` to ensure RTTI works across shared library boundaries.
