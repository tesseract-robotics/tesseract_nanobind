# Installation

## Users: Install from PyPI

```bash
pip install tesseract-robotics-nanobind
```

The wheel bundles all C++ dependencies (~50MB). No compiler needed.

```python
import tesseract_robotics
from tesseract_robotics.planning import Robot

robot = Robot.from_tesseract_support("abb_irb2400")
print(f"Loaded: {len(robot.get_joint_names('manipulator'))} joints")
```

**Supported platforms:** macOS (ARM), Linux (x86_64/aarch64), Windows (x64).

**Python versions:** 3.12–3.14, from one `cp312-abi3` stable-ABI wheel per platform.
That includes Rhino 9, which embeds CPython 3.13. Rhino 8 (CPython 3.9) and Python
3.10–3.11 stay on 0.35.0.8, the last release with wheels for them.

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
pixi run build   # resolves conda deps + compiles the nanobind extensions
```

The tesseract C++ libraries come **prebuilt** from the
[`tesseract-robotics` conda channel](https://anaconda.org/tesseract-robotics) —
only the nanobind extension modules compile locally (~10-15 min first time,
incremental after).

That's it. Run an example:

```bash
pixi run tesseract_freespace_ompl_example
```

---

## What Pixi Does

Pixi replaces the usual conda/pip/brew chaos with a single lockfile. For this project, it manages:

| Category | Examples |
| -------- | -------- |
| Prebuilt tesseract C++ stack | tesseract-robotics, tesseract-robotics-planning (conda channel) |
| Transitive C++ libraries | Eigen, PCL, OMPL, FCL, Bullet, OSQP, console_bridge |
| Python packages | numpy, scipy, nanobind, pytest, loguru |
| Build tools | cmake, ninja, pkg-config |
| Platform-specific | llvm-openmp (macOS), patchelf (Linux) |

### Why This Matters for C++ Bindings

Pure Python packages are forgiving - pip resolves versions at install time and everything just works. C++ bindings are not forgiving.

The nanobind-generated code must match the *exact* headers it was compiled against. If you build against Eigen 3.4.0 but run against 3.4.1, subtle struct layout differences can cause silent memory corruption or crashes. These bugs are notoriously hard to diagnose.

The `pixi.lock` file pins **exact versions** of all ~200 transitive dependencies - down to libc and OpenSSL. Every developer and CI machine builds against identical binaries. This eliminates an entire class of "works on my machine" bugs.

!!! warning "The lockfile is a loaded weapon"
    The Eigen 5 migration on conda-forge demonstrated the flip side: a lockfile
    solved mid-migration froze an `orocos-kdl` built against Eigen 3.4 next to
    tesseract libs built against Eigen 5 — a legal solve (the old KDL build
    predates the `eigen-abi` metadata) that heap-corrupts every KDL code path at
    runtime. CI runs `pixi install --locked` so lock↔manifest drift fails loudly,
    and the build jobs run a kinematics ABI canary against the freshly built
    wheel. If you re-lock, re-run the test suite.

```bash
# See what's pinned
pixi list

# Update after someone changes pyproject.toml
pixi install
```

---

## C++ Dependencies

The tesseract C++ stack arrives as two conda packages (pinned `==0.35.0`), built by
the [tesseract-robotics channel](https://anaconda.org/tesseract-robotics):

| Package | What it provides |
| ------- | ---------------- |
| tesseract-robotics | Environment, collision detection, kinematics, scene graph, srdf/urdf |
| tesseract-robotics-planning | Task composer, motion planning pipelines (OMPL, TrajOpt, Descartes, Ruckig) |

Everything else (trajopt, descartes-light, opw-kinematics, boost-plugin-loader,
eigen, boost, ompl, bullet, fcl, …) arrives transitively, matched to how those
feedstocks were built. Don't pin the transitive C++ deps in `pyproject.toml` —
let the tesseract packages drive the resolution so the ABI stays coherent.

### Submodule (Optional)

The `ws/src/tesseract_ros_workcell` submodule provides industrial workcell models:

```bash
git submodule update --init ws/src/tesseract_ros_workcell
```

It fetches about 50 MB of URDF/mesh assets for a multi-robot workcell with
positioner and rail, used by `src/tesseract_robotics/examples/twc_workcell_positioner_viewer.py`.
Name the path: a bare `git submodule update --init` also clones the `upstream/`
sources and `packaging/` feedstocks, which only the 0.36 build loops need.

---

## Troubleshooting

### Plugin Loading Errors

```text
Error: Failed to load plugin library
```

Run within pixi: `pixi run python your_script.py` or enter `pixi shell` first.

### OpenMP Crashes (macOS)

Pixi installs `llvm-openmp` from conda-forge. If you have Homebrew's libomp installed, library conflicts can cause crashes. The pixi environment isolates this, but running outside `pixi shell` may pick up the wrong library.

A second duplication survives *inside* a conda or pixi environment, because the published macOS wheel carries its own delocated `libomp.dylib`. `numpy` loads and initialises the environment's copy at import; the wheel's copy is mapped when `tesseract_robotics` is imported and stays idle until something opens an OpenMP parallel region. Descartes' ladder solver is the first thing that does, so it aborts with `OMP: Error #15` where the rest of a suite passes — and under `KMP_DUPLICATE_LIB_OK=TRUE` it survives `num_threads=1` but segfaults at 2. The mechanism, the measurement and the packaging fix are in [macOS Wheel Gotchas](../developer/macos-wheels.md).

### Scattered Segfaults / "double free or corruption"

Random crashes across kinematics, planning, and trajopt code paths — while
imports and simple geometry work — are the signature of an ABI-incoherent
dependency set (see the lockfile warning above). Re-lock with
`pixi lock`, reinstall, and re-run the suite. CI's `locked: true` +
kinematics canary exist to stop this class of wheel from shipping.
