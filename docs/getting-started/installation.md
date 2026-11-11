# Installation

Users install a wheel from PyPI; it bundles the C++ libraries and needs no compiler, no ROS
and no conda. Contributors build from source with pixi.

## Users: install from PyPI

```bash
pip install --pre "tesseract-robotics-nanobind[support]"
```

`--pre` installs the current release candidate, `0.36.0.0rc1`; drop it once 0.36.0.0 is final.
The wheel bundles all C++ dependencies (~30 MB). No compiler needed. The `[support]` extra
installs tesseract's example robots and meshes as a separate wheel,
`tesseract-robotics-nanobind-support` (~40 MB); the examples and these pages load robots from it.
Leave it out if you only load your own robots.

```python
from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.common import GeneralResourceLocator
from tesseract_robotics.planning import Robot

locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
robot = Robot.from_tesseract_support("abb_irb2400", locator)
print(f"Loaded: {len(robot.get_joint_names('manipulator'))} joints")
```

**Supported platforms:**

| OS | Architectures | Minimum |
| -- | ------------- | ------- |
| Linux | x86_64, aarch64 | `manylinux_2_39`: Ubuntu 24.04, Debian 13 (Ubuntu 22.04 stays on 0.35.0.8) |
| macOS | arm64, x86_64 | macOS 15.0 (`macosx_15_0`, imposed by the bundled `libomp`) |
| Windows | x86_64 | |

**Python versions:** 3.12–3.15, from one `cp312-abi3` stable-ABI wheel per platform.
That includes Rhino 9, which embeds CPython 3.13. Rhino 8 (CPython 3.9) and Python
3.10–3.11 stay on 0.35.0.8, the last release with wheels for them.

On 3.15, `zstandard`, which `viser` requires, compiles from source until it ships 3.15 wheels, so a plain `uv pip install` needs a C compiler there.

---

## Developers: build from source

### Prerequisites

Install [pixi](https://pixi.sh):

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

You also need the platform's C++17 compiler (Apple clang, gcc/g++ or MSVC; pixi does not
provide it), git, and disk for `.native/` (5.2 GB on macOS arm64 after a build).

### Build

```bash
git clone https://github.com/tesseract-robotics/tesseract_nanobind.git
cd tesseract_nanobind
pixi run build   # native-build, then the bindings (editable) and the support data
```

`pixi run build` first runs `pixi run native-build`, which compiles the C++ stack (tesseract,
trajopt, tesseract_planning, descartes_light, opw_kinematics, ruckig, boost_plugin_loader on
vcpkg's third-party libraries) into `.native/<triplet>/` with `scripts/native_stack.py`, the
script the wheel workflow runs. The first build compiles the whole stack and is slow; each of
its stages records a content-addressed identity, so later runs skip a stage whose inputs did not
change. The bindings then compile against `.native/<triplet>/ws/install`. See
[The native stack](../developer/native-stack.md) for the stages and how to change them.

Run an example:

```bash
pixi run tesseract_freespace_ompl_example
```

---

## What pixi does

Pixi resolves the toolchain and the Python environment from one lockfile. It does not provide
the C++ stack, which `scripts/native_stack.py` builds:

| Category | Examples |
| -------- | -------- |
| Build tools | cmake, ninja, make, pkg-config, ccache, colcon, vcs2l |
| Python packages | numpy, scipy, nanobind, scikit-build-core, pytest, loguru |
| Platform-specific | llvm-openmp (macOS) |

The `pixi.lock` file pins the exact versions of the toolchain and Python packages, so every
developer and CI machine builds the bindings with the same tools. The C++ stack is pinned
elsewhere: the commits in `dependencies.rosinstall`, and the vcpkg revision and package list in
`scripts/native_stack.toml`.

```bash
# See what's pinned
pixi list

# Update after someone changes pyproject.toml
pixi install
```

---

## C++ dependencies

`scripts/native_stack.py` builds three stages into `.native/<triplet>/`:

| Stage | Source | Output |
| ----- | ------ | ------ |
| `sources` | `vcs import` of `dependencies.rosinstall` (tesseract, trajopt, tesseract_planning, descartes_light, opw_kinematics, ruckig, boost_plugin_loader) | `ws/src` |
| `vcpkg` | the revision and packages in `scripts/native_stack.toml` (Eigen, Boost, OMPL, FCL, Bullet, PCL, OSQP, spdlog, ...) | `vcpkg/installed/<triplet>` |
| `colcon` | the sources against the vcpkg libraries | `ws/install` (`TESSERACT_CPP_PREFIX`) |

To run any command against that stack, use `python scripts/native_stack.py exec -- <command>`.
Both the pixi tasks and the wheel workflow go through the same script, so a development build and
a wheel compile the same sources with the same flags.

### Submodule (optional)

The `ws/src/tesseract_ros_workcell` submodule provides industrial workcell models:

```bash
git submodule update --init ws/src/tesseract_ros_workcell
```

It fetches about 50 MB of URDF/mesh assets for a multi-robot workcell with
positioner and rail, used by `src/tesseract_robotics/examples/twc_workcell_positioner_viewer.py`.
Name the path: a bare `git submodule update --init` also clones the `upstream/`
sources, which only the 0.36 inner loop needs.

---

## Troubleshooting

### Plugin loading errors

```text
Error: Failed to load plugin library
```

Run within pixi: `pixi run python your_script.py` or enter `pixi shell` first.

### OpenMP crashes (macOS)

Pixi installs `llvm-openmp` from conda-forge. If you have Homebrew's libomp installed, library conflicts can cause crashes. The pixi environment isolates this, but running outside `pixi shell` may pick up the wrong library.

The PyPI wheels carry their own `libomp.dylib`. In a plain venv (pip, uv) that is the only runtime, but inside a conda or pixi environment that has `llvm-openmp` it makes two: Descartes aborts with `OMP: Error #15`, and under `KMP_DUPLICATE_LIB_OK=TRUE` it survives `num_threads=1` but segfaults at 2. Install the wheel into a venv, or use the pixi environment's editable install. The mechanism is in [macOS Wheel Gotchas](../developer/macos-wheels.md).

### Scattered segfaults / "double free or corruption"

Random crashes across kinematics, planning, and trajopt code paths, while imports and simple
geometry work, are the signature of bindings compiled against a different C++ stack than the one
they load. Rebuild both against the same inputs: `pixi run build` recompiles any stage whose
inputs changed, and an editable install keeps pointing at `.native/<triplet>/ws/install`.
