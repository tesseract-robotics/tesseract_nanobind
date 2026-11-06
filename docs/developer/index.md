# Developer Guide

## Pixi Workspace

This project uses [pixi](https://pixi.sh) exclusively for package management. Pixi manages all C++ libraries, Python packages, build tools, and platform-specific dependencies through a single lockfile (`pixi.lock`). No pip, conda, poetry, or venv.

The tesseract C++ libraries are not built here: they are prebuilt conda packages from the `tesseract-robotics` channel. [How the binaries are built](binaries.md) traces every binary from the upstream source to the PyPI wheel.

### First-Time Setup

Install pixi ([docs](https://pixi.sh/latest/#installation)), then clone and build:

```bash
curl -fsSL https://pixi.sh/install.sh | bash

git clone --recurse-submodules https://github.com/tesseract-robotics/tesseract_nanobind.git
cd tesseract_nanobind
pixi run build
```

`pixi run build` resolves the environment, which pulls the prebuilt C++ libraries, and compiles only the nanobind extensions.

### Available Tasks

```bash
pixi task list
```

| Task | Description | What it does |
|------|-------------|--------------|
| `build` | Build bindings | alias for `install` |
| `install` | Install bindings | editable `pip install -e . --no-build-isolation` against the conda tesseract libs |
| `test` | Run tests | pytest with xdist parallel, excluding benchmarks (depends on `install`) |
| `test-benchmark` | Run benchmarks | benchmark tests, serial |
| `build-wheel` | Portable wheel | `scripts/build_linux_wheel.sh` / `build_macos_wheel.sh` (see [binaries](binaries.md)) |
| `typecheck` | Type check | pyright on `src/tesseract_robotics/` |
| `lint` | Lint | ruff check |
| `fmt` | Format | ruff format |
| `stubs` | Regenerate stubs | the committed `.pyi` stubs; see [Stub Generation](#stub-generation) |
| `stubs-check` | Check stubs | fails if committed stubs differ from the built modules |
| `docs` | Live docs | mkdocs serve with auto-reload |
| `docs-build` | Build docs | Static site to `site/` |
| `docs-dotgraphs` | Pipeline graphs | regenerates the pipeline DOT graph SVGs (requires graphviz) |
| `check-changelog` | Check changelog | `CHANGELOG.md` release structure (the Docs workflow runs it too) |
| `audit-gil` (`-e audit`) | GIL audit | Every GIL release in the bindings against `scripts/nogil_entry_points.yaml`; see [Threading and the GIL](threading.md) |
| `audit-test` (`-e audit`) | Audit contract tests | `tests/audit`, which needs libclang |
| `gil-hotspots` | GIL hotspots | Records benchmarks and tests with `py-spy --gil`, ranks the bindings that hold the GIL ([Finding candidates](threading.md#finding-candidates)) |

### Running Tests

```bash
pixi run test                              # full suite, parallel
pixi shell && pytest tests -x --testmon    # incremental (only changed)
```

### Daily Workflow

```bash
# First time setup (installs the conda C++ libs, compiles the bindings)
pixi run build

# Run tests
pixi run test

# Run a single test file
pixi run python -m pytest tests/trajopt_sqp/test_trajopt_sqp_bindings.py -v

# Run only tests affected by recent changes
pixi run python -m pytest --testmon

# Run an example
pixi run tesseract_freespace_ompl_example
# or
pixi run python -m tesseract_robotics.examples.freespace_ompl_example

# Interactive shell (all env vars set)
pixi shell
python -c "from tesseract_robotics.planning import Robot; print('ok')"
```

### Rebuild After C++ Changes

If you modify a C++ binding file (`src/*_bindings.cpp`):

```bash
# Reinstall bindings only (fast, ~30s)
pixi run install
```

To work against unreleased upstream C++, use the `upstream` environment: see
[Building against upstream main](upstream-main.md).

### Task Dependency Chain

Tasks use `depends-on` for ordered execution:

```mermaid
flowchart LR
    test --> install
    build --> install
```

Running `pixi run test` reinstalls the bindings first, since it depends on `install`.

### Environments

The default environment is for local development; `py312` is the one CI builds wheels in:

```bash
# Default environment (Python from pixi.lock)
pixi run test

# The wheel build environment
pixi run -e py312 test
```

Environments defined in `pyproject.toml`:

| Environment | Python | Usage |
|-------------|--------|-------|
| `default` | from lockfile | Local development |
| `py312` | 3.12.x | CI wheels (abi3) |
| `py315` | 3.15.x | Suite on the newest Python |
| `upstream` | 3.12.x | 0.36 inner loop, C++ built from `upstream/` |

[How the binaries are built](binaries.md#pixi-environments-where-each-one-gets-its-c) lists where each environment gets its C++.

### Dependency Management

All dependencies live in `pyproject.toml` under `[tool.pixi.dependencies]` (conda packages) and `[tool.pixi.pypi-dependencies]` (PyPI packages).

```bash
# Add a conda dependency
pixi add some-package

# Add a PyPI dependency
pixi add --pypi some-package

# Update lockfile after editing pyproject.toml manually
pixi install

# See what's installed
pixi list
```

Critical pins: `tesseract-robotics ==0.36.0.dev20261103`, `tesseract-robotics-planning ==0.36.0.dev20261014` and `trajopt ==0.36.0.dev20261013`, from the `tesseract-robotics/label/dev` channel, plus `taskflow 3.11.*` (header-only, matched to the planning build). Everything else (eigen, boost, …) follows from those packages' `run_exports`; pinning more deadlocks the solve.

### CONDA_PREFIX and Worktrees

When working in a git worktree, the `CONDA_PREFIX` env var may point to the main repo's `.pixi/envs/default` instead of the worktree's. This causes build failures (e.g. missing `cereal`). Override when building manually:

```bash
CONDA_PREFIX=$(pwd)/.pixi/envs/default pip install -e . --no-build-isolation
```

Using `pixi run` handles this automatically.

### Building Portable Wheels

Editable installs bake absolute paths, so they aren't portable. For distributable wheels:

```bash
pixi run build-wheel                  # bundles native deps (Linux: patchelf, macOS: scripts/repair_macos_wheel.py)
pip install wheelhouse/tesseract*.whl
```

---

## Architecture

```mermaid
graph TD
    subgraph "C++ Libraries"
        A[tesseract_environment]
        B[tesseract_kinematics]
        C[tesseract_collision]
        D[tesseract_planning]
    end

    subgraph "nanobind Modules"
        E[tesseract_environment]
        F[tesseract_kinematics]
        G[tesseract_collision]
        H[tesseract_motion_planners]
    end

    subgraph "Python API"
        I[tesseract_robotics.planning]
        J[Robot, MotionProgram, TaskComposer,<br/>plan_freespace / plan_ompl / plan_cartesian]
    end

    A --> E
    B --> F
    C --> G
    D --> H
    E --> I
    F --> I
    G --> I
    H --> I
    I --> J
```

### Project Layout

```
tesseract_nanobind/
├── pyproject.toml             # pixi workspace + package config
├── pixi.lock                  # locked deps
├── CMakeLists.txt             # nanobind module build
├── src/
│   ├── tesseract_robotics/    # Python package
│   │   ├── planning/          # High-level API (pure Python)
│   │   ├── viewer/            # 3D visualization
│   │   ├── trajopt_ifopt/     # Low-level optimization
│   │   ├── trajopt_sqp/       # SQP solver
│   │   └── <module>/          # nanobind module + __init__.py + .pyi
│   ├── tesseract_nb.h         # Shared precompiled header
│   └── <module>/              # C++ binding source (*_bindings.cpp)
├── tests/                     # pytest tests
├── examples/                  # Usage examples
├── scripts/
│   ├── generate_stubs.py      # Regenerate / check .pyi stubs
│   ├── audit_gil.py           # GIL releases vs nogil_entry_points.yaml (libclang)
│   ├── gil_hotspots.py        # Rank GIL-held time per binding (py-spy --gil)
│   ├── gil_call_counts.py     # pytest plugin: binding calls per call site
│   ├── gil_entry_points.py    # nogil_entry_points.yaml loader
│   ├── nogil_entry_points.yaml # Every binding that releases the GIL, and why
│   ├── build_linux_wheel.sh   # Portable manylinux wheel (patchelf)
│   ├── build_macos_wheel.sh   # Portable macOS wheel
│   └── build_upstream.sh      # 0.36 inner loop: build upstream/ into the env
├── upstream/                  # tesseract, trajopt, tesseract_planning, bpl submodules (0.36)
└── packaging/                 # forked feedstock submodules (0.36 outer loop)
```

---

## Cross-Module Type Resolution

nanobind maintains separate type registries per module. When a function returns a type from another module, that module must be imported first:

```cpp
NB_MODULE(_my_module, m) {
    // Import module that defines the type BEFORE using it
    nb::module_::import_("tesseract_robotics.tesseract_collision._tesseract_collision");

    // Now can return DiscreteContactManager from functions
    .def("getContactManager", [...] { return self.getDiscreteContactManager(); })
}
```

See [Migration Notes](migration.md#cross-module-type-resolution) for details.

---

## Type Checking

```bash
pixi run typecheck
```

Uses pyright configured via `pyrightconfig.json`. Key design decisions:

- **Python code is type-checked** — the `planning/` module and other pure Python code
- **Auto-generated stubs are excluded** — nanobind generates `.pyi` stubs with C++ type artifacts
- **Hand-written stub exception** — `ompl_base/_ompl_base.pyi` is manually maintained

When adding new bindings, declare inheritance in nanobind to get correct stubs:

```cpp
// Good - generates: class Derived(Base)
nb::class_<Derived, Base>(m, "Derived")

// Bad - generates: class Derived (no inheritance)
nb::class_<Derived>(m, "Derived")
```

---

## Stub Generation

After modifying C++ bindings, regenerate the `.pyi` stubs:

```bash
pixi run stubs
```

This finds every nanobind extension module in the installed package and writes each
stub to `src/tesseract_robotics/<module>/`. Each module is rendered by
`nanobind.stubgen` in a fresh interpreter, because which cross-module types resolve
depends on what is already imported. Stubs are committed for IDE support and type
checking.

Stubs are generated, never hand-edited. Edits made by hand are lost at the next
regeneration. Put documentation in the binding's docstring instead.

**Drift gate (gh-159).** `pixi run stubs-check` exits 1 when a committed stub differs
from what the build renders. CI runs it in every build job, on all platforms. Stubgen output depends on the nanobind version, so nanobind is pinned
exactly in `pyproject.toml`. Bumping nanobind means regenerating the stubs in the same
commit.

**No `std::` in stubs.** stubgen writes a C++ type it can't map to Python as a quoted
C++ name. Such a type has no binding or converter, so it also fails at runtime. Spellings
of `std::` types differ by platform (libc++ `std::__1::`, libstdc++ `std::__cxx11::`),
so such a leak also makes the stub platform-dependent. `tests/scripts/test_generate_stubs.py`
rejects them. Quoted `tesseract::…` names are the same kind of gap but
platform-independent. Each one means the type isn't bound, or the module doesn't
`nb::module_::import_()` the module that binds it.

nanobind's `std::unordered_map` caster matches default-allocator maps only.
`src/tesseract_nb.h` adds a caster for Tesseract's `Eigen::aligned_allocator` maps
(`TransformMap` and friends).

---

## Pre-commit Hooks

```bash
pre-commit install
pre-commit install --hook-type pre-push
```

| Hook | Stage | What it does |
|------|-------|-------------|
| ruff check --fix | pre-commit | Auto-fix lint issues |
| ruff format | pre-commit | Format code |
| stage-formatted | pre-commit | Auto-stage ruff changes |
| typecheck (`pixi run typecheck`, pyright) | pre-push | Type check |

Skip when needed: `git commit --no-verify` / `git push --no-verify`

---

## Changelog at Release Time

`CHANGELOG.md` follows [Keep a Changelog](https://keepachangelog.com/) and is
published on the docs site. `scripts/check_changelog.py` checks its release
structure; the Docs workflow runs it before building, so a mis-cut release fails
CI instead of publishing a wrong changelog (0.35.0.7 and 0.35.0.8 were both cut
by hand, and both broke it). To cut `<version>`:

1. Rename `## [Unreleased]` to `## [<version>] - <YYYY-MM-DD>` and add a new,
   empty `## [Unreleased]` above it.
2. Add `[<version>]: https://github.com/tesseract-robotics/tesseract_nanobind/compare/<previous>...<version>`
   above the previous release's link.
3. Point `[Unreleased]:` at `compare/<version>...HEAD`.
4. Run `pixi run check-changelog` before committing.

Anything merged after the tag goes under `## [Unreleased]`, not into the
release's section.

---

## Contributing

1. Fork the repository
2. Create a feature branch (or use a [git worktree](https://git-scm.com/docs/git-worktree))
3. `pixi run build` (first time)
4. Make changes
5. `pixi run test`
6. `pixi run typecheck`
7. Submit a pull request
