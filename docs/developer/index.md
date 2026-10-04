# Developer Guide

## Pixi Workspace

This project uses [pixi](https://pixi.sh) exclusively for package management. Pixi manages all C++ libraries, Python packages, build tools, and platform-specific dependencies through a single lockfile (`pixi.lock`). No pip, conda, poetry, or venv.

The tesseract C++ libraries are not built here: they are prebuilt conda packages from the `tesseract-robotics` channel. [How the binaries are built](binaries.md) traces every binary from the upstream source to the PyPI wheel.

### Available Tasks

```bash
pixi task list
```

| Task | Description | What it does |
|------|-------------|--------------|
| `build` | Build bindings | alias for `install` |
| `install` | Install bindings | editable `pip install -e . --no-build-isolation` against the conda tesseract libs |
| `test` | Run tests | pytest with xdist parallel (depends on `install`) |
| `build-wheel` | Portable wheel | `scripts/build_linux_wheel.sh` / `build_macos_wheel.sh` (see [binaries](binaries.md)) |
| `typecheck` | Type check | pyright on `src/tesseract_robotics/` |
| `lint` | Lint | ruff check |
| `fmt` | Format | ruff format |
| `docs` | Live docs | mkdocs serve with auto-reload |
| `docs-build` | Build docs | Static site to `site/` |

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

The workspace defines multiple Python environments for CI testing:

```bash
# Default environment (Python from pixi.lock)
pixi run test

# Specific Python version
pixi run -e py39 test
pixi run -e py312 test
```

Environments defined in `pyproject.toml`:

| Environment | Python | Usage |
|-------------|--------|-------|
| `default` | 3.14 (unpinned, `>=3.10`) | Local development |
| `py39` | 3.9.x | CI wheels; C++ from the `py312` env via `TESSERACT_CPP_PREFIX` |
| `py310` | 3.10.x | CI wheels |
| `py311` | 3.11.x | CI wheels |
| `py312` | 3.12.x | CI wheels (abi3) |
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

Critical pins: only `tesseract-robotics ==0.35.0` and `tesseract-robotics-planning ==0.35.0`. Everything else (trajopt, eigen, boost, taskflow, …) follows from those packages' `run_exports`; pinning more deadlocks the solve.

### CONDA_PREFIX and Worktrees

When working in a git worktree, the `CONDA_PREFIX` env var may point to the main repo's `.pixi/envs/default` instead of the worktree's. This causes build failures (e.g. missing `cereal`). Override when building manually:

```bash
CONDA_PREFIX=$(pwd)/.pixi/envs/default pip install -e . --no-build-isolation
```

Using `pixi run` handles this automatically.

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
│   ├── generate_stubs.sh      # Regenerate .pyi stubs
│   ├── build_linux_wheel.sh   # Portable manylinux wheel (patchelf)
│   ├── build_macos_wheel.sh   # Portable macOS wheel (delocate)
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

After modifying C++ bindings, regenerate `.pyi` stubs:

```bash
bash scripts/generate_stubs.sh
```

This introspects all 23 nanobind modules and writes stubs to `src/tesseract_robotics/<module>/`. Stubs are committed to the repo for IDE support and type checking.

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

## Contributing

1. Fork the repository
2. Create a feature branch (or use a [git worktree](https://git-scm.com/docs/git-worktree))
3. `pixi run build` (first time)
4. Make changes
5. `pixi run test`
6. `pixi run typecheck`
7. Submit a pull request
