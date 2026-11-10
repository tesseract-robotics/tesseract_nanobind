# Scripts

Development scripts for tesseract_python_nanobind.

## Setup

The tesseract C++ libraries are no longer built from source — they come prebuilt
from the [`tesseract-robotics` conda channel](https://github.com/tesseract-robotics-packaging) (declared in `pyproject.toml`). Just
`pixi run install` (editable); pixi installs the C++ libs into the env automatically.
Portable wheels are built by CI without conda (see [Building Wheels](#building-wheels)).

## Development

### `env.sh`
Exports the tesseract resource env vars for ad-hoc tooling. Source it inside an
active pixi/conda env (`pixi shell`) before running Python. Note: the package
auto-configures these same paths from `$CONDA_PREFIX` on first import, so this is
only needed by tools that read the env vars before importing `tesseract_robotics`.

```bash
source scripts/env.sh
```

Sets (all derived from `$CONDA_PREFIX`):
- `TESSERACT_SUPPORT_DIR` - path to URDF/mesh resources (`share/tesseract/support`)
- `TESSERACT_RESOURCE_PATH` - resource locator base path (`share/`)
- `TESSERACT_TASK_COMPOSER_CONFIG_FILE` - task composer plugin config

### `run_benchmarks.sh`
Runs all planning benchmarks and shows summary report.

```bash
./scripts/run_benchmarks.sh
```

Tests all 5 examples:
- Default pipeline timing (5 tests)
- OMPL CPU scaling with 1/2/4/8 planners (skips if example needs TrajOpt)

### `run_tests.sh`
Runs pytest with correct environment. Preferred way to run tests:

```bash
./scripts/run_tests.sh                    # all tests (parallel)
./scripts/run_tests.sh -v                 # verbose with long traceback
./scripts/run_tests.sh -f                 # rerun only failed tests (serial)
./scripts/run_tests.sh -s                 # run serial (no parallel)
./scripts/run_tests.sh -k "freespace"     # filter tests
./scripts/run_tests.sh -x                 # stop on first failure
```

Flags:
- `-v, --verbose`: verbose output with long tracebacks
- `-f, --failed`: rerun only tests that failed last run (runs serial by default for easier debugging)
- `-s, --serial`: disable parallel execution (`-n auto`)

### `generate_stubs.py`
Regenerates the committed `.pyi` stubs of every nanobind extension module with
`nanobind.stubgen` (run through pixi tasks):

```bash
pixi run stubs         # rewrite the stubs
pixi run stubs-check   # exit 1 if any committed stub differs from the build
```

Run `pixi run stubs` after any binding change. CI runs the check in every build job.

### `build_docs.sh`
Builds documentation using mkdocs:

```bash
./scripts/build_docs.sh        # build docs
./scripts/build_docs.sh serve  # serve locally at http://localhost:8000
```

### `check_changelog.py`
Checks `CHANGELOG.md`'s release structure: `## [Unreleased]` first, dated and
unique release headings listed newest first, and compare links that chain each
release to the one before it. The Docs workflow runs it before building; see
*Changelog at Release Time* in the developer guide for how to cut a release.

```bash
pixi run check-changelog
```

### `run_scaling_benchmarks.sh`
Runs OMPL parallel planner scaling benchmarks (1/2/4/8 planners):

```bash
./scripts/run_scaling_benchmarks.sh
```

Tests how examples scale with parallel OMPL planners.

## Building Wheels

The PyPI wheels are built without conda by `.github/workflows/wheels.yml`, the
method of [tesseract_python](https://github.com/tesseract-robotics/tesseract_python):
the tesseract C++ stack (`dependencies.rosinstall`) is built from source with colcon
against vcpkg dependencies, `pip wheel .` builds the bindings against it, and
auditwheel (Linux), delvewheel (Windows) or `repair_macos_wheel.py` (macOS)
bundles the closure. The macOS repair batches load-command edits and signs each
changed image once; repeated `--lib-dir` arguments name the native, vcpkg and
Homebrew OpenMP library directories. It preserves plugin YAML and derives the
minimum macOS tag from the bundled libraries.
The plugin factory libraries are linked into the extension modules and registered
through their anchors, so the repair tools bundle them like any other dependency.
See [docs/developer/binaries.md](../docs/developer/binaries.md#building-a-wheel).

