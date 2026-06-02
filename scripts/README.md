# Scripts

Development scripts for tesseract_python_nanobind.

## Setup

### `build_tesseract_cpp.sh`
Builds tesseract C++ libraries using colcon. Run from project root.

```bash
./scripts/build_tesseract_cpp.sh
```

## Development

### `env.sh`
Sets environment variables for development. Source before running Python:

```bash
source scripts/env.sh
```

Sets:
- `DYLD_LIBRARY_PATH` - shared library path (required for plugin loading on macOS)
- `TESSERACT_SUPPORT_DIR` - path to URDF/mesh resources
- `TESSERACT_RESOURCE_PATH` - resource locator base path
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

### `generate_stubs.sh`
Regenerates `.pyi` stub files for all nanobind modules using `stubgen`:

```bash
./scripts/generate_stubs.sh
```

Run after any binding changes to keep stubs in sync with actual API.

### `build_docs.sh`
Builds documentation using mkdocs:

```bash
./scripts/build_docs.sh        # build docs
./scripts/build_docs.sh serve  # serve locally at http://localhost:8000
```

### `run_scaling_benchmarks.sh`
Runs OMPL parallel planner scaling benchmarks (1/2/4/8 planners):

```bash
./scripts/run_scaling_benchmarks.sh
```

Tests how examples scale with parallel OMPL planners.

## Building Wheels

### `build-wheel` (pixi task → `build_linux_wheel.sh` / `build_macos_wheel.sh`)
Builds Python wheels for distribution. The pixi `build-wheel` task delegates to
the right platform script: `build_linux_wheel.sh` (patchelf + `$ORIGIN` rpaths +
manylinux retag) is the base task, and the `osx-arm64` target overrides it to
`build_macos_wheel.sh` (delocate + `@loader_path`). Both accept the same modes.

#### Dev mode (fast, for local testing)
```bash
pixi run build-wheel --dev
```
- Builds wheel in `dist/` without bundling
- Fast (~30s) - just runs pip wheel
- **Only works in the pixi env where it was built** (bakes in absolute paths)
- Use for quick iteration when testing in the same env

#### Portable mode (slow, for distribution)
```bash
pixi run build-wheel
```
- Builds fully portable wheel in `wheelhouse/`
- Slow (~5min) - bundles all native deps
- **Works across any pixi/conda env** - all dependencies bundled

#### What portable mode does:
1. Builds wheel with `pip wheel`
2. Bundles linked native deps and fixes rpaths
   (macOS: `delocate-wheel`; Linux: `ldd` + `patchelf --set-rpath`)
3. Manually adds plugin factory libs (not caught by the linker since they're dlopen'd at runtime):
   - `libtesseract_collision_bullet_factories`
   - `libtesseract_collision_fcl_factories`
   - `libtesseract_kinematics_*_factory`
   - `libtesseract_task_composer_*_factories`
4. Patches runtime config YAMLs (`task_composer_config`, robot `search_paths`)
5. Repacks wheel (Linux retags `linux_x86_64` → `manylinux_2_35_x86_64`)

#### Why two modes?
Editable installs (`pip install -e .`) and dev wheels bake absolute env paths into the nanobind modules via `CMAKE_INSTALL_RPATH`. Example:
```
@rpath/libtesseract_environment.dylib
-> .pixi/envs/default/lib/libtesseract_environment.dylib
```

Delocate converts these to `@loader_path` and bundles the dylibs:
```
@loader_path/.dylibs/libtesseract_environment.dylib
```

This makes the wheel self-contained and portable across environments.

#### Install portable wheel:
```bash
pip install wheelhouse/tesseract*.whl

# Test (no DYLD_LIBRARY_PATH needed):
python -c "from tesseract_robotics import tesseract_collision; print('OK')"
```
