# macOS wheels to PyPI — Split Workflow Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build macOS arm64 wheels on CI via a second workflow file that invokes the same pixi tasks used locally, eliminating sync drift between CI inline YAML and `scripts/build_wheel.sh`.

**Architecture:** Split `wheels.yml` into two workflows — `wheels.yml` (Linux-only, stripped of macOS branches) and `wheels-macos.yml` (new, macOS arm64 only, calls `pixi run build-cpp` + `pixi run build-wheel`). Refactor `scripts/build_wheel.sh` to drop its `conda create --clone` block so it runs in the active pixi env (local and CI). Add a `build-wheel` pixi task.

**Tech Stack:** GitHub Actions, pixi, colcon, pip, delocate, `pypa/gh-action-pypi-publish`.

**Related docs:**
- Spec: `docs/superpowers/specs/2026-04-22-macos-wheels-to-pypi-design.md`
- Local macOS wheel script: `scripts/build_wheel.sh`
- Local macOS C++ build script: `scripts/build_tesseract_cpp.sh`

**Branch:** `jf/macos-wheels-to-pypi` (reset to spec commit `aeb684e`, speculation commits from first attempt wiped).

---

## File Structure

Four files touched:

- `scripts/build_wheel.sh` — refactor: strip the `conda create --clone` block, make the portable path run in the active env.
- `pyproject.toml` — add pixi task `build-wheel`.
- `.github/workflows/wheels-macos.yml` — new file.
- `.github/workflows/wheels.yml` — strip all macOS-conditional branches, simplify to Linux-only.

Each is its own commit. After commit 3 (CI split), running both workflows validates the whole thing end-to-end.

---

## Task 1: Refactor `scripts/build_wheel.sh`

**Goal:** Remove the `conda create --clone` isolation block so the script runs in the active env (local: pixi shell; CI: pixi-activated session). The `--dev` path is untouched.

**Files:**
- Modify: `scripts/build_wheel.sh`

- [ ] **Step 1: Apply the edit**

Replace lines 49–63 of `scripts/build_wheel.sh` (from `# Full portable build with temp env` through `pip wheel . -w dist/ --no-build-isolation`). Current content:

```bash
# Full portable build with temp env
BUILD_ENV="tesseract_wheel_build_$$"
echo "Building portable wheel..."
echo "Using temporary env: $BUILD_ENV"

eval "$(conda shell.zsh hook)"
conda create -n "$BUILD_ENV" --clone tesseract_nb -y
conda activate "$BUILD_ENV"

pip install delocate setuptools-scm

rm -rf dist/ wheelhouse/

echo "Building wheel..."
pip wheel . -w dist/ --no-build-isolation
```

Change to:

```bash
# Full portable build in the active env (pixi locally, pixi-activated in CI).
# Historical conda-clone isolation was removed — pixi already provides isolation
# via its per-env pyproject.toml features.
echo "Building portable wheel in active env: ${CONDA_PREFIX:-NO ENV}"
if [[ -z "$CONDA_PREFIX" ]]; then
    echo "❌ No active pixi/conda env. Run via 'pixi shell' or 'pixi run build-wheel'."
    exit 1
fi

pip install delocate setuptools-scm

rm -rf dist/ wheelhouse/

echo "Building wheel..."
pip wheel . -w dist/ --no-build-isolation
```

Also delete lines 122–123 of the original file:

```bash
conda deactivate
conda env remove -n "$BUILD_ENV" -y
```

(These no longer have a matching `conda activate` / `conda create` — the cleanup is no-op after the refactor.)

Everything else in the script stays verbatim (DYLD_LIBRARY_PATH export, CMAKE_PREFIX_PATH, the delocate step, plugin bundle, YAML patch, repack).

- [ ] **Step 2: Verify the shell script parses**

Run:
```bash
zsh -n scripts/build_wheel.sh && echo OK
```
Expected: `OK`.

- [ ] **Step 3: Verify conda references are gone from the portable path**

Run:
```bash
grep -n "conda " scripts/build_wheel.sh || echo "clean"
```
Expected: `clean` (no conda references remaining).

- [ ] **Step 4: Commit**

Run:
```bash
git add scripts/build_wheel.sh
git -c user.name="Jelle Feringa" -c user.email="jelleferinga@gmail.com" \
  commit --author="Jelle Feringa <jelleferinga@gmail.com>" \
  -m "refactor: strip conda clone from build_wheel.sh, run in active env"
```

---

## Task 2: Add `build-wheel` pixi task

**Files:**
- Modify: `pyproject.toml`

- [ ] **Step 1: Apply the edit**

Open `pyproject.toml`. Under `[tool.pixi.tasks]` (around line 202), after the existing `build-cpp` task and before `install`, add:

```toml
build-wheel = { cmd = "zsh scripts/build_wheel.sh", description = "Build portable wheel (macOS: delocate + plugins + YAML patch)" }
```

Keep the existing `build-cpp`, `install`, `build`, `test`, `test-benchmark`, `lint`, `fmt`, `typecheck`, `docs`, `docs-build` tasks unchanged.

- [ ] **Step 2: Verify pyproject.toml parses and the task is registered**

Run:
```bash
python -c "import tomllib; d=tomllib.loads(open('pyproject.toml').read()); print(d['tool']['pixi']['tasks']['build-wheel'])"
```
Expected: `{'cmd': 'zsh scripts/build_wheel.sh', 'description': 'Build portable wheel (macOS: delocate + plugins + YAML patch)'}`.

Also verify pixi recognises the task:
```bash
pixi task list 2>&1 | grep build-wheel
```
Expected: shows `build-wheel` in the task list.

- [ ] **Step 3: Commit**

Run:
```bash
git add pyproject.toml
git -c user.name="Jelle Feringa" -c user.email="jelleferinga@gmail.com" \
  commit --author="Jelle Feringa <jelleferinga@gmail.com>" \
  -m "feat(pixi): add build-wheel task"
```

---

## Task 3: Create `.github/workflows/wheels-macos.yml`

**Files:**
- Create: `.github/workflows/wheels-macos.yml`

- [ ] **Step 1: Write the new file**

Full content (paste exactly):

```yaml
name: Build & Test (macOS)

on:
  push:
    branches:
      - main
    tags:
      - '[0-9]+.[0-9]+.[0-9]+'
      - '[0-9]+.[0-9]+.[0-9]+.[0-9]+'
    paths-ignore:
      - '**.md'
      - 'docs/**'
      - '.gitignore'
      - 'LICENSE'
  pull_request:
    paths-ignore:
      - '**.md'
      - 'docs/**'
      - '.gitignore'
      - 'LICENSE'
  workflow_dispatch:

jobs:
  build:
    runs-on: macos-14
    strategy:
      fail-fast: false
      matrix:
        python: ['3.9', '3.10', '3.11', '3.12']
    defaults:
      run:
        shell: bash -el {0}
    steps:
    - uses: actions/checkout@v4
      with:
        path: ws/src/tesseract_nanobind
    - name: brew
      run: |
        brew install automake autoconf libtool
        # libomp comes from pixi (llvm-openmp) to avoid duplicate library crashes
    - name: setup pixi
      uses: prefix-dev/setup-pixi@v0.8.0
      env:
        PIXI_ENV: py${{ matrix.python == '3.9' && '39' || matrix.python == '3.10' && '310' || matrix.python == '3.11' && '311' || '312' }}
      with:
        manifest-path: ws/src/tesseract_nanobind/pyproject.toml
        cache: true
        environments: ${{ env.PIXI_ENV }}
        activate-environment: ${{ env.PIXI_ENV }}
    - name: typecheck
      working-directory: ws/src/tesseract_nanobind
      run: pixi run typecheck
    - name: cache vcs repos
      uses: actions/cache@v4
      with:
        path: |
          ws/src/tesseract
          ws/src/tesseract_planning
          ws/src/trajopt
          ws/src/descartes_light
          ws/src/opw_kinematics
          ws/src/ruckig
        key: vcs-${{ hashFiles('ws/src/tesseract_nanobind/dependencies.rosinstall') }}
    - name: vcs import
      working-directory: ws/src
      run: vcs import --input tesseract_nanobind/dependencies.rosinstall --skip-existing
    - name: cache colcon build
      uses: actions/cache@v4
      id: colcon-cache
      with:
        path: ws/install
        key: colcon-macos-v1-py${{ matrix.python }}-${{ hashFiles('ws/src/tesseract_nanobind/dependencies.rosinstall', 'ws/src/tesseract_nanobind/pyproject.toml') }}
    - name: build C++ deps
      if: steps.colcon-cache.outputs.cache-hit != 'true'
      working-directory: ws/src/tesseract_nanobind
      run: pixi run build-cpp
    - name: build wheel
      working-directory: ws/src/tesseract_nanobind
      run: pixi run build-wheel
    - name: archive wheel
      uses: actions/upload-artifact@v4
      with:
        name: python-${{ matrix.python }}-macos-arm64
        path: ws/src/tesseract_nanobind/wheelhouse/tesseract*.whl
    - name: archive logs
      if: failure()
      uses: actions/upload-artifact@v4
      with:
        name: build-logs-macos-arm64-py${{ matrix.python }}
        path: '**/*.log'
        retention-days: 2

  test-wheel:
    needs: build
    runs-on: macos-14
    continue-on-error: ${{ matrix.python == '3.9' }}
    strategy:
      fail-fast: false
      matrix:
        python: ['3.9', '3.12']
    steps:
      - uses: actions/checkout@v4
        with:
          path: tesseract_nanobind
      - uses: actions/setup-python@v5
        with:
          python-version: ${{ matrix.python }}
      - uses: actions/download-artifact@v4
        with:
          name: python-${{ matrix.python }}-macos-arm64
          path: wheelhouse/
      - name: install wheel in virgin env
        run: |
          python -m venv test_env
          source test_env/bin/activate
          pip install wheelhouse/tesseract*.whl
      - name: smoke test
        run: |
          source test_env/bin/activate
          python -c "import tesseract_robotics; print('Version:', tesseract_robotics.__version__)"
          python -c "from tesseract_robotics import tesseract_common, tesseract_geometry, tesseract_scene_graph"
      - name: run full tests
        run: |
          source test_env/bin/activate
          pip install pytest pytest-xdist scipy
          cd tesseract_nanobind
          pytest tests -n auto \
            --ignore=tests/tesseract_time_parameterization \
            --ignore=tests/examples \
            --ignore=tests/tesseract_kinematics \
            --ignore=tests/benchmarks \
            -v --tb=short

  publish:
    needs: [build, test-wheel]
    if: startsWith(github.ref, 'refs/tags/')
    runs-on: macos-14
    environment: pypi
    permissions:
      id-token: write
    steps:
    - uses: actions/download-artifact@v4
      with:
        pattern: python-*-macos-arm64
        merge-multiple: true
        path: dist/
    - name: List wheels
      run: ls -la dist/
    - name: Publish to PyPI
      uses: pypa/gh-action-pypi-publish@release/v1
```

Design notes baked in:
- Cache key prefix `colcon-macos-v1-` (separate from Linux's `colcon-v11-`) so the macOS cache is independent.
- `continue-on-error: ${{ matrix.python == '3.9' }}` — matches Linux test-wheel (py3.9 has intermittent GC segfaults with pytest-xdist).
- Pytest ignores mirror Linux. `debug libs` step is omitted — it was Linux-only anyway (`patchelf`/`ldd`).
- `publish` job runs on `macos-14`. The Trusted Publisher config only depends on the `environment: pypi` gate + OIDC `id-token: write`; the runner OS doesn't matter for twine upload.

- [ ] **Step 2: Verify YAML parses**

Run:
```bash
python -c "import yaml; d=yaml.safe_load(open('.github/workflows/wheels-macos.yml')); print(list(d['jobs']))"
```
Expected: `['build', 'test-wheel', 'publish']`.

- [ ] **Step 3: Verify jobs have expected matrix sizes**

Run:
```bash
python -c "
import yaml
d = yaml.safe_load(open('.github/workflows/wheels-macos.yml'))
print('build python:', d['jobs']['build']['strategy']['matrix']['python'])
print('test-wheel python:', d['jobs']['test-wheel']['strategy']['matrix']['python'])
print('publish gate:', d['jobs']['publish']['if'])
"
```
Expected:
```
build python: ['3.9', '3.10', '3.11', '3.12']
test-wheel python: ['3.9', '3.12']
publish gate: startsWith(github.ref, 'refs/tags/')
```

- [ ] **Step 4: Commit**

Run:
```bash
git add .github/workflows/wheels-macos.yml
git -c user.name="Jelle Feringa" -c user.email="jelleferinga@gmail.com" \
  commit --author="Jelle Feringa <jelleferinga@gmail.com>" \
  -m "ci: add wheels-macos.yml (runs pixi tasks, no sync drift)"
```

---

## Task 4: Strip macOS branches from `wheels.yml`

**Files:**
- Modify: `.github/workflows/wheels.yml`

**Goal:** Remove every `if: matrix.config.os == 'macos'` step and the macOS matrix entry. Simplify `matrix.config` away entirely since there's only one config left (Linux x64). The workflow ends up covering Linux only.

- [ ] **Step 1: Rewrite `wheels.yml`**

Replace the entire file contents. The result should be Linux-only. Exact content:

```yaml
name: Build & Test (Linux)

on:
  push:
    branches:
      - main
    tags:
      - '[0-9]+.[0-9]+.[0-9]+'
      - '[0-9]+.[0-9]+.[0-9]+.[0-9]+'
    paths-ignore:
      - '**.md'
      - 'docs/**'
      - '.gitignore'
      - 'LICENSE'
  pull_request:
    paths-ignore:
      - '**.md'
      - 'docs/**'
      - '.gitignore'
      - 'LICENSE'
  workflow_dispatch:

jobs:
  build:
    runs-on: ubuntu-22.04
    strategy:
      fail-fast: false
      matrix:
        python: ['3.9', '3.10', '3.11', '3.12']
    defaults:
      run:
        shell: bash -el {0}
    steps:
    - uses: actions/checkout@v4
      with:
        path: ws/src/tesseract_nanobind
    - name: setup pixi
      uses: prefix-dev/setup-pixi@v0.8.0
      env:
        PIXI_ENV: py${{ matrix.python == '3.9' && '39' || matrix.python == '3.10' && '310' || matrix.python == '3.11' && '311' || '312' }}
      with:
        manifest-path: ws/src/tesseract_nanobind/pyproject.toml
        cache: true
        environments: ${{ env.PIXI_ENV }}
        activate-environment: ${{ env.PIXI_ENV }}
    - name: typecheck
      working-directory: ws/src/tesseract_nanobind
      run: pixi run typecheck
    - name: cache vcs repos
      uses: actions/cache@v4
      with:
        path: |
          ws/src/tesseract
          ws/src/tesseract_planning
          ws/src/trajopt
          ws/src/descartes_light
          ws/src/opw_kinematics
          ws/src/ruckig
        key: vcs-${{ hashFiles('ws/src/tesseract_nanobind/dependencies.rosinstall') }}
    - name: vcs import
      working-directory: ws/src
      run: vcs import --input tesseract_nanobind/dependencies.rosinstall --skip-existing
    - name: patch upstream missing includes
      working-directory: ws/src
      run: |
        # tesseract_planning 0.34.0 uses std::runtime_error without #include <stdexcept>
        # Works on macOS (transitive include) but fails on Linux GCC
        find tesseract_planning -name '*.h' -path '*/poly/*' | xargs grep -l 'std::runtime_error' | while read f; do
          if ! grep -q '#include <stdexcept>' "$f"; then
            sed -i '/#include <string>/a #include <stdexcept>' "$f"
            echo "  patched: $f"
          fi
        done
    - name: cache colcon build
      uses: actions/cache@v4
      id: colcon-cache
      with:
        path: ws/install
        key: colcon-linux-v11-py${{ matrix.python }}-${{ hashFiles('ws/src/tesseract_nanobind/dependencies.rosinstall', 'ws/src/tesseract_nanobind/pyproject.toml') }}
    - name: colcon build C++ deps
      if: steps.colcon-cache.outputs.cache-hit != 'true'
      working-directory: ws
      run: |
        export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$GITHUB_WORKSPACE/ws/install/lib
        export CMAKE_PREFIX_PATH=$CONDA_PREFIX:$GITHUB_WORKSPACE/ws/install
        export LIBRARY_PATH=$CONDA_PREFIX/lib:$LIBRARY_PATH
        export CMAKE_POLICY_VERSION_MINIMUM=3.5

        colcon build --merge-install \
            --packages-ignore tesseract_python tesseract_examples vhacd qpoases tesseract_nanobind tesseract_viewer_python twc_application twc_motion_planning twc_msgs twc_support \
            --event-handlers console_cohesion+ \
            --cmake-force-configure \
            --cmake-args -GNinja -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_CXX_VISIBILITY_PRESET=default -DCMAKE_VISIBILITY_INLINES_HIDDEN=OFF \
            -DINSTALL_OMPL=OFF -DINSTALL_OMPL_TAG=master -DBUILD_CLOUD_CLIENT=OFF -DBUILD_IPOPT=OFF -DBUILD_SNOPT=OFF \
            -DBUILD_SHARED_LIBS=ON -DTESSERACT_ENABLE_EXAMPLES=OFF -DTESSERACT_BUILD_TRAJOPT_IFOPT=ON \
            -DTESSERACT_ENABLE_TESTING=OFF \
            -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
            '-DCMAKE_INSTALL_RPATH=$ORIGIN' \
            -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON
    - name: build nanobind wheel
      working-directory: ws/src/tesseract_nanobind
      run: |
        export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$GITHUB_WORKSPACE/ws/install/lib
        export CMAKE_PREFIX_PATH=$CONDA_PREFIX:$GITHUB_WORKSPACE/ws/install
        pip install auditwheel setuptools-scm
        CMAKE_ARGS="-DCMAKE_POLICY_VERSION_MINIMUM=3.5" \
          pip wheel . -w dist/ --no-build-isolation
    - name: bundle plugins into wheel
      working-directory: ws/src/tesseract_nanobind
      run: |
        # Plugin factories are runtime-loaded (dlopen), not caught by auditwheel
        # Bundle them BEFORE auditwheel so deps get repaired together
        WHEEL_FILE=$(ls dist/tesseract*.whl)
        WHEEL_DIR=$(mktemp -d)
        unzip -q "$WHEEL_FILE" -d "$WHEEL_DIR"

        PLUGINS_DIR="$WHEEL_DIR/tesseract_robotics"

        PLUGINS=(
          libtesseract_collision_bullet_factories.so
          libtesseract_collision_fcl_factories.so
          libtesseract_kinematics_core_factories.so
          libtesseract_kinematics_kdl_factories.so
          libtesseract_kinematics_opw_factory.so
          libtesseract_kinematics_ur_factory.so
          libtesseract_task_composer_factories.so
          libtesseract_task_composer_planning_factories.so
          libtesseract_task_composer_taskflow_factories.so
        )

        for plugin in "${PLUGINS[@]}"; do
          if [[ -f "$GITHUB_WORKSPACE/ws/install/lib/$plugin" ]]; then
            cp "$GITHUB_WORKSPACE/ws/install/lib/$plugin" "$PLUGINS_DIR/"
            echo "  Added: $plugin"
          fi
        done

        # Patch task_composer_config YAMLs (resolved at runtime by __init__.py)
        echo "Patching task composer configs..."
        for yaml_file in "$WHEEL_DIR/tesseract_robotics/data/task_composer_config"/*.yaml; do
          if [[ -f "$yaml_file" ]] && grep -q '/usr/local/lib' "$yaml_file"; then
            sed -i 's|/usr/local/lib|"@PLUGIN_PATH@"|g' "$yaml_file"
            echo "  Patched: $(basename $yaml_file)"
          fi
        done

        # Remove search_paths from robot YAMLs (forces use of env vars set by __init__.py)
        echo "Removing hardcoded search_paths from robot YAMLs..."
        find "$WHEEL_DIR/tesseract_robotics/data/tesseract_support" -name "*.yaml" -type f | while read yaml_file; do
          if grep -q 'search_paths:' "$yaml_file"; then
            sed -i '/search_paths:/d; /^[[:space:]]*- \/.*$/d' "$yaml_file"
            echo "  Removed search_paths from: $(basename $yaml_file)"
          fi
        done

        # Repack wheel (auditwheel will repair it next)
        rm "$WHEEL_FILE"
        cd "$WHEEL_DIR"
        echo "Wheel contents before auditwheel:"
        find . -name "*.so" | head -20
        zip -rq "$GITHUB_WORKSPACE/ws/src/tesseract_nanobind/dist/$(basename $WHEEL_FILE)" .
        rm -rf "$WHEEL_DIR"
    - name: bundle deps
      working-directory: ws/src/tesseract_nanobind
      run: |
        pip install patchelf

        WHEEL_FILE=$(ls dist/tesseract*.whl)
        WHEEL_DIR=$(mktemp -d)
        unzip -q "$WHEEL_FILE" -d "$WHEEL_DIR"
        PKG_DIR="$WHEEL_DIR/tesseract_robotics"

        echo "Bundling all .so dependencies into package root..."

        echo "Copying tesseract libs..."
        echo "Files in ws/install/lib matching *.so* (for debug):"
        ls -la $GITHUB_WORKSPACE/ws/install/lib/*.so* 2>/dev/null | head -20 || true
        for lib in $GITHUB_WORKSPACE/ws/install/lib/*.so*; do
          if [[ -f "$lib" && ! -L "$lib" ]]; then
            cp "$lib" "$PKG_DIR/"
            echo "  Copied: $(basename $lib)"
          elif [[ -L "$lib" ]]; then
            target=$(readlink -f "$lib")
            if [[ -f "$target" ]]; then
              cp "$target" "$PKG_DIR/$(basename $lib)"
              echo "  Copied (from symlink): $(basename $lib)"
            fi
          fi
        done

        echo "Finding and copying external deps..."
        export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$GITHUB_WORKSPACE/ws/install/lib

        deps_file=$(mktemp)
        for so in "$PKG_DIR"/*.so*; do
          if [[ -f "$so" ]]; then
            ldd "$so" 2>/dev/null | grep "=>" | awk '{print $3}' | grep -v "^$" >> "$deps_file" || true
          fi
        done

        sort -u "$deps_file" | while read dep; do
          if [[ -f "$dep" && "$dep" != /lib* && "$dep" != /usr/lib* ]]; then
            base=$(basename "$dep")
            if [[ ! -f "$PKG_DIR/$base" ]]; then
              cp "$dep" "$PKG_DIR/"
              echo "  Copied: $base"
            fi
          fi
        done
        rm "$deps_file"

        echo "Setting rpath on all libs..."
        for lib in "$PKG_DIR"/*.so*; do
          if [[ -f "$lib" && ! -L "$lib" ]]; then
            patchelf --set-rpath '$ORIGIN' "$lib" 2>/dev/null || true
          fi
        done

        echo "Patching Python extensions in subdirectories..."
        find "$PKG_DIR" -mindepth 2 -name "*.so" -type f | while read ext; do
          patchelf --set-rpath '$ORIGIN/..' "$ext" 2>/dev/null || true
          echo "  Patched: $(basename $ext)"
        done

        echo "Checking for missing deps..."
        for so in "$PKG_DIR"/*.so*; do
          if [[ -f "$so" && ! -L "$so" ]]; then
            missing=$(ldd "$so" 2>&1 | grep "not found" || true)
            if [[ -n "$missing" ]]; then
              echo "WARNING: $(basename $so) has missing deps:"
              echo "$missing"
            fi
          fi
        done

        WHEELHOUSE="$GITHUB_WORKSPACE/ws/src/tesseract_nanobind/wheelhouse"
        mkdir -p "$WHEELHOUSE"
        WHEEL_NAME=$(basename "$WHEEL_FILE" | sed 's/linux_x86_64/manylinux_2_35_x86_64/')
        rm -f "$WHEEL_FILE"
        cd "$WHEEL_DIR"
        zip -rq "$WHEELHOUSE/$WHEEL_NAME" .
        cd -
        rm -rf "$WHEEL_DIR"

        echo "Wheel contents:"
        unzip -l "$WHEELHOUSE/$WHEEL_NAME" | grep "\.so" | wc -l
        echo "shared libraries bundled"
    - name: archive wheels
      uses: actions/upload-artifact@v4
      with:
        name: python-${{ matrix.python }}-linux-x64
        path: ws/src/tesseract_nanobind/wheelhouse/tesseract*.whl
    - name: archive logs
      if: failure()
      uses: actions/upload-artifact@v4
      with:
        name: build-logs-linux-x64-py${{ matrix.python }}
        path: '**/*.log'
        retention-days: 2

  test-wheel:
    needs: build
    runs-on: ubuntu-24.04
    continue-on-error: ${{ matrix.python == '3.9' }}
    strategy:
      fail-fast: false
      matrix:
        python: ['3.9', '3.12']
    steps:
      - uses: actions/checkout@v4
        with:
          path: tesseract_nanobind
      - uses: actions/setup-python@v5
        with:
          python-version: ${{ matrix.python }}
      - uses: actions/download-artifact@v4
        with:
          name: python-${{ matrix.python }}-linux-x64
          path: wheelhouse/
      - name: install wheel in virgin env
        run: |
          python -m venv test_env
          source test_env/bin/activate
          pip install wheelhouse/tesseract*.whl
      - name: debug libs
        run: |
          source test_env/bin/activate
          PKG_DIR=$(python -c "import tesseract_robotics; from pathlib import Path; print(Path(tesseract_robotics.__file__).parent)")
          echo "Package dir: $PKG_DIR"
          echo "--- .so files in package (count) ---"
          ls "$PKG_DIR"/*.so* 2>/dev/null | wc -l
          echo "--- Plugin factories ---"
          ls -la "$PKG_DIR"/lib*_factories.so 2>/dev/null | head -5 || echo "No plugin factories"
          echo "--- Missing deps in any lib ---"
          for so in "$PKG_DIR"/*.so*; do
            missing=$(ldd "$so" 2>&1 | grep "not found" || true)
            if [[ -n "$missing" ]]; then
              echo "$(basename $so): $missing"
            fi
          done
          echo "--- End missing deps check ---"
      - name: smoke test
        run: |
          source test_env/bin/activate
          python -c "import tesseract_robotics; print('Version:', tesseract_robotics.__version__)"
          python -c "from tesseract_robotics import tesseract_common, tesseract_geometry, tesseract_scene_graph"
      - name: run full tests
        run: |
          source test_env/bin/activate
          pip install pytest pytest-xdist scipy
          cd tesseract_nanobind
          pytest tests -n auto \
            --ignore=tests/tesseract_time_parameterization \
            --ignore=tests/examples \
            --ignore=tests/tesseract_kinematics \
            --ignore=tests/benchmarks \
            -v --tb=short

  publish:
    needs: [build, test-wheel]
    if: startsWith(github.ref, 'refs/tags/')
    runs-on: ubuntu-latest
    environment: pypi
    permissions:
      id-token: write
    steps:
    - uses: actions/download-artifact@v4
      with:
        pattern: python-*-linux-*
        merge-multiple: true
        path: dist/
    - name: List wheels
      run: ls -la dist/
    - name: Publish to PyPI
      uses: pypa/gh-action-pypi-publish@release/v1
```

Key differences from the prior `wheels.yml`:
- Workflow `name` renamed to `Build & Test (Linux)` to mirror the macOS workflow.
- `strategy.matrix` no longer has a `config` nest — just `python`. Runner is fixed at `ubuntu-22.04` for `build` and `ubuntu-24.04` for `test-wheel`.
- All macOS-conditional steps removed.
- Step names dropped the `(Linux)` suffix since there's no other platform in this file.
- Cache key prefix changed to `colcon-linux-v11-` for clarity (still includes python version + rosinstall hash). v11 preserves existing cache reuse.
- `debug libs` step retained (Linux-only tooling — appropriate here).
- `publish` pattern stays `python-*-linux-*` because this workflow only publishes Linux wheels; macOS publishes from `wheels-macos.yml`.
- Trimmed verbose `debug libs` output block (removed inspection of specific plugin rpaths, ldd dumps — kept the "missing deps" check which is the signal).

- [ ] **Step 2: Verify YAML parses and structure is right**

Run:
```bash
python -c "
import yaml
d = yaml.safe_load(open('.github/workflows/wheels.yml'))
print('jobs:', list(d['jobs']))
print('build runs-on:', d['jobs']['build']['runs-on'])
print('build matrix:', d['jobs']['build']['strategy']['matrix'])
print('publish gate:', d['jobs']['publish']['if'])
print('publish pattern:', d['jobs']['publish']['steps'][0]['with']['pattern'])
"
```

Expected:
```
jobs: ['build', 'test-wheel', 'publish']
build runs-on: ubuntu-22.04
build matrix: {'python': ['3.9', '3.10', '3.11', '3.12']}
publish gate: startsWith(github.ref, 'refs/tags/')
publish pattern: python-*-linux-*
```

- [ ] **Step 3: Confirm no `matrix.config` references remain**

Run:
```bash
grep -n "matrix\.config\|macos\|DYLD" .github/workflows/wheels.yml || echo "clean"
```
Expected: `clean` — no macOS-related references in the Linux workflow.

- [ ] **Step 4: Commit**

Run:
```bash
git add .github/workflows/wheels.yml
git -c user.name="Jelle Feringa" -c user.email="jelleferinga@gmail.com" \
  commit --author="Jelle Feringa <jelleferinga@gmail.com>" \
  -m "ci: strip macos from wheels.yml (moved to wheels-macos.yml)"
```

---

## Task 5: Push branch + update PR #45

**Files:**
- None (git push only)

- [ ] **Step 1: Push the refactor stack**

Run:
```bash
git log --oneline origin/main..HEAD
```

Expected (most recent first): 4 new commits on top of the spec commit:
```
<sha> ci: strip macos from wheels.yml (moved to wheels-macos.yml)
<sha> ci: add wheels-macos.yml (runs pixi tasks, no sync drift)
<sha> feat(pixi): add build-wheel task
<sha> refactor: strip conda clone from build_wheel.sh, run in active env
aeb684e docs: spec for macos wheels to pypi
```

- [ ] **Step 2: Push**

Run:
```bash
git push origin jf/macos-wheels-to-pypi
```

No `--force-with-lease` needed here — the reset already happened and we're now adding new commits on top, not rewriting.

Pre-push hook runs pytest-testmon; it should pass (no code changes to runtime, only CI + scripts).

- [ ] **Step 3: Update PR #45 body (optional but tidy)**

The old PR body references the three-edit approach. Update it to reflect the split. Run:

```bash
gh pr edit 45 --repo tesseract-robotics/tesseract_nanobind --body "$(cat <<'EOF'
## Summary

Publish macOS arm64 wheels to PyPI by splitting CI into two workflows that reuse the local build scripts (single source of truth).

**Changes:**
- `refactor: strip conda clone from build_wheel.sh, run in active env` — wheel-build script now works in either local pixi shell or CI-activated pixi env.
- `feat(pixi): add build-wheel task` — new pixi task wrapping `scripts/build_wheel.sh`.
- `ci: add wheels-macos.yml` — new workflow; macOS jobs invoke `pixi run build-cpp` + `pixi run build-wheel`.
- `ci: strip macos from wheels.yml` — Linux-only workflow, macOS branches moved to new file.

Unblocked by PyPI filesize limit increase to 250 MB.

Spec: `docs/superpowers/specs/2026-04-22-macos-wheels-to-pypi-design.md`
Plan: `docs/superpowers/plans/2026-04-22-macos-wheels-to-pypi.md`

## Test plan

- [ ] PR CI green on all `wheels.yml` jobs (Linux build + test-wheel, py3.9–3.12)
- [ ] PR CI green on all `wheels-macos.yml` jobs (macOS build + test-wheel, py3.9–3.12 / py3.9+3.12)
- [ ] `python-3.12-macos-arm64` artifact: size < 250 MB, 9 plugin factories in `.dylibs/`, YAMLs patched to `@PLUGIN_PATH@`
- [ ] On tag: both workflows' `publish` jobs upload to PyPI, all 8 wheels visible on pypi.org
EOF
)"
```

---

## Task 6: Monitor PR CI + diagnose failures

**Files:**
- None (CI observation)

- [ ] **Step 1: Watch checks**

Run:
```bash
gh pr checks --repo tesseract-robotics/tesseract_nanobind --watch 45
```

Two workflows run in parallel. Linux should stay green (flow unchanged). macOS is the novel path.

- [ ] **Step 2: Triage macOS failures**

If any macOS job fails, pull the failed log:
```bash
RUN_ID=$(gh run list --repo tesseract-robotics/tesseract_nanobind \
  --workflow wheels-macos.yml --branch jf/macos-wheels-to-pypi \
  --limit 1 --json databaseId --jq '.[0].databaseId')
gh run view $RUN_ID --repo tesseract-robotics/tesseract_nanobind --log-failed | tail -200
```

Because the build uses `pixi run build-cpp` + `pixi run build-wheel`, any failure reproduces locally with the same two commands. Hand back to the controller/human to reproduce and fix in the shell script (not the CI YAML) — that way the fix automatically applies everywhere.

---

## Task 7: Merge, validate, release

**Files:**
- None (git + PyPI)

- [ ] **Step 1: Rebase onto main if main advanced**

Run:
```bash
git fetch origin main
git rebase origin/main
```

- [ ] **Step 2: Merge PR rebase-style (linear history)**

Run:
```bash
gh pr merge --repo tesseract-robotics/tesseract_nanobind --rebase --delete-branch 45
```

- [ ] **Step 3: Sync local main**

Run:
```bash
git checkout main
git pull --ff-only
git branch -d jf/macos-wheels-to-pypi 2>/dev/null || true
```

- [ ] **Step 4: Post-merge validation**

Trigger each workflow on main:
```bash
gh workflow run wheels.yml --repo tesseract-robotics/tesseract_nanobind --ref main
gh workflow run wheels-macos.yml --repo tesseract-robotics/tesseract_nanobind --ref main
```

Watch both run to completion. Publish jobs should NOT fire (no tag).

- [ ] **Step 5: Cut release tag**

With explicit user confirmation of the version:
```bash
git tag -a 0.34.1.1 -m "release: macos arm64 wheels via split workflow"
git push origin 0.34.1.1
```

- [ ] **Step 6: Watch publish**

Two tag-triggered runs now in flight — one on `wheels.yml`, one on `wheels-macos.yml`. Both have `publish` jobs gated on `refs/tags/*`. Watch each:
```bash
gh run watch <linux-run-id> --repo tesseract-robotics/tesseract_nanobind
gh run watch <macos-run-id> --repo tesseract-robotics/tesseract_nanobind
```

- [ ] **Step 7: Verify PyPI**

Run:
```bash
curl -s https://pypi.org/pypi/tesseract-robotics-nanobind/0.34.1.1/json | python3 -c "
import sys, json
d = json.load(sys.stdin)
for f in d['urls']:
    print(f\"{f['filename']:70s} {f['size']/1e6:7.1f} MB\")
"
```

Expected: 8 wheels total (4 linux + 4 macos for py3.9–3.12).

- [ ] **Step 8: Clean-env macOS smoke install**

Run on an macOS arm64 machine (the controller's local works):
```bash
python3.12 -m venv /tmp/pypi-smoke
source /tmp/pypi-smoke/bin/activate
pip install --no-cache-dir tesseract-robotics-nanobind==0.34.1.1
tesseract_nanobind_selftest
python -c "from tesseract_robotics import tesseract_common, tesseract_geometry, tesseract_scene_graph; print('OK')"
deactivate
rm -rf /tmp/pypi-smoke
```

Expected: selftest passes, imports succeed.

---

## Rollback

Two-step revert:
1. Delete `.github/workflows/wheels-macos.yml`.
2. `git revert` Tasks 4 (`wheels.yml` strip), 2 (pixi task), 1 (`build_wheel.sh`) in reverse order.
3. If a broken macOS wheel reached PyPI: yank via PyPI web UI.

No database state, no external system coupling.

---

## Self-review notes

Spec coverage:
- Spec §"In scope" bullet 1 (split workflows) → Tasks 3 (create) + 4 (strip).
- Spec §"In scope" bullet 2 (wheels-macos.yml invokes pixi tasks) → Task 3 Step 1.
- Spec §"In scope" bullet 3 (refactor build_wheel.sh) → Task 1.
- Spec §"In scope" bullet 4 (new pixi task) → Task 2.
- Spec §"In scope" bullet 5 (per-workflow publish) → Tasks 3 + 4 (both files have `publish` jobs).
- Spec §"Validation plan" steps 1–7 → Tasks 5, 6, 7.

Placeholder check: no "TBD" / "TODO" / "fill in" / "similar to" references in this plan.

Type consistency: pixi task name `build-wheel` is used consistently in Task 2 (definition) and Task 3 (invocation). Artifact names `python-${python}-linux-x64` and `python-${python}-macos-arm64` match the respective workflow `upload-artifact` and `download-artifact` steps.
