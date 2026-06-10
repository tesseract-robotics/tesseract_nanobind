#!/usr/bin/env bash
# Build wheel (Linux)
#
# Invoked by `pixi run build-wheel` on Linux (the base build-wheel task; macOS
# overrides it to build_macos_wheel.sh via the osx-arm64 target). Also used by
# .github/workflows/wheels-linux.yml.
#
# Usage:
#   ./build_linux_wheel.sh        # Full portable manylinux wheel (patchelf + plugins + YAML patch)
#   ./build_linux_wheel.sh --dev  # Fast dev build, no bundling (only works in current env)
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

DEV_MODE=false
if [[ "$1" == "--dev" ]]; then
    DEV_MODE=true
fi

cd "$PROJECT_ROOT"

# The tesseract C++ libs + plugin factories are provided by the tesseract-robotics
# conda packages, installed under $CONDA_PREFIX/lib (no more colcon ws/install tree).
LIB_DIR="$CONDA_PREFIX/lib"
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib"
export CMAKE_PREFIX_PATH="$CONDA_PREFIX"

if $DEV_MODE; then
    echo "Building dev wheel (no bundling)..."
    rm -rf dist/
    pip install setuptools-scm
    CMAKE_ARGS="-DCMAKE_POLICY_VERSION_MINIMUM=3.5" \
        pip wheel . -w dist/ --no-build-isolation
    echo ""
    echo "Dev wheel: dist/"
    echo "Install: pip install dist/tesseract*.whl"
    echo "Note: only works in current env"
    exit 0
fi

# Full portable build in the active env (pixi locally, pixi-activated in CI).
echo "Building portable manylinux wheel in active env: ${CONDA_PREFIX:-NO ENV}"
if [[ -z "$CONDA_PREFIX" ]]; then
    echo "❌ No active pixi/conda env. Run via 'pixi shell' or 'pixi run build-wheel'."
    exit 1
fi

pip install setuptools-scm

rm -rf dist/ wheelhouse/

echo "Building wheel..."
CMAKE_ARGS="-DCMAKE_POLICY_VERSION_MINIMUM=3.5" \
    pip wheel . -w dist/ --no-build-isolation

WHEEL_FILE=$(ls dist/tesseract*.whl)
WHEEL_DIR=$(mktemp -d)
unzip -q "$WHEEL_FILE" -d "$WHEEL_DIR"
PKG_DIR="$WHEEL_DIR/tesseract_robotics"

# Plugin factories are runtime-loaded (dlopen), so the linker never sees them —
# copy them into the package root explicitly.
echo "Adding plugin factories..."
PLUGINS=(
    libtesseract_collision_bullet_factories.so
    libtesseract_collision_fcl_factories.so
    libtesseract_kinematics_factories.so
    libtesseract_kinematics_kdl_factories.so
    libtesseract_kinematics_opw_factory.so
    libtesseract_kinematics_ur_factory.so
    libtesseract_task_composer_factories.so
    libtesseract_task_composer_planning_factories.so
    libtesseract_task_composer_taskflow_factories.so
)
for plugin in "${PLUGINS[@]}"; do
    if [[ -f "$LIB_DIR/$plugin" ]]; then
        cp "$LIB_DIR/$plugin" "$PKG_DIR/"
        echo "  Added: $plugin"
    fi
done

# Bundle the tesseract libs + every shared-lib dependency into the package root.
#
# Unlike the old colcon ws/install/lib (a clean tesseract-only tree we could bulk
# copy), $CONDA_PREFIX/lib holds the entire conda env, so we copy dependency-driven
# instead: ldd is recursive, so one pass over the objects actually loaded at runtime
# — the extension modules (subpackage *.so) plus the dlopen'd plugin factories —
# yields the full transitive set of libs they need (tesseract_*, boost, ompl, fcl,
# vtk via urdf→pcl_io, …). This mirrors what delocate does automatically on macOS.
echo "Bundling tesseract libs + transitive deps..."
seeds=()
while IFS= read -r ext; do seeds+=("$ext"); done < <(find "$PKG_DIR" -name "*.so" -type f)
for plugin in "${PLUGINS[@]}"; do
    [[ -f "$PKG_DIR/$plugin" ]] && seeds+=("$PKG_DIR/$plugin")
done

deps_file=$(mktemp)
for so in "${seeds[@]}"; do
    ldd "$so" 2>/dev/null | grep "=>" | awk '{print $3}' | grep -v "^$" >> "$deps_file" || true
done
sort -u "$deps_file" | while read -r dep; do
    [[ -f "$dep" ]] || continue
    # Skip OS libs and the interpreter's libpython (provided by the host at runtime).
    case "$dep" in
        /lib/*|/lib64/*|/usr/lib/*|/usr/lib64/*) continue ;;
    esac
    base=$(basename "$dep")
    case "$base" in
        libpython*) continue ;;
        # gcc runtime is manylinux-whitelisted: it MUST come from the host/env,
        # never the wheel. Bundling libstdc++ put two copies in every
        # numpy-importing process; their STB_GNU_UNIQUE locale statics unify
        # across copies and std::regex corrupts the heap during env.init.
        # See docs/developer/linux-wheels.md (gh-119).
        libstdc++*|libgcc_s*) continue ;;
    esac
    if [[ ! -f "$PKG_DIR/$base" ]]; then
        # pip dereferences symlinks on extract, so ship the real file (cp -L) under
        # the SONAME it's referenced by.
        cp -L "$dep" "$PKG_DIR/$base"
        echo "  Bundled: $base"
    fi
done
rm "$deps_file"

# Relink so libs find their siblings in the package root. Top-level libs and
# extensions resolve via $ORIGIN; extensions nested in subpackages need
# $ORIGIN/.. to reach the package root.
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

# Patch task_composer_config YAMLs (resolved at runtime by __init__.py).
echo "Patching task composer configs..."
for yaml_file in "$PKG_DIR/data/task_composer_config"/*.yaml; do
    if [[ -f "$yaml_file" ]] && grep -q '/usr/local/lib' "$yaml_file"; then
        sed -i 's|/usr/local/lib|"@PLUGIN_PATH@"|g' "$yaml_file"
        echo "  Patched: $(basename $yaml_file)"
    fi
done

# Remove search_paths from robot YAMLs (forces use of env vars set by __init__.py).
echo "Removing hardcoded search_paths from robot YAMLs..."
find "$PKG_DIR/data/tesseract/support" -name "*.yaml" -type f | while read yaml_file; do
    if grep -q 'search_paths:' "$yaml_file"; then
        sed -i '/search_paths:/d; /^[[:space:]]*- \/.*$/d' "$yaml_file"
        echo "  Removed search_paths from: $(basename $yaml_file)"
    fi
done

# Surface any unresolved deps (resolved relative to $ORIGIN now that rpaths are set).
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

# Repack as a manylinux wheel.
mkdir -p "$PROJECT_ROOT/wheelhouse"
WHEEL_NAME=$(basename "$WHEEL_FILE" | sed -E 's/linux_(x86_64|aarch64)/manylinux_2_35_\1/')
cd "$WHEEL_DIR"
zip -rq "$PROJECT_ROOT/wheelhouse/$WHEEL_NAME" .
cd "$PROJECT_ROOT"
rm -rf "$WHEEL_DIR"

echo ""
echo "Portable wheel: wheelhouse/$WHEEL_NAME"
echo "  $(unzip -l "$PROJECT_ROOT/wheelhouse/$WHEEL_NAME" | grep -c '\.so') shared libraries bundled"
echo "Install: pip install wheelhouse/tesseract*.whl"
