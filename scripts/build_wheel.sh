#!/bin/zsh
# Build wheel (macOS)
# Usage:
#   ./build_wheel.sh        # Full portable wheel with delocate (slow, for distribution)
#   ./build_wheel.sh --dev  # Fast dev build, no delocate (only works in current env)
set -e

SCRIPT_DIR="${0:a:h}"
PROJECT_ROOT="$SCRIPT_DIR/.."

DEV_MODE=false
if [[ "$1" == "--dev" ]]; then
    DEV_MODE=true
fi

cd "$PROJECT_ROOT"

# Set library paths
export DYLD_LIBRARY_PATH="$PROJECT_ROOT/ws/install/lib:$CONDA_PREFIX/lib"
export CMAKE_PREFIX_PATH="$CONDA_PREFIX:$PROJECT_ROOT/ws/install"

if $DEV_MODE; then
    echo "Building dev wheel (no delocate)..."
    rm -rf dist/

    # Fix plugin rpaths on macOS (so they work without DYLD_LIBRARY_PATH)
    if [[ "$OSTYPE" == "darwin"* ]]; then
        echo "Fixing plugin factory rpaths..."
        LIB_DIR="$PROJECT_ROOT/ws/install/lib"
        for factory in "$LIB_DIR"/*_factor*.dylib; do
            if [[ -f "$factory" ]]; then
                if ! otool -l "$factory" 2>/dev/null | grep -q LC_RPATH; then
                    install_name_tool -add_rpath "$LIB_DIR" "$factory" 2>/dev/null || true
                    echo "  Fixed: $(basename $factory)"
                fi
            fi
        done
    fi

    pip install setuptools-scm
    pip wheel . -w dist/ --no-build-isolation
    echo ""
    echo "Dev wheel: dist/"
    echo "Install: pip install dist/tesseract*.whl"
    echo "Note: only works in current conda env"
    exit 0
fi

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

echo "Running delocate..."
mkdir -p wheelhouse
delocate-wheel -w wheelhouse -v dist/tesseract*.whl

# Add plugin factories (runtime-loaded, not caught by delocate)
echo "Adding plugin factories..."
WHEEL_FILE=$(ls wheelhouse/tesseract*.whl)
WHEEL_DIR=$(mktemp -d)
unzip -q "$WHEEL_FILE" -d "$WHEEL_DIR"

PLUGINS=(
    libtesseract_collision_bullet_factories.dylib
    libtesseract_collision_fcl_factories.dylib
    libtesseract_kinematics_core_factories.dylib
    libtesseract_kinematics_kdl_factories.dylib
    libtesseract_kinematics_opw_factory.dylib
    libtesseract_kinematics_ur_factory.dylib
    libtesseract_task_composer_factories.dylib
    libtesseract_task_composer_planning_factories.dylib
    libtesseract_task_composer_taskflow_factories.dylib
)

for plugin in "${PLUGINS[@]}"; do
    if [[ -f "$PROJECT_ROOT/ws/install/lib/$plugin" ]]; then
        cp "$PROJECT_ROOT/ws/install/lib/$plugin" "$WHEEL_DIR/tesseract_robotics/.dylibs/"
        echo "  Added: $plugin"
    fi
done

# Rewrite @rpath/libX.dylib refs to @loader_path/libX.dylib on all plugins + their transitive
# deps that we need to bundle. Mirrors what patchelf does for Linux — no delocate dance, no
# SameFileError edge cases.
echo "Resolving plugin transitive deps + rewriting install_names..."
DYLIBS_DIR="$WHEEL_DIR/tesseract_robotics/.dylibs"

# Work queue of dylibs to process (plugins + any of their deps we still need to fetch)
for plugin in "${PLUGINS[@]}"; do
    [[ -f "$DYLIBS_DIR/$plugin" ]] || continue
    queue+=("$DYLIBS_DIR/$plugin")
done

processed=()
while (( ${#queue[@]} )); do
    current="${queue[1]}"
    queue=("${queue[@]:1}")  # zsh: skip first element (offset counts elements to skip)

    # Skip if already processed
    if [[ " ${processed[*]} " == *" $current "* ]]; then
        continue
    fi
    processed+=("$current")

    # Rewrite this dylib's own install id (if @rpath) to @loader_path
    current_id=$(otool -D "$current" | tail -1)
    if [[ "$current_id" == @rpath/* ]]; then
        install_name_tool -id "@loader_path/${current_id#@rpath/}" "$current" 2>/dev/null || true
    fi

    # Walk @rpath/ deps
    while read -r dep; do
        [[ -z "$dep" ]] && continue
        base="${dep#@rpath/}"
        # Rewrite the reference
        install_name_tool -change "$dep" "@loader_path/$base" "$current" 2>/dev/null || true
        # Ensure the target dep is bundled in .dylibs/
        if [[ ! -f "$DYLIBS_DIR/$base" ]]; then
            for src_dir in "$PROJECT_ROOT/ws/install/lib" "$CONDA_PREFIX/lib"; do
                if [[ -f "$src_dir/$base" ]]; then
                    cp "$src_dir/$base" "$DYLIBS_DIR/$base"
                    echo "  Copied dep: $base"
                    queue+=("$DYLIBS_DIR/$base")
                    break
                fi
            done
        fi
    done < <(otool -L "$current" | tail -n +2 | awk '{print $1}' | grep '^@rpath/' || true)
done

# Assert every plugin factory has zero @rpath/ deps — catches the class of bugs
# where install_name rewriting misses one. Fails the build immediately instead
# of surfacing 20 min later in test-wheel.
echo "Asserting no @rpath/ remains in plugin factories..."
fail=0
for f in "$DYLIBS_DIR"/*_factor*.dylib; do
    if otool -L "$f" 2>&1 | grep -q '@rpath/'; then
        echo "  BUG: $(basename $f) still has @rpath/ deps"
        otool -L "$f" | grep '@rpath/' | sed 's/^/    /'
        fail=1
    fi
done
if (( fail )); then
    echo "❌ install_name rewriting incomplete — see above"
    exit 1
fi
echo "  all plugin factories cleanly resolved via @loader_path/"

# Patch task_composer_config YAMLs (resolved at runtime by __init__.py)
echo "Patching task composer configs..."
for yaml_file in "$WHEEL_DIR/tesseract_robotics/data/task_composer_config"/*.yaml; do
    if [[ -f "$yaml_file" ]] && grep -q '/usr/local/lib' "$yaml_file"; then
        sed -i '' 's|/usr/local/lib|"@PLUGIN_PATH@"|g' "$yaml_file"
        echo "  Patched: $(basename $yaml_file)"
    fi
done

# Remove search_paths from robot YAMLs (forces use of env vars set by __init__.py)
echo "Removing hardcoded search_paths from robot YAMLs..."
find "$WHEEL_DIR/tesseract_robotics/data/tesseract_support" -name "*.yaml" -type f | while read yaml_file; do
    if grep -q 'search_paths:' "$yaml_file"; then
        # Remove search_paths: line and following lines with - /path
        sed -i '' '/search_paths:/d; /^[[:space:]]*- \/.*$/d' "$yaml_file"
        echo "  Removed search_paths from: $(basename $yaml_file)"
    fi
done

rm "$WHEEL_FILE"
cd "$WHEEL_DIR"
zip -rq "$PROJECT_ROOT/wheelhouse/$(basename $WHEEL_FILE)" .
cd "$PROJECT_ROOT"
rm -rf "$WHEEL_DIR"

echo ""
echo "Portable wheel: wheelhouse/"
echo "Install: pip install wheelhouse/tesseract*.whl"
