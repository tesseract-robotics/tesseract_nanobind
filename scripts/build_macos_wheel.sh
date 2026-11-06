#!/bin/zsh
# Build wheel (macOS)
#
# Invoked by `pixi run build-wheel` on macOS (the osx-arm64 target overrides the
# base build-wheel task, which runs build_linux_wheel.sh). Also used by
# .github/workflows/wheels-macos.yml.
#
# Usage:
#   ./build_macos_wheel.sh        # Full portable wheel, for distribution
#   ./build_macos_wheel.sh --dev  # Fast dev build, no repair (only works in current env)
set -e

SCRIPT_DIR="${0:a:h}"
PROJECT_ROOT="${SCRIPT_DIR:h}"

DEV_MODE=false
if [[ "$1" == "--dev" ]]; then
    DEV_MODE=true
fi

cd "$PROJECT_ROOT"

# Tag and build for macOS 14.0, the tag of every published macOS wheel (#156). Unset,
# scikit-build-core uses the host's macOS: a local build on 15 tagged 15.0. The bundled
# conda-forge dylibs require 11.0.
export MACOSX_DEPLOYMENT_TARGET=14.0

# The tesseract C++ libs + plugin factories are provided by the tesseract-robotics
# conda packages in the active env (no more colcon ws/install tree).
LIB_DIR="$CONDA_PREFIX/lib"

# Set library paths
export DYLD_LIBRARY_PATH="$CONDA_PREFIX/lib"
export CMAKE_PREFIX_PATH="$CONDA_PREFIX"

if $DEV_MODE; then
    echo "Building dev wheel (no repair)..."
    rm -rf dist/

    # Fix plugin rpaths on macOS (so they work without DYLD_LIBRARY_PATH)
    if [[ "$OSTYPE" == "darwin"* ]]; then
        echo "Fixing plugin factory rpaths..."
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

rm -rf dist/ wheelhouse/

echo "Building wheel..."
pip wheel . -w dist/ --no-build-isolation

echo "Repairing wheel..."
python "$SCRIPT_DIR/repair_macos_wheel.py" dist/tesseract*.whl --env-lib "$LIB_DIR" --out wheelhouse

echo ""
echo "Portable wheel: wheelhouse/"
echo "Install: uv pip install wheelhouse/tesseract*.whl"
