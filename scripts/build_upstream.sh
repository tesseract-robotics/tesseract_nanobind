#!/usr/bin/env bash
# Inner loop for the upstream-main track: incrementally build the upstream/ C++
# submodules and install them into the `upstream` pixi env's prefix, where the
# bindings find them exactly as they find the conda packages on main.
#
# Mirrors the feedstocks' recipe/build.sh (same ${CMAKE_ARGS} from conda's compiler
# activation, same -D flags), but keeps one persistent ninja build dir per package
# under upstream/build/ and compiles through ccache, so an edit in an upstream
# checkout recompiles only what it touches. ninja uses every core by default.
#
# Run via `pixi run build-upstream`.
set -euo pipefail

root="${PIXI_PROJECT_ROOT:?run through pixi: pixi run build-upstream}"
src="$root/upstream"
build="$src/build"

# As upstream's own conda recipe (tesseract#1305): conda's clang adds
# -fvisibility-inlines-hidden, which hides cereal's registry singleton; under Mach-O's
# two-level namespace every dylib then keeps its own registry, and since #1305 moved
# registration into the compiled libs, consumers throw "unregistered polymorphic type".
# Read at first configure only: an existing build dir needs -DCMAKE_CXX_FLAGS="$CXXFLAGS".
if [[ "$OSTYPE" == darwin* ]]; then
    export CXXFLAGS="${CXXFLAGS//-fvisibility-inlines-hidden/}"
fi

# configure on first use (or after `rm -rf upstream/build/<name>`), then build + install
build_pkg() {
    local name=$1 source_dir=$2
    shift 2
    local build_dir="$build/$name"
    if [[ ! -f "$build_dir/build.ninja" ]]; then
        # shellcheck disable=SC2086  # CMAKE_ARGS is a flag list by design
        # conda-build adds the install prefix to CMAKE_ARGS itself; outside it we do.
        # BUILD_WITH_INSTALL_RPATH: no build-tree rpaths, so a repeat install has none to
        # strip (CMake re-runs `install_name_tool -delete_rpath` on up-to-date files,
        # which errors once the first install already removed it).
        cmake -GNinja ${CMAKE_ARGS} \
            -DCMAKE_INSTALL_PREFIX="$CONDA_PREFIX" \
            -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" \
            -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON \
            -DCMAKE_C_COMPILER_LAUNCHER=ccache \
            -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
            -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_SHARED_LIBS=ON \
            -DTESSERACT_ENABLE_TESTING=OFF \
            -DTESSERACT_ENABLE_EXAMPLES=OFF \
            "$@" \
            -S "$source_dir" \
            -B "$build_dir"
    fi
    cmake --build "$build_dir" --target install
}

build_pkg boost_plugin_loader "$src/boost_plugin_loader"
build_pkg tesseract "$src/tesseract"
# trajopt is a set of sibling CMake projects; same order as the trajopt feedstock
for p in trajopt_common trajopt_sco trajopt_ifopt trajopt; do
    build_pkg "trajopt/$p" "$src/trajopt/$p"
done
build_pkg trajopt/trajopt_sqp "$src/trajopt/trajopt_optimizers/trajopt_sqp"
build_pkg tesseract_planning "$src/tesseract_planning"
