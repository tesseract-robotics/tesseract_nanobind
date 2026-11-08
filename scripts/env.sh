#!/bin/bash
# Source this file to set the tesseract resource env vars for ad-hoc use
# (examples, benchmarks) inside an active pixi/conda environment.
#
# Usage:
#   pixi shell            # or: pixi run ...
#   source scripts/env.sh
#
# Note: tesseract_robotics/__init__.py auto-configures these same paths on first
# import (from $CONDA_PREFIX), so this script is only needed for tooling that reads
# the env vars before importing the package.

# Get the project root (parent of scripts/) — support both bash and zsh
if [ -n "$BASH_SOURCE" ]; then
    _SCRIPTS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
else
    _SCRIPTS_DIR="$( cd "$( dirname "$0" )" && pwd )"
fi
SCRIPT_DIR="$( cd "$_SCRIPTS_DIR/.." && pwd )"

if [ -z "$CONDA_PREFIX" ]; then
    echo "❌ No active pixi/conda environment (CONDA_PREFIX unset)."
    echo "   Run 'pixi shell' (or invoke via 'pixi run ...') first."
    return 1 2>/dev/null || exit 1
fi

# The tesseract C++ libs + bundled data come from the tesseract-robotics conda
# packages installed under $CONDA_PREFIX.
SHARE_DIR="$CONDA_PREFIX/share"

# Qt6 cross-compile fix (conda Qt6 needs this on macOS)
export QT_HOST_PATH=$CONDA_PREFIX

# Tesseract resource paths (support meshes/URDFs live under share/tesseract/support)
export TESSERACT_SUPPORT_DIR="$SHARE_DIR/tesseract/support"
export TESSERACT_RESOURCE_PATH="$SHARE_DIR"

# Task composer config (required for planning examples and tests)
export TESSERACT_TASK_COMPOSER_DIR="$SHARE_DIR/tesseract_planning/task_composer"
export TESSERACT_TASK_COMPOSER_CONFIG_FILE="$TESSERACT_TASK_COMPOSER_DIR/config/task_composer_plugins.yaml"

echo "Environment set up (from \$CONDA_PREFIX=$CONDA_PREFIX):"
echo "  TESSERACT_SUPPORT_DIR: $TESSERACT_SUPPORT_DIR"
echo "  TESSERACT_RESOURCE_PATH: $TESSERACT_RESOURCE_PATH"
echo "  TESSERACT_TASK_COMPOSER_CONFIG_FILE: $TESSERACT_TASK_COMPOSER_CONFIG_FILE"
echo ""
echo "Run tests:    ./scripts/run_tests.sh"
echo "Run examples: python examples/freespace_ompl_example.py"
