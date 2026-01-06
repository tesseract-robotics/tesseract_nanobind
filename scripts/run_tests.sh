#!/bin/bash
# Run all pytests with correct environment
# Usage: ./run_tests.sh [-v|--verbose] [-f|--failed] [-s|--serial] [pytest args...]
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR/.."
cd "$PROJECT_ROOT"

source "$SCRIPT_DIR/env.sh"

# Parse flags
VERBOSE_ARGS=""
FAILED_ARGS=""
PARALLEL=1
REMAINING_ARGS=()
for arg in "$@"; do
  case $arg in
    -v|--verbose)
      VERBOSE_ARGS="-v --tb=long"
      ;;
    -f|--failed)
      FAILED_ARGS="--lf"
      PARALLEL=0  # disable parallel when rerunning failed tests
      ;;
    -s|--serial)
      PARALLEL=0
      ;;
    *)
      REMAINING_ARGS+=("$arg")
      ;;
  esac
done

PARALLEL_ARGS=""
if [ "$PARALLEL" -eq 1 ]; then
  PARALLEL_ARGS="-n auto"
fi

exec pytest tesseract_nanobind/tests --ignore=tesseract_nanobind/tests/benchmarks $PARALLEL_ARGS $VERBOSE_ARGS $FAILED_ARGS "${REMAINING_ARGS[@]}"
