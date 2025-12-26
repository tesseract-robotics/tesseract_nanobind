#!/bin/bash
# Run all pytests with correct environment
# Usage: ./run_tests.sh [-v|--verbose] [pytest args...]
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR/.."
cd "$PROJECT_ROOT"

source "$SCRIPT_DIR/env.sh"

# Parse verbose flag
VERBOSE_ARGS=""
REMAINING_ARGS=()
for arg in "$@"; do
  case $arg in
    -v|--verbose)
      VERBOSE_ARGS="-v --tb=long"
      ;;
    *)
      REMAINING_ARGS+=("$arg")
      ;;
  esac
done

exec pytest tesseract_nanobind/tests --ignore=tesseract_nanobind/tests/benchmarks -n auto $VERBOSE_ARGS "${REMAINING_ARGS[@]}"
