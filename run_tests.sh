#!/bin/bash
# Run tesseract tests with module-level isolation
#
# The tesseract C++ library's KinematicsPluginFactory uses global state
# that doesn't reset properly between tests. This causes segfaults when
# multiple plugin factories are created/destroyed in the same process.
#
# This script runs tests in separate pytest invocations to work around
# this architectural limitation.
#
# See: docs/KINEMATICS_PLUGIN_FACTORY_ISSUE.md
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Source environment if available
if [ -f env.sh ]; then
    source env.sh
fi

export TESSERACT_HEADLESS=1
export DYLD_LIBRARY_PATH="${PWD}/ws/install/lib:${CONDA_PREFIX}/lib:${DYLD_LIBRARY_PATH:-}"
export TESSERACT_SUPPORT_DIR="${TESSERACT_SUPPORT_DIR:-${PWD}/ws/src/tesseract/tesseract_support}"
export TESSERACT_RESOURCE_PATH="${TESSERACT_RESOURCE_PATH:-${PWD}/ws/src/tesseract}"
export TESSERACT_TASK_COMPOSER_DIR="${TESSERACT_TASK_COMPOSER_DIR:-${PWD}/ws/src/tesseract_planning/tesseract_task_composer}"
export TESSERACT_TASK_COMPOSER_CONFIG_FILE="${TESSERACT_TASK_COMPOSER_CONFIG_FILE:-${TESSERACT_TASK_COMPOSER_DIR}/config/task_composer_plugins.yaml}"

PYTEST_OPTS="${PYTEST_OPTS:--v --tb=short}"
TESTS_DIR="tesseract_nanobind/tests"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

FAILED=0
PASSED=0

run_tests() {
    local name="$1"
    shift
    echo -e "${YELLOW}>>> $name${NC}"
    if pytest $PYTEST_OPTS "$@" 2>&1; then
        ((PASSED++))
        echo -e "${GREEN}>>> PASSED: $name${NC}\n"
        return 0
    else
        ((FAILED++))
        echo -e "${RED}>>> FAILED: $name${NC}\n"
        return 1
    fi
}

echo "=== tesseract_robotics test suite ==="
echo ""

# Phase 1: Tests that don't use kinematics plugin factories - can run together
echo "=== Phase 1: Core tests (no plugin isolation needed) ==="
run_tests "core" \
    "$TESTS_DIR/tesseract_common/" \
    "$TESTS_DIR/tesseract_geometry/" \
    "$TESTS_DIR/tesseract_command_language/" \
    "$TESTS_DIR/tesseract_scene_graph/" || true

# Phase 2: Collision tests (use Environment but not KinematicsPluginFactory directly)
echo ""
echo "=== Phase 2: Collision tests ==="
run_tests "collision" "$TESTS_DIR/tesseract_collision/" || true

# Phase 3: Environment tests that don't use kinematics groups
echo ""
echo "=== Phase 3: Environment tests (basic) ==="
run_tests "environment_basic" \
    "$TESTS_DIR/tesseract_environment/test_tesseract_environment.py" \
    "$TESTS_DIR/tesseract_environment/test_mesh_material_loading.py" \
    "$TESTS_DIR/tesseract_environment/test_environment_commands.py" || true

# Phase 4: Kinematics tests - MUST be isolated due to plugin factory global state
echo ""
echo "=== Phase 4: Kinematics tests (isolated - plugin factory limitation) ==="
run_tests "kinematics_kdl" "$TESTS_DIR/tesseract_kinematics/test_kdl_kinematics.py" || true
run_tests "kinematics_opw" "$TESTS_DIR/tesseract_kinematics/test_opw_kinematics.py" || true
run_tests "environment_kingroup" "$TESTS_DIR/tesseract_environment/test_tesseract_environment_kingroup.py" || true

# Phase 5: Motion planners - use different plugin configs
echo ""
echo "=== Phase 5: Motion planner tests (isolated) ==="
run_tests "planner_descartes" "$TESTS_DIR/tesseract_motion_planners/test_descartes_planner.py" || true
run_tests "planner_motion" "$TESTS_DIR/tesseract_motion_planners/test_motion_planners.py" || true
run_tests "planner_trajopt" "$TESTS_DIR/tesseract_motion_planners/test_trajopt_planner.py" || true
run_tests "planner_time_optimal" "$TESTS_DIR/tesseract_motion_planners/test_time_optimal_trajectory.py" || true
run_tests "planner_iterative_spline" "$TESTS_DIR/tesseract_motion_planners/test_iterative_spline.py" || true

# Phase 6: Task composer and time parameterization
echo ""
echo "=== Phase 6: Task composer and time param (isolated) ==="
run_tests "task_composer" "$TESTS_DIR/tesseract_task_composer/" || true
run_tests "time_param" "$TESTS_DIR/tesseract_time_parameterization/" || true

# Phase 7: High-level planning API
echo ""
echo "=== Phase 7: Planning API tests (isolated) ==="
run_tests "planning_api" "$TESTS_DIR/tesseract_planning/" || true

# Phase 8: Example tests (subprocess-based only - in-process tests are disabled)
# tests/examples/test_examples.py is skipped due to C++ plugin state corruption issues
# tests/test_examples.py runs examples in isolated subprocesses which works correctly
echo ""
echo "=== Phase 8: Example tests (subprocess-based) ==="
run_tests "examples" "$TESTS_DIR/test_examples.py" || true

# Summary
echo ""
echo "==========================================="
echo "               SUMMARY"
echo "==========================================="
echo -e "${GREEN}Passed: $PASSED${NC}"
echo -e "${RED}Failed: $FAILED${NC}"
echo ""

if [ $FAILED -gt 0 ]; then
    echo -e "${RED}Some tests failed. See above for details.${NC}"
    exit 1
else
    echo -e "${GREEN}All test phases completed!${NC}"
fi
