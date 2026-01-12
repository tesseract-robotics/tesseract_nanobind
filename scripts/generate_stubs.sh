#!/bin/bash
# Generate .pyi stub files from nanobind modules
#
# Usage:
#   ./scripts/generate_stubs.sh          # Generate stubs to src/
#   ./scripts/generate_stubs.sh --check  # Verify stubs are up to date

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
SRC_DIR="$PROJECT_ROOT/src/tesseract_robotics"

# Module name -> subdir mapping
# Format: "full.module.name:subdir"
MODULES=(
    "tesseract_robotics.ifopt._ifopt:ifopt"
    "tesseract_robotics.tesseract_collision._tesseract_collision:tesseract_collision"
    "tesseract_robotics.tesseract_command_language._tesseract_command_language:tesseract_command_language"
    "tesseract_robotics.tesseract_common._tesseract_common:tesseract_common"
    "tesseract_robotics.tesseract_environment._tesseract_environment:tesseract_environment"
    "tesseract_robotics.tesseract_geometry._tesseract_geometry:tesseract_geometry"
    "tesseract_robotics.tesseract_kinematics._tesseract_kinematics:tesseract_kinematics"
    "tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners:tesseract_motion_planners"
    "tesseract_robotics.tesseract_motion_planners_descartes._tesseract_motion_planners_descartes:tesseract_motion_planners_descartes"
    "tesseract_robotics.tesseract_motion_planners_ompl._tesseract_motion_planners_ompl:tesseract_motion_planners_ompl"
    "tesseract_robotics.tesseract_motion_planners_simple._tesseract_motion_planners_simple:tesseract_motion_planners_simple"
    "tesseract_robotics.tesseract_motion_planners_trajopt._tesseract_motion_planners_trajopt:tesseract_motion_planners_trajopt"
    "tesseract_robotics.tesseract_motion_planners_trajopt_ifopt._tesseract_motion_planners_trajopt_ifopt:tesseract_motion_planners_trajopt_ifopt"
    "tesseract_robotics.tesseract_scene_graph._tesseract_scene_graph:tesseract_scene_graph"
    "tesseract_robotics.tesseract_srdf._tesseract_srdf:tesseract_srdf"
    "tesseract_robotics.tesseract_state_solver._tesseract_state_solver:tesseract_state_solver"
    "tesseract_robotics.tesseract_task_composer._tesseract_task_composer:tesseract_task_composer"
    "tesseract_robotics.tesseract_time_parameterization._tesseract_time_parameterization:tesseract_time_parameterization"
    "tesseract_robotics.tesseract_urdf._tesseract_urdf:tesseract_urdf"
    "tesseract_robotics.trajopt_ifopt._trajopt_ifopt:trajopt_ifopt"
    "tesseract_robotics.trajopt_sqp._trajopt_sqp:trajopt_sqp"
)

# Pattern file for type cleanup (if exists)
PATTERN_FILE="$PROJECT_ROOT/stubs/patterns.txt"
PATTERN_ARG=""
if [ -f "$PATTERN_FILE" ]; then
    PATTERN_ARG="-p $PATTERN_FILE"
fi

echo "Generating stubs for ${#MODULES[@]} modules..."

# Generate each module to its correct subdirectory
for entry in "${MODULES[@]}"; do
    MODULE="${entry%%:*}"
    SUBDIR="${entry##*:}"
    STUB_NAME="${MODULE##*.}"  # e.g., _tesseract_kinematics
    OUTPUT_DIR="$SRC_DIR/$SUBDIR"
    OUTPUT_FILE="$OUTPUT_DIR/${STUB_NAME}.pyi"

    echo "  $MODULE -> $SUBDIR/"

    TRAJOPT_LOG_THRESH=ERROR python -m nanobind.stubgen \
        -m "$MODULE" \
        $PATTERN_ARG \
        -o "$OUTPUT_FILE" \
        -q 2>&1 | grep -v "^You can set logging" || true
done

# Create py.typed marker
touch "$SRC_DIR/py.typed"

echo ""
echo "Done. Generated ${#MODULES[@]} stub files."
echo ""
echo "Generated files:"
find "$SRC_DIR" -name "*.pyi" -type f 2>/dev/null | sort | head -25
