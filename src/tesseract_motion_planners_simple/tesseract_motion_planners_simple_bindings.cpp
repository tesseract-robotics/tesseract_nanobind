/**
 * @file tesseract_motion_planners_simple_bindings.cpp
 * @brief nanobind bindings for tesseract_motion_planners simple
 */

#include "tesseract_nb.h"
#include "tesseract_nb_yaml.h"

#include <tesseract/common/eigen_types.h>

// interpolate() returns, and getInterpolatedInstructions() takes, the opaque VectorIsometry3d
// bound in tesseract_common.
NB_MAKE_OPAQUE(tesseract::common::VectorIsometry3d)

// tesseract_motion_planners core (for PlannerRequest/Response)
#include <tesseract/motion_planners/types.h>

// tesseract_motion_planners simple
#include <tesseract/motion_planners/simple/interpolation.h>
#include <tesseract/motion_planners/simple/simple_motion_planner.h>
#include <tesseract/motion_planners/simple/profile/simple_planner_profile.h>
#include <tesseract/motion_planners/simple/profile/simple_planner_fixed_size_move_profile.h>
#include <tesseract/motion_planners/simple/profile/simple_planner_fixed_size_assign_move_profile.h>
#include <tesseract/motion_planners/simple/profile/simple_planner_fixed_size_assign_no_ik_move_profile.h>
#include <tesseract/motion_planners/simple/profile/simple_planner_lvs_move_profile.h>
#include <tesseract/motion_planners/simple/profile/simple_planner_lvs_no_ik_move_profile.h>
#include <tesseract/motion_planners/simple/profile/simple_planner_lvs_assign_move_profile.h>
#include <tesseract/motion_planners/simple/profile/simple_planner_lvs_assign_no_ik_move_profile.h>

// tesseract_common (Profile/ProfileDictionary moved here in 0.33)
#include <tesseract/common/profile.h>
#include <tesseract/common/profile_dictionary.h>

// tesseract_environment
#include <tesseract/environment/environment.h>

// tesseract_command_language
#include <tesseract/command_language/composite_instruction.h>
#include <tesseract/command_language/poly/instruction_poly.h>
#include <tesseract/command_language/poly/move_instruction_poly.h>
#include <tesseract/command_language/poly/waypoint_poly.h>

namespace tp = tesseract::motion_planners;
namespace tcl = tesseract::command_language;
namespace te = tesseract::environment;
namespace tc = tesseract::common;

NB_MODULE(_tesseract_motion_planners_simple, m) {
    m.doc() = "tesseract_motion_planners_simple Python bindings";

    // Modules defining the types this one names, imported first so the stubs resolve them.
    nb::module_::import_("tesseract_robotics.tesseract_common._tesseract_common");
    nb::module_::import_("tesseract_robotics.tesseract_environment._tesseract_environment");
    // MotionPlanner base
    nb::module_::import_("tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners");
    // Profile base and the instruction types (Profile is re-exported from tesseract_common)
    nb::module_::import_("tesseract_robotics.tesseract_command_language._tesseract_command_language");

    // ========== SimpleMotionPlanner ==========
    // getName/solve/terminate/clear/clone come from MotionPlanner (virtual dispatch).
    nb::class_<tp::SimpleMotionPlanner, tp::MotionPlanner>(m, "SimpleMotionPlanner")
        .def(nb::init<std::string>(), "name"_a = "SIMPLE");

    // ========== generateInterpolatedProgram ==========
    m.def("generateInterpolatedProgram",
          [](const tcl::CompositeInstruction& instructions,
             const std::shared_ptr<const te::Environment>& env,
             double state_lvs,
             double translation_lvs,
             double rotation_lvs,
             int min_steps) {
              return tp::generateInterpolatedProgram(instructions, env, state_lvs, translation_lvs, rotation_lvs, min_steps);
          },
          "instructions"_a,
          "env"_a,
          "state_longest_valid_segment_length"_a = 5 * M_PI / 180,
          "translation_longest_valid_segment_length"_a = 0.15,
          "rotation_longest_valid_segment_length"_a = 5 * M_PI / 180,
          "min_steps"_a = 1,
          "Generate an interpolated program from a composite instruction");

    // ========== Interpolation (interpolation.h) ==========
    m.def("interpolate",
          nb::overload_cast<const Eigen::Isometry3d&, const Eigen::Isometry3d&, long>(&tp::interpolate),
          "start"_a, "stop"_a, "steps"_a,
          "Poses from `start` to `stop`: `steps + 1` of them, translation lerped, rotation slerped.");
    m.def("interpolate",
          nb::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&, const Eigen::Ref<const Eigen::VectorXd>&, long>(
              &tp::interpolate),
          "start"_a, "stop"_a, "steps"_a,
          "Joint states from `start` to `stop`, one per column: `steps + 1` columns.");
    m.def("interpolate_waypoint", &tp::interpolate_waypoint, "start"_a, "stop"_a, "steps"_a,
          "Waypoints from `start` to `stop`: `steps + 1` of them, of the start's type.");
    m.def("getInterpolatedInstructions",
          nb::overload_cast<const std::vector<std::string>&, const Eigen::MatrixXd&, const tcl::MoveInstructionPoly&>(
              &tp::getInterpolatedInstructions),
          "joint_names"_a, "states"_a, "base_instruction"_a,
          "Move instructions for the columns of `states` after the first, each a child of "
          "`base_instruction`; the last is `base_instruction` itself.");
    m.def("getInterpolatedInstructions",
          nb::overload_cast<const tc::VectorIsometry3d&, const std::vector<std::string>&, const Eigen::MatrixXd&,
                            const tcl::MoveInstructionPoly&>(&tp::getInterpolatedInstructions),
          "poses"_a, "joint_names"_a, "states"_a, "base_instruction"_a,
          "Cartesian move instructions for the columns of `states` after the first, seeded with "
          "those states; the last is `base_instruction` itself.");

    // ========== SimplePlannerMoveProfile (abstract base) ==========
    nb::class_<tp::SimplePlannerMoveProfile, tc::Profile>(m, "SimplePlannerMoveProfile")
        .def_static("createKey", &tc::Profile::createKey<tp::SimplePlannerMoveProfile>,
                    "The key every simple-planner move profile registers under.")
        .def("generate", &tp::SimplePlannerMoveProfile::generate,
             "prev_instruction"_a, "prev_seed"_a, "base_instruction"_a, "next_instruction"_a, "env"_a,
             "global_manip_info"_a,
             "The move instructions the simple planner emits for `base_instruction`, ending with it. "
             "`next_instruction` is a null InstructionPoly for the last move.",
             nb::call_guard<nb::gil_scoped_release>());

    // ========== SimplePlannerCompositeProfile ==========
    nb::class_<tp::SimplePlannerCompositeProfile, tc::Profile>(m, "SimplePlannerCompositeProfile")
        .def(nb::init<>())
        .def_static("createKey", &tc::Profile::createKey<tp::SimplePlannerCompositeProfile>,
                    "The key this profile type registers under in a ProfileDictionary.");

    // ========== SimplePlannerFixedSizeMoveProfile ==========
    auto py_FixedSizeMoveProfile = nb::class_<tp::SimplePlannerFixedSizeMoveProfile, tp::SimplePlannerMoveProfile>(m, "SimplePlannerFixedSizeMoveProfile");
    py_FixedSizeMoveProfile
        .def(nb::init<int, int>(), "freespace_steps"_a = 10, "linear_steps"_a = 10)
        .def_rw("freespace_steps", &tp::SimplePlannerFixedSizeMoveProfile::freespace_steps)
        .def_rw("linear_steps", &tp::SimplePlannerFixedSizeMoveProfile::linear_steps);
    bind_value_equality(py_FixedSizeMoveProfile);
    bind_from_yaml(py_FixedSizeMoveProfile);

    // ========== SimplePlannerFixedSizeAssignMoveProfile ==========
    auto py_FixedSizeAssignMoveProfile = nb::class_<tp::SimplePlannerFixedSizeAssignMoveProfile, tp::SimplePlannerMoveProfile>(m, "SimplePlannerFixedSizeAssignMoveProfile");
    py_FixedSizeAssignMoveProfile
        .def(nb::init<int, int>(), "freespace_steps"_a = 10, "linear_steps"_a = 10)
        .def_rw("freespace_steps", &tp::SimplePlannerFixedSizeAssignMoveProfile::freespace_steps)
        .def_rw("linear_steps", &tp::SimplePlannerFixedSizeAssignMoveProfile::linear_steps);
    bind_value_equality(py_FixedSizeAssignMoveProfile);
    bind_from_yaml(py_FixedSizeAssignMoveProfile);

    // ========== SimplePlannerFixedSizeAssignNoIKMoveProfile ==========
    auto py_FixedSizeAssignNoIKMoveProfile = nb::class_<tp::SimplePlannerFixedSizeAssignNoIKMoveProfile, tp::SimplePlannerMoveProfile>(m, "SimplePlannerFixedSizeAssignNoIKMoveProfile");
    py_FixedSizeAssignNoIKMoveProfile
        .def(nb::init<int, int>(), "freespace_steps"_a = 10, "linear_steps"_a = 10)
        .def_rw("freespace_steps", &tp::SimplePlannerFixedSizeAssignNoIKMoveProfile::freespace_steps)
        .def_rw("linear_steps", &tp::SimplePlannerFixedSizeAssignNoIKMoveProfile::linear_steps);
    bind_value_equality(py_FixedSizeAssignNoIKMoveProfile);
    bind_from_yaml(py_FixedSizeAssignNoIKMoveProfile);

    // ========== SimplePlannerLVSMoveProfile ==========
    auto py_LVSMoveProfile = nb::class_<tp::SimplePlannerLVSMoveProfile, tp::SimplePlannerMoveProfile>(m, "SimplePlannerLVSMoveProfile");
    py_LVSMoveProfile
        .def(nb::init<double, double, double, int, int>(),
             "state_longest_valid_segment_length"_a = 5 * M_PI / 180,
             "translation_longest_valid_segment_length"_a = 0.1,
             "rotation_longest_valid_segment_length"_a = 5 * M_PI / 180,
             "min_steps"_a = 1,
             "max_steps"_a = std::numeric_limits<int>::max())
        .def_rw("state_longest_valid_segment_length", &tp::SimplePlannerLVSMoveProfile::state_longest_valid_segment_length)
        .def_rw("translation_longest_valid_segment_length", &tp::SimplePlannerLVSMoveProfile::translation_longest_valid_segment_length)
        .def_rw("rotation_longest_valid_segment_length", &tp::SimplePlannerLVSMoveProfile::rotation_longest_valid_segment_length)
        .def_rw("min_steps", &tp::SimplePlannerLVSMoveProfile::min_steps)
        .def_rw("max_steps", &tp::SimplePlannerLVSMoveProfile::max_steps);
    bind_value_equality(py_LVSMoveProfile);
    bind_from_yaml(py_LVSMoveProfile);

    // ========== SimplePlannerLVSNoIKMoveProfile ==========
    auto py_LVSNoIKMoveProfile = nb::class_<tp::SimplePlannerLVSNoIKMoveProfile, tp::SimplePlannerMoveProfile>(m, "SimplePlannerLVSNoIKMoveProfile");
    py_LVSNoIKMoveProfile
        .def(nb::init<double, double, double, int, int>(),
             "state_longest_valid_segment_length"_a = 5 * M_PI / 180,
             "translation_longest_valid_segment_length"_a = 0.1,
             "rotation_longest_valid_segment_length"_a = 5 * M_PI / 180,
             "min_steps"_a = 1,
             "max_steps"_a = std::numeric_limits<int>::max())
        .def_rw("state_longest_valid_segment_length", &tp::SimplePlannerLVSNoIKMoveProfile::state_longest_valid_segment_length)
        .def_rw("translation_longest_valid_segment_length", &tp::SimplePlannerLVSNoIKMoveProfile::translation_longest_valid_segment_length)
        .def_rw("rotation_longest_valid_segment_length", &tp::SimplePlannerLVSNoIKMoveProfile::rotation_longest_valid_segment_length)
        .def_rw("min_steps", &tp::SimplePlannerLVSNoIKMoveProfile::min_steps)
        .def_rw("max_steps", &tp::SimplePlannerLVSNoIKMoveProfile::max_steps);
    bind_value_equality(py_LVSNoIKMoveProfile);
    bind_from_yaml(py_LVSNoIKMoveProfile);

    // ========== SimplePlannerLVSAssignMoveProfile ==========
    auto py_LVSAssignMoveProfile = nb::class_<tp::SimplePlannerLVSAssignMoveProfile, tp::SimplePlannerMoveProfile>(m, "SimplePlannerLVSAssignMoveProfile");
    py_LVSAssignMoveProfile
        .def(nb::init<double, double, double, int, int>(),
             "state_longest_valid_segment_length"_a = 5 * M_PI / 180,
             "translation_longest_valid_segment_length"_a = 0.1,
             "rotation_longest_valid_segment_length"_a = 5 * M_PI / 180,
             "min_steps"_a = 1,
             "max_steps"_a = std::numeric_limits<int>::max())
        .def_rw("state_longest_valid_segment_length", &tp::SimplePlannerLVSAssignMoveProfile::state_longest_valid_segment_length)
        .def_rw("translation_longest_valid_segment_length", &tp::SimplePlannerLVSAssignMoveProfile::translation_longest_valid_segment_length)
        .def_rw("rotation_longest_valid_segment_length", &tp::SimplePlannerLVSAssignMoveProfile::rotation_longest_valid_segment_length)
        .def_rw("min_steps", &tp::SimplePlannerLVSAssignMoveProfile::min_steps)
        .def_rw("max_steps", &tp::SimplePlannerLVSAssignMoveProfile::max_steps);
    bind_value_equality(py_LVSAssignMoveProfile);
    bind_from_yaml(py_LVSAssignMoveProfile);

    // ========== SimplePlannerLVSAssignNoIKMoveProfile ==========
    auto py_LVSAssignNoIKMoveProfile = nb::class_<tp::SimplePlannerLVSAssignNoIKMoveProfile, tp::SimplePlannerMoveProfile>(m, "SimplePlannerLVSAssignNoIKMoveProfile");
    py_LVSAssignNoIKMoveProfile
        .def(nb::init<double, double, double, int, int>(),
             "state_longest_valid_segment_length"_a = 5 * M_PI / 180,
             "translation_longest_valid_segment_length"_a = 0.1,
             "rotation_longest_valid_segment_length"_a = 5 * M_PI / 180,
             "min_steps"_a = 1,
             "max_steps"_a = std::numeric_limits<int>::max())
        .def_rw("state_longest_valid_segment_length", &tp::SimplePlannerLVSAssignNoIKMoveProfile::state_longest_valid_segment_length)
        .def_rw("translation_longest_valid_segment_length", &tp::SimplePlannerLVSAssignNoIKMoveProfile::translation_longest_valid_segment_length)
        .def_rw("rotation_longest_valid_segment_length", &tp::SimplePlannerLVSAssignNoIKMoveProfile::rotation_longest_valid_segment_length)
        .def_rw("min_steps", &tp::SimplePlannerLVSAssignNoIKMoveProfile::min_steps)
        .def_rw("max_steps", &tp::SimplePlannerLVSAssignNoIKMoveProfile::max_steps);
    bind_value_equality(py_LVSAssignNoIKMoveProfile);
    bind_from_yaml(py_LVSAssignNoIKMoveProfile);
}
