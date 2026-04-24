/**
 * @file tesseract_motion_planners_simple_bindings.cpp
 * @brief nanobind bindings for tesseract_motion_planners simple
 */

#include "tesseract_nb.h"

// tesseract_motion_planners core (for PlannerRequest/Response)
#include <tesseract_motion_planners/core/types.h>

// tesseract_motion_planners simple
#include <tesseract_motion_planners/simple/interpolation.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_move_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_move_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_no_ik_move_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_move_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_no_ik_move_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_assign_move_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_assign_no_ik_move_profile.h>

// tesseract_common (Profile/ProfileDictionary moved here in 0.33)
#include <tesseract_common/profile.h>
#include <tesseract_common/profile_dictionary.h>

// tesseract_environment
#include <tesseract_environment/environment.h>

// tesseract_command_language
#include <tesseract_command_language/composite_instruction.h>

namespace tp = tesseract_planning;
namespace te = tesseract_environment;
namespace tc = tesseract_common;

NB_MODULE(_tesseract_motion_planners_simple, m) {
    m.doc() = "tesseract_motion_planners_simple Python bindings";

    // Import MotionPlanner base type for clone() return type
    nb::module_::import_("tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners");

    // Import Profile base from tesseract_command_language (re-exported from tesseract_common)
    nb::module_::import_("tesseract_robotics.tesseract_command_language._tesseract_command_language");

    // ========== SimpleMotionPlanner ==========
    // Note: Not binding inheritance from MotionPlanner to avoid cross-module issues
    nb::class_<tp::SimpleMotionPlanner>(m, "SimpleMotionPlanner")
        .def(nb::init<std::string>(), "name"_a = "SIMPLE")
        .def("getName", &tp::SimpleMotionPlanner::getName)
        .def("solve", &tp::SimpleMotionPlanner::solve, "request"_a, nb::call_guard<nb::gil_scoped_release>())
        .def("terminate", &tp::SimpleMotionPlanner::terminate)
        .def("clear", &tp::SimpleMotionPlanner::clear)
        .def("clone", [](const tp::SimpleMotionPlanner& self) { return self.clone(); });

    // ========== generateInterpolatedProgram ==========
    m.def("generateInterpolatedProgram",
          [](const tp::CompositeInstruction& instructions,
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

    // ========== SimplePlannerMoveProfile (abstract base) ==========
    nb::class_<tp::SimplePlannerMoveProfile, tc::Profile>(m, "SimplePlannerMoveProfile");

    // ========== SimplePlannerCompositeProfile ==========
    nb::class_<tp::SimplePlannerCompositeProfile, tc::Profile>(m, "SimplePlannerCompositeProfile")
        .def(nb::init<>());

    // ========== SimplePlannerFixedSizeMoveProfile ==========
    nb::class_<tp::SimplePlannerFixedSizeMoveProfile, tp::SimplePlannerMoveProfile>(m, "SimplePlannerFixedSizeMoveProfile")
        .def(nb::init<int, int>(), "freespace_steps"_a = 10, "linear_steps"_a = 10)
        .def_rw("freespace_steps", &tp::SimplePlannerFixedSizeMoveProfile::freespace_steps)
        .def_rw("linear_steps", &tp::SimplePlannerFixedSizeMoveProfile::linear_steps);

    // ========== SimplePlannerFixedSizeAssignMoveProfile ==========
    nb::class_<tp::SimplePlannerFixedSizeAssignMoveProfile, tp::SimplePlannerMoveProfile>(m, "SimplePlannerFixedSizeAssignMoveProfile")
        .def(nb::init<int, int>(), "freespace_steps"_a = 10, "linear_steps"_a = 10)
        .def_rw("freespace_steps", &tp::SimplePlannerFixedSizeAssignMoveProfile::freespace_steps)
        .def_rw("linear_steps", &tp::SimplePlannerFixedSizeAssignMoveProfile::linear_steps);

    // ========== SimplePlannerFixedSizeAssignNoIKMoveProfile ==========
    nb::class_<tp::SimplePlannerFixedSizeAssignNoIKMoveProfile, tp::SimplePlannerMoveProfile>(m, "SimplePlannerFixedSizeAssignNoIKMoveProfile")
        .def(nb::init<int, int>(), "freespace_steps"_a = 10, "linear_steps"_a = 10)
        .def_rw("freespace_steps", &tp::SimplePlannerFixedSizeAssignNoIKMoveProfile::freespace_steps)
        .def_rw("linear_steps", &tp::SimplePlannerFixedSizeAssignNoIKMoveProfile::linear_steps);

    // ========== SimplePlannerLVSMoveProfile ==========
    nb::class_<tp::SimplePlannerLVSMoveProfile, tp::SimplePlannerMoveProfile>(m, "SimplePlannerLVSMoveProfile")
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

    // ========== SimplePlannerLVSNoIKMoveProfile ==========
    nb::class_<tp::SimplePlannerLVSNoIKMoveProfile, tp::SimplePlannerMoveProfile>(m, "SimplePlannerLVSNoIKMoveProfile")
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

    // ========== SimplePlannerLVSAssignMoveProfile ==========
    nb::class_<tp::SimplePlannerLVSAssignMoveProfile, tp::SimplePlannerMoveProfile>(m, "SimplePlannerLVSAssignMoveProfile")
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

    // ========== SimplePlannerLVSAssignNoIKMoveProfile ==========
    nb::class_<tp::SimplePlannerLVSAssignNoIKMoveProfile, tp::SimplePlannerMoveProfile>(m, "SimplePlannerLVSAssignNoIKMoveProfile")
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

    // ========== ProfileDictionary helpers ==========
    m.def("ProfileDictionary_addSimplePlannerMoveProfile",
          [](tc::ProfileDictionary& dict, const std::string& ns, const std::string& profile_name,
             std::shared_ptr<tp::SimplePlannerMoveProfile> profile) {
              dict.addProfile(ns, profile_name, profile);
          }, "dict"_a, "ns"_a, "profile_name"_a, "profile"_a,
          "Add a simple planner move profile to ProfileDictionary");

    m.def("ProfileDictionary_addSimplePlannerCompositeProfile",
          [](tc::ProfileDictionary& dict, const std::string& ns, const std::string& profile_name,
             std::shared_ptr<tp::SimplePlannerCompositeProfile> profile) {
              dict.addProfile(ns, profile_name, profile);
          }, "dict"_a, "ns"_a, "profile_name"_a, "profile"_a,
          "Add a simple planner composite profile to ProfileDictionary");
}
