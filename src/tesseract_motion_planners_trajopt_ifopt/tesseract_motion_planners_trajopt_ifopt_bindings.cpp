/**
 * @file tesseract_motion_planners_trajopt_ifopt_bindings.cpp
 * @brief nanobind bindings for tesseract_motion_planners TrajOpt IFOPT
 *
 * NOTE: 0.33 API changes:
 * - TrajOptIfoptPlanProfile -> TrajOptIfoptMoveProfile
 * - TrajOptIfoptDefaultPlanProfile -> TrajOptIfoptDefaultMoveProfile
 * - Profile/ProfileDictionary moved to tesseract_common
 */

#include "tesseract_nb.h"

// tesseract_motion_planners core (for PlannerRequest/Response)
#include <tesseract_motion_planners/core/types.h>

// tesseract_common (Profile and ProfileDictionary moved here in 0.33)
#include <tesseract_common/profile.h>
#include <tesseract_common/profile_dictionary.h>

// tesseract_motion_planners TrajOpt IFOPT
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_move_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/profile/trajopt_ifopt_osqp_solver_profile.h>
#include <tesseract_motion_planners/trajopt_ifopt/trajopt_ifopt_waypoint_config.h>

// trajopt_common for collision config
#include <trajopt_common/collision_types.h>

namespace tp = tesseract_planning;
namespace tc = tesseract_common;

NB_MODULE(_tesseract_motion_planners_trajopt_ifopt, m) {
    m.doc() = "tesseract_motion_planners_trajopt_ifopt Python bindings";

    // Import Profile type from tesseract_command_language for cross-module inheritance
    auto cl_module = nb::module_::import_("tesseract_robotics.tesseract_command_language._tesseract_command_language");

    // Import MotionPlanner base type for clone() return type
    nb::module_::import_("tesseract_robotics.tesseract_motion_planners._tesseract_motion_planners");

    // ========== TrajOptIfoptCartesianWaypointConfig ==========
    nb::class_<tp::TrajOptIfoptCartesianWaypointConfig>(m, "TrajOptIfoptCartesianWaypointConfig")
        .def(nb::init<>())
        .def_rw("enabled", &tp::TrajOptIfoptCartesianWaypointConfig::enabled)
        .def_rw("use_tolerance_override", &tp::TrajOptIfoptCartesianWaypointConfig::use_tolerance_override)
        .def_rw("lower_tolerance", &tp::TrajOptIfoptCartesianWaypointConfig::lower_tolerance)
        .def_rw("upper_tolerance", &tp::TrajOptIfoptCartesianWaypointConfig::upper_tolerance)
        .def_rw("coeff", &tp::TrajOptIfoptCartesianWaypointConfig::coeff);

    // ========== TrajOptIfoptJointWaypointConfig ==========
    nb::class_<tp::TrajOptIfoptJointWaypointConfig>(m, "TrajOptIfoptJointWaypointConfig")
        .def(nb::init<>())
        .def_rw("enabled", &tp::TrajOptIfoptJointWaypointConfig::enabled)
        .def_rw("use_tolerance_override", &tp::TrajOptIfoptJointWaypointConfig::use_tolerance_override)
        .def_rw("lower_tolerance", &tp::TrajOptIfoptJointWaypointConfig::lower_tolerance)
        .def_rw("upper_tolerance", &tp::TrajOptIfoptJointWaypointConfig::upper_tolerance)
        .def_rw("coeff", &tp::TrajOptIfoptJointWaypointConfig::coeff);

    // ========== TrajOptIfoptMoveProfile (base, was TrajOptIfoptPlanProfile) ==========
    nb::class_<tp::TrajOptIfoptMoveProfile, tc::Profile>(m, "TrajOptIfoptMoveProfile")
        .def("getKey", &tp::TrajOptIfoptMoveProfile::getKey)
        .def_static("getStaticKey", &tp::TrajOptIfoptMoveProfile::getStaticKey);

    // SWIG-compatible alias
    m.attr("TrajOptIfoptPlanProfile") = m.attr("TrajOptIfoptMoveProfile");

    // ========== TrajOptIfoptCompositeProfile (base) ==========
    nb::class_<tp::TrajOptIfoptCompositeProfile, tc::Profile>(m, "TrajOptIfoptCompositeProfile")
        .def("getKey", &tp::TrajOptIfoptCompositeProfile::getKey)
        .def_static("getStaticKey", &tp::TrajOptIfoptCompositeProfile::getStaticKey);

    // ========== TrajOptIfoptSolverProfile (base) ==========
    nb::class_<tp::TrajOptIfoptSolverProfile, tc::Profile>(m, "TrajOptIfoptSolverProfile")
        .def("getKey", &tp::TrajOptIfoptSolverProfile::getKey)
        .def_static("getStaticKey", &tp::TrajOptIfoptSolverProfile::getStaticKey);

    // ========== TrajOptIfoptDefaultMoveProfile (was TrajOptIfoptDefaultPlanProfile) ==========
    nb::class_<tp::TrajOptIfoptDefaultMoveProfile, tp::TrajOptIfoptMoveProfile>(m, "TrajOptIfoptDefaultMoveProfile")
        .def(nb::init<>())
        .def_rw("cartesian_cost_config", &tp::TrajOptIfoptDefaultMoveProfile::cartesian_cost_config)
        .def_rw("cartesian_constraint_config", &tp::TrajOptIfoptDefaultMoveProfile::cartesian_constraint_config)
        .def_rw("joint_cost_config", &tp::TrajOptIfoptDefaultMoveProfile::joint_cost_config)
        .def_rw("joint_constraint_config", &tp::TrajOptIfoptDefaultMoveProfile::joint_constraint_config);

    // SWIG-compatible alias
    m.attr("TrajOptIfoptDefaultPlanProfile") = m.attr("TrajOptIfoptDefaultMoveProfile");

    // ========== TrajOptIfoptDefaultCompositeProfile ==========
    // Note: longest_valid_segment_fraction/length removed in 0.33, collision config via TrajOptCollisionConfig
    nb::class_<tp::TrajOptIfoptDefaultCompositeProfile, tp::TrajOptIfoptCompositeProfile>(m, "TrajOptIfoptDefaultCompositeProfile")
        .def(nb::init<>())
        .def_rw("smooth_velocities", &tp::TrajOptIfoptDefaultCompositeProfile::smooth_velocities)
        .def_rw("velocity_coeff", &tp::TrajOptIfoptDefaultCompositeProfile::velocity_coeff)
        .def_rw("smooth_accelerations", &tp::TrajOptIfoptDefaultCompositeProfile::smooth_accelerations)
        .def_rw("acceleration_coeff", &tp::TrajOptIfoptDefaultCompositeProfile::acceleration_coeff)
        .def_rw("smooth_jerks", &tp::TrajOptIfoptDefaultCompositeProfile::smooth_jerks)
        .def_rw("jerk_coeff", &tp::TrajOptIfoptDefaultCompositeProfile::jerk_coeff);

    // ========== TrajOptIfoptOSQPSolverProfile ==========
    // Note: This profile has unique_ptr members that can't be easily exposed
    // Users can rely on default configuration
    nb::class_<tp::TrajOptIfoptOSQPSolverProfile, tp::TrajOptIfoptSolverProfile>(m, "TrajOptIfoptOSQPSolverProfile")
        .def(nb::init<>());

    // Helper to add TrajOptIfopt move profile to ProfileDictionary directly
    m.def("ProfileDictionary_addTrajOptIfoptMoveProfile", [](tc::ProfileDictionary& dict,
                                                              const std::string& ns,
                                                              const std::string& profile_name,
                                                              std::shared_ptr<tp::TrajOptIfoptMoveProfile> profile) {
        dict.addProfile(ns, profile_name, profile);
    }, "dict"_a, "ns"_a, "profile_name"_a, "profile"_a,
    "Add TrajOptIfopt move profile to ProfileDictionary");

    // Legacy alias
    m.def("ProfileDictionary_addTrajOptIfoptPlanProfile", [](tc::ProfileDictionary& dict,
                                                              const std::string& ns,
                                                              const std::string& profile_name,
                                                              std::shared_ptr<tp::TrajOptIfoptMoveProfile> profile) {
        dict.addProfile(ns, profile_name, profile);
    }, "dict"_a, "ns"_a, "profile_name"_a, "profile"_a,
    "Add TrajOptIfopt plan profile to ProfileDictionary (legacy alias)");

    // Helper to add TrajOptIfopt composite profile to ProfileDictionary directly
    m.def("ProfileDictionary_addTrajOptIfoptCompositeProfile", [](tc::ProfileDictionary& dict,
                                                                   const std::string& ns,
                                                                   const std::string& profile_name,
                                                                   std::shared_ptr<tp::TrajOptIfoptCompositeProfile> profile) {
        dict.addProfile(ns, profile_name, profile);
    }, "dict"_a, "ns"_a, "profile_name"_a, "profile"_a,
    "Add TrajOptIfopt composite profile to ProfileDictionary");

    // Helper to add TrajOptIfopt solver profile to ProfileDictionary directly
    m.def("ProfileDictionary_addTrajOptIfoptSolverProfile", [](tc::ProfileDictionary& dict,
                                                                const std::string& ns,
                                                                const std::string& profile_name,
                                                                std::shared_ptr<tp::TrajOptIfoptSolverProfile> profile) {
        dict.addProfile(ns, profile_name, profile);
    }, "dict"_a, "ns"_a, "profile_name"_a, "profile"_a,
    "Add TrajOptIfopt solver profile to ProfileDictionary");

    // ========== TrajOptIfoptMotionPlanner ==========
    nb::class_<tp::TrajOptIfoptMotionPlanner>(m, "TrajOptIfoptMotionPlanner")
        .def(nb::init<std::string>(), "name"_a)
        .def("getName", &tp::TrajOptIfoptMotionPlanner::getName)
        .def("solve", &tp::TrajOptIfoptMotionPlanner::solve, "request"_a, nb::call_guard<nb::gil_scoped_release>())
        .def("terminate", &tp::TrajOptIfoptMotionPlanner::terminate)
        .def("clear", &tp::TrajOptIfoptMotionPlanner::clear)
        .def("clone", [](const tp::TrajOptIfoptMotionPlanner& self) { return self.clone(); });
}
