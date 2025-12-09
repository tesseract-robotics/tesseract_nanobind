/**
 * @file tesseract_motion_planners_trajopt_bindings.cpp
 * @brief nanobind bindings for tesseract_motion_planners TrajOpt
 */

#include "tesseract_nb.h"

// tesseract_motion_planners core (for PlannerRequest/Response)
#include <tesseract_motion_planners/core/types.h>

// tesseract_common (for Profile base class and ProfileDictionary)
#include <tesseract_common/profile.h>
#include <tesseract_common/profile_dictionary.h>

// tesseract_motion_planners TrajOpt (0.33.x: Plan -> Move profile rename)
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_move_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/trajopt_waypoint_config.h>

// trajopt_common for TrajOptCollisionConfig
#include <trajopt_common/collision_types.h>

// tesseract_collision for ContactTestType
#include <tesseract_collision/core/types.h>

namespace tp = tesseract_planning;
namespace tc = tesseract_common;

NB_MODULE(_tesseract_motion_planners_trajopt, m) {
    m.doc() = "tesseract_motion_planners_trajopt Python bindings";

    // Import tesseract_common for Profile base class (cross-module inheritance)
    nb::module_::import_("tesseract_robotics.tesseract_common._tesseract_common");

    // Import tesseract_collision for CollisionEvaluatorType and other types
    nb::module_::import_("tesseract_robotics.tesseract_collision._tesseract_collision");

    // Note: CollisionEvaluatorType enum is bound in tesseract_collision (moved there in 0.33.x)

    // ========== TrajOptCartesianWaypointConfig ==========
    nb::class_<tp::TrajOptCartesianWaypointConfig>(m, "TrajOptCartesianWaypointConfig")
        .def(nb::init<>())
        .def_rw("enabled", &tp::TrajOptCartesianWaypointConfig::enabled)
        .def_rw("use_tolerance_override", &tp::TrajOptCartesianWaypointConfig::use_tolerance_override)
        .def_rw("lower_tolerance", &tp::TrajOptCartesianWaypointConfig::lower_tolerance)
        .def_rw("upper_tolerance", &tp::TrajOptCartesianWaypointConfig::upper_tolerance)
        .def_rw("coeff", &tp::TrajOptCartesianWaypointConfig::coeff);

    // ========== TrajOptJointWaypointConfig ==========
    nb::class_<tp::TrajOptJointWaypointConfig>(m, "TrajOptJointWaypointConfig")
        .def(nb::init<>())
        .def_rw("enabled", &tp::TrajOptJointWaypointConfig::enabled)
        .def_rw("use_tolerance_override", &tp::TrajOptJointWaypointConfig::use_tolerance_override)
        .def_rw("lower_tolerance", &tp::TrajOptJointWaypointConfig::lower_tolerance)
        .def_rw("upper_tolerance", &tp::TrajOptJointWaypointConfig::upper_tolerance)
        .def_rw("coeff", &tp::TrajOptJointWaypointConfig::coeff);

    // ========== TrajOptCollisionConfig (replaces CollisionCostConfig/CollisionConstraintConfig in 0.33.x) ==========
    nb::class_<trajopt_common::TrajOptCollisionConfig>(m, "TrajOptCollisionConfig")
        .def(nb::init<>())
        .def_rw("enabled", &trajopt_common::TrajOptCollisionConfig::enabled)
        .def_rw("contact_manager_config", &trajopt_common::TrajOptCollisionConfig::contact_manager_config)
        .def_rw("collision_check_config", &trajopt_common::TrajOptCollisionConfig::collision_check_config)
        .def_rw("collision_coeff_data", &trajopt_common::TrajOptCollisionConfig::collision_coeff_data)
        .def_rw("collision_margin_buffer", &trajopt_common::TrajOptCollisionConfig::collision_margin_buffer)
        .def_rw("max_num_cnt", &trajopt_common::TrajOptCollisionConfig::max_num_cnt);

    // Aliases for backwards compatibility (0.28.x API)
    m.attr("CollisionCostConfig") = m.attr("TrajOptCollisionConfig");
    m.attr("CollisionConstraintConfig") = m.attr("TrajOptCollisionConfig");

    // ========== TrajOptMoveProfile (base) - renamed from TrajOptPlanProfile in 0.33.x ==========
    nb::class_<tp::TrajOptMoveProfile, tc::Profile>(m, "TrajOptMoveProfile")
        .def("getKey", &tp::TrajOptMoveProfile::getKey)
        .def_static("getStaticKey", &tp::TrajOptMoveProfile::getStaticKey);

    // Alias for backwards compatibility
    m.attr("TrajOptPlanProfile") = m.attr("TrajOptMoveProfile");

    // ========== TrajOptCompositeProfile (base) ==========
    nb::class_<tp::TrajOptCompositeProfile, tc::Profile>(m, "TrajOptCompositeProfile")
        .def("getKey", &tp::TrajOptCompositeProfile::getKey)
        .def_static("getStaticKey", &tp::TrajOptCompositeProfile::getStaticKey);

    // ========== TrajOptDefaultMoveProfile - renamed from TrajOptDefaultPlanProfile in 0.33.x ==========
    nb::class_<tp::TrajOptDefaultMoveProfile, tp::TrajOptMoveProfile>(m, "TrajOptDefaultMoveProfile")
        .def(nb::init<>())
        .def_rw("cartesian_cost_config", &tp::TrajOptDefaultMoveProfile::cartesian_cost_config)
        .def_rw("cartesian_constraint_config", &tp::TrajOptDefaultMoveProfile::cartesian_constraint_config)
        .def_rw("joint_cost_config", &tp::TrajOptDefaultMoveProfile::joint_cost_config)
        .def_rw("joint_constraint_config", &tp::TrajOptDefaultMoveProfile::joint_constraint_config);

    // Alias for backwards compatibility
    m.attr("TrajOptDefaultPlanProfile") = m.attr("TrajOptDefaultMoveProfile");

    // ========== TrajOptDefaultCompositeProfile ==========
    nb::class_<tp::TrajOptDefaultCompositeProfile, tp::TrajOptCompositeProfile>(m, "TrajOptDefaultCompositeProfile")
        .def(nb::init<>())
        .def_rw("collision_cost_config", &tp::TrajOptDefaultCompositeProfile::collision_cost_config)
        .def_rw("collision_constraint_config", &tp::TrajOptDefaultCompositeProfile::collision_constraint_config)
        .def_rw("smooth_velocities", &tp::TrajOptDefaultCompositeProfile::smooth_velocities)
        .def_rw("velocity_coeff", &tp::TrajOptDefaultCompositeProfile::velocity_coeff)
        .def_rw("smooth_accelerations", &tp::TrajOptDefaultCompositeProfile::smooth_accelerations)
        .def_rw("acceleration_coeff", &tp::TrajOptDefaultCompositeProfile::acceleration_coeff)
        .def_rw("smooth_jerks", &tp::TrajOptDefaultCompositeProfile::smooth_jerks)
        .def_rw("jerk_coeff", &tp::TrajOptDefaultCompositeProfile::jerk_coeff)
        .def_rw("avoid_singularity", &tp::TrajOptDefaultCompositeProfile::avoid_singularity)
        .def_rw("avoid_singularity_coeff", &tp::TrajOptDefaultCompositeProfile::avoid_singularity_coeff);

    // Helper to add TrajOpt move profile to ProfileDictionary directly
    m.def("ProfileDictionary_addTrajOptMoveProfile", [](tc::ProfileDictionary& dict,
                                                         const std::string& ns,
                                                         const std::string& profile_name,
                                                         std::shared_ptr<tp::TrajOptMoveProfile> profile) {
        dict.addProfile(ns, profile_name, profile);
    }, "dict"_a, "ns"_a, "profile_name"_a, "profile"_a,
    "Add TrajOpt move profile to ProfileDictionary");

    // Alias for backwards compatibility
    m.attr("ProfileDictionary_addTrajOptPlanProfile") = m.attr("ProfileDictionary_addTrajOptMoveProfile");

    // Helper to add TrajOpt composite profile to ProfileDictionary directly
    m.def("ProfileDictionary_addTrajOptCompositeProfile", [](tc::ProfileDictionary& dict,
                                                              const std::string& ns,
                                                              const std::string& profile_name,
                                                              std::shared_ptr<tp::TrajOptCompositeProfile> profile) {
        dict.addProfile(ns, profile_name, profile);
    }, "dict"_a, "ns"_a, "profile_name"_a, "profile"_a,
    "Add TrajOpt composite profile to ProfileDictionary");

    // ========== TrajOptMotionPlanner ==========
    nb::class_<tp::TrajOptMotionPlanner>(m, "TrajOptMotionPlanner")
        .def(nb::init<std::string>(), "name"_a)
        .def("getName", &tp::TrajOptMotionPlanner::getName)
        .def("solve", &tp::TrajOptMotionPlanner::solve, "request"_a)
        .def("terminate", &tp::TrajOptMotionPlanner::terminate)
        .def("clear", &tp::TrajOptMotionPlanner::clear);
}
